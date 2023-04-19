# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial
import io
import os
import select

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver

class FixedSerial(serial.Serial):
    # seria.Serial.read() with removed "if timeout.expired():" line.
    # With this change, the function provides classical POSIX
    # semantics of read(), which returns as soon as some bytes are
    # received. The implementation in pySerial either waits for the
    # full buffer or timeout expiration - nothing in between. This
    # means that with pySerial, you either pay big overherd by reading
    # character-by-character or have increased latency due to waiting
    # for timeout expiration.
    def read(self, size=1):
        """\
        Read size bytes from the serial port. If a timeout is set it may
        return less characters as requested. With no timeout it will block
        until the requested number of bytes is read.
        """
        if not self.is_open:
            raise PortNotOpenError()
        read = bytearray()
        timeout = serial.Timeout(self._timeout)
        while len(read) < size:
            try:
                ready, _, _ = select.select([self.fd, self.pipe_abort_read_r], [], [], timeout.time_left())
                if self.pipe_abort_read_r in ready:
                    os.read(self.pipe_abort_read_r, 1000)
                    break
                # If select was used with a timeout, and the timeout occurs, it
                # returns with empty lists -> thus abort read operation.
                # For timeout == 0 (non-blocking operation) also abort when
                # there is nothing to read.
                if not ready:
                    break   # timeout
                buf = os.read(self.fd, size - len(read))
            except OSError as e:
                # this is for Python 3.x where select.error is a subclass of
                # OSError ignore BlockingIOErrors and EINTR. other errors are shown
                # https://www.python.org/dev/peps/pep-0475.
                if e.errno not in (errno.EAGAIN, errno.EALREADY, errno.EWOULDBLOCK, errno.EINPROGRESS, errno.EINTR):
                    raise SerialException('read failed: {}'.format(e))
            except select.error as e:
                # this is for Python 2.x
                # ignore BlockingIOErrors and EINTR. all errors are shown
                # see also http://www.python.org/dev/peps/pep-3151/#select
                if e[0] not in (errno.EAGAIN, errno.EALREADY, errno.EWOULDBLOCK, errno.EINPROGRESS, errno.EINTR):
                    raise SerialException('read failed: {}'.format(e))
            else:
                # read should always return some data as select reported it was
                # ready to read when we get to this point.
                if not buf:
                    # Disconnected devices, at least on Linux, show the
                    # behavior that they are always ready to read immediately
                    # but reading returns nothing.
                    raise SerialException(
                        'device reports readiness to read but returned no data '
                        '(device disconnected or multiple access on port?)')
                read.extend(buf)
                break
        return bytes(read)


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    frame_id = driver.get_frame_id()

    serial_port = driver.declare_parameter('port', '/dev/ttyUSB0').value
    serial_baud = driver.declare_parameter('baud', 4800).value

    try:
        GPS = FixedSerial(port=serial_port, baudrate=serial_baud, timeout=2)
        driver.get_logger().info("Successfully connected to {0} at {1}.".format(serial_port, serial_baud))
        try:
            data = bytearray()
            while rclpy.ok():
                data.extend(GPS.read(1024)) # read at most 1024 bytes
                lines = data.splitlines(keepends=True)
                data.clear()
                def process_line(line):
                    line = line.decode("utf-8").rstrip()
                    try:
                        driver.add_sentence(line, frame_id)
                    except ValueError as e:
                        driver.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                            "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                            "with the NMEA sentences that caused it." % e)
                # process complete lines
                for line in lines[0:-1]:
                    process_line(line)
                if lines[-1].endswith(b'\r\n'):
                    process_line(lines[-1])
                else:
                    # continue with the last, incomplete line
                    data = lines[-1]
        except Exception as e:
            driver.get_logger().error("Ros error: {0}".format(e))
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        driver.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))

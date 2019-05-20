# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Rein Appeldoorn
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

import socket
import sys

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver


def main(args=None):
    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    try:
        local_ip = driver.get_parameter('ip').value or '0.0.0.0'
        local_port = driver.get_parameter('port').value or 10110
        buffer_size = driver.get_parameter('buffer_size').value or 4096
        timeout = driver.get_parameter('timeout_sec').value or 2
    except KeyError as e:
        driver.get_logger().err("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    while rclpy.ok():
        try:
            # Create a socket
            socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Bind the socket to the port
            socket_.bind((local_ip, local_port))

            # Set timeout
            socket_.settimeout(timeout)
        except socket.error as exc:
            rclpy.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            sys.exit(1)

        # recv-loop: When we're connected, keep receiving stuff until that fails
        while rclpy.ok():
            try:
                data, remote_address = socket_.recvfrom(buffer_size)

                # strip the data
                data_list = data.strip().split("\n")

                for data in data_list:

                    try:
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        rclpy.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, "
                            "including a bag file with the NMEA sentences that caused it." % e)

            except socket.error as exc:
                driver.get_logger().error("Caught exception socket.error during recvfrom: %s" % exc)
                socket_.close()
                # This will break out of the recv-loop so we start another iteration of the connection-loop
                break

        socket_.close()  # Close socket

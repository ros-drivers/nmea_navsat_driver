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


from functools import partial

from nmea_msgs.msg import Sentence
import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver


def nmea_sentence_callback(nmea_sentence, driver):
    try:
        driver.add_sentence(nmea_sentence.sentence, frame_id=nmea_sentence.header.frame_id,
                            timestamp=nmea_sentence.header.stamp)
    except ValueError as e:
        rclpy.get_logger().warn(
            "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
            "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file with "
            "the NMEA sentences that caused it." % e)


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    driver.get_frame_id()

    driver.create_subscription(
        Sentence, 'nmea_sentence', partial(nmea_sentence_callback, driver=driver), 10)

    rclpy.spin(driver)

    rclpy.shutdown()

#!/usr/bin/env python3
"""Testing node for nmea_navsat_driver."""

# Software License Agreement (BSD License)
#
# Copyright (c) 2020-2021, Robert Bosch GmbH
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

import io
import os
import socket
import subprocess
import sys
import threading
import unittest
from contextlib import redirect_stdout

import rospkg
import rospy
import rostest
import serial
import virtualserialports
import yaml
from sensor_msgs.msg import *


class TestDriver(unittest.TestCase):
    """Test nmea_navsat_driver."""

    def initialize(self):
        """Initialize class."""
        with open(rospkg.RosPack().get_path('nmea_navsat_driver') + '/test/configs/config_log.yaml') as f:
            self.cfg_logs = yaml.safe_load(f)
        with open(rospkg.RosPack().get_path('nmea_navsat_driver') + '/test/configs/config_launch.yaml') as f:
            self.cfg_launches = yaml.safe_load(f)
        with open(rospkg.RosPack().get_path('nmea_navsat_driver') + '/test/configs/config_tester.yaml') as f:
            self.cfg_tester = yaml.safe_load(f)

        self.test_failed = False

        # tcp ports can't be re-used in quick succession, therefore cache it
        # here to increment it later
        self.tcp_port = self.cfg_tester['tcp_port']

    def cb(self, msg, args):
        """Calback for all messages."""
        log, topic = args

        if topic not in self.msg_counter.keys():
            self.msg_counter[topic] = 0

        self.msg_counter[topic] += 1
        self.check_msg(msg, log, topic)

    def check_msg(self, msg, log, topic):
        """Check whether message content complies with target specs."""
        topics = self.cfg_logs[log]['topics']

        if topic in topics.keys():
            for attribute in topics[topic]:
                if not attribute.startswith('_'):
                    val = getattr(msg, attribute)

                    if 'min' in topics[topic][attribute].keys():
                        if val < topics[topic][attribute]['min']:
                            self.errors.append(
                                '{} {} {} {}: violating minimium value {}: {}'.format(log, msg.header.stamp.to_sec(),
                                                                                      topic, attribute,
                                                                                      topics[topic][attribute]['min'],
                                                                                      val))
                    if 'max' in topics[topic][attribute].keys():
                        if val > topics[topic][attribute]['max']:
                            self.errors.append(
                                '{} {} {} {}: violating maximum value {}: {}'.format(log, msg.header.stamp.to_sec(),
                                                                                     topic, attribute,
                                                                                     topics[topic][attribute]['max'],
                                                                                     val))
                    if 'val' in topics[topic][attribute].keys():
                        if val != topics[topic][attribute]['val']:
                            self.errors.append(
                                '{} {} {} {}: violating value {}: {}'.format(log, msg.header.stamp.to_sec(), topic,
                                                                             attribute, topics[topic][attribute]['val'],
                                                                             val))

    @staticmethod
    def create_vsps():
        """Create virtual serial ports (vsp) for serial driver testing."""
        ports = []
        f = io.StringIO()
        with redirect_stdout(f):
            th_vsp = threading.Thread(target=virtualserialports.run, args=(2, False, False))
            th_vsp.daemon = True
            th_vsp.start()

            rospy.sleep(2.)

        while len(ports) < 2:
            ports = f.getvalue().split('\n')[:-1]

            if len(ports) < 2:
                rospy.logwarn('Virtual serial ports not ready yet, waiting...')
                rospy.sleep(2.)

        return ports

    def playback_log(self, launchfile, log):
        """Playback a logfile."""
        playback_path = rospkg.RosPack().get_path('nmea_navsat_driver') + '/test/logs/' + log

        if self.cfg_launches[launchfile]['interface'] == 'serial':
            if self.cfg_logs[log]['mode'] == 'nmea':
                with open(playback_path, 'r') as f:
                    for line in f:
                        self.serial_writer.write(str(line).encode('utf-8'))
                        rospy.sleep(1. / self.cfg_logs[log]['rate'] / self.cfg_tester['speedup_rate'])
            elif self.cfg_logs[log]['mode'] == 'binary':
                with open(playback_path, 'rb') as f:
                    byte = f.read(self.cfg_tester['binary_read_len'])
                    while byte:
                        self.serial_writer.write(byte)
                        rospy.sleep(1. / self.cfg_logs[log]['rate'] / self.cfg_tester['speedup_rate'] *
                                    self.cfg_tester['binary_read_len'])
                        byte = f.read(self.cfg_tester['binary_read_len'])
        elif self.cfg_launches[launchfile]['interface'] == 'tcp':
            if self.cfg_logs[log]['mode'] == 'nmea':
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((self.cfg_tester['tcp_host'], self.tcp_port))
                    s.listen()
                    conn, addr = s.accept()
                    with conn:
                        rospy.loginfo('TCP driver connected')
                        if self.cfg_logs[log]['mode'] == 'nmea':
                            with open(playback_path, 'r') as f:
                                for line in f:
                                    conn.send(str(line).encode('utf-8'))
                                    rospy.sleep(1. / self.cfg_logs[log]['rate'] / self.cfg_tester['speedup_rate'])

                self.tcp_port += 1

    def print_errors(self, launchfile, log, msg_types):
        """Print errors."""
        for topic in self.cfg_logs[log]['topics']:
            msg_count_target = self.cfg_logs[log]['topics'][topic]['_message_count']
            if msg_types[topic] not in self.cfg_launches[launchfile]['messages']:
                rospy.loginfo(
                    '{}: skipping topic {} since not in scope for launch file {}'.format(log, topic, launchfile))
            elif topic not in self.msg_counter:
                self.errors.append('{}: topic "{}" not published for log'.format(log, topic))
            elif self.msg_counter[topic] != msg_count_target:
                self.errors.append(
                    '{}: unmet target message count {}: {} for topic {}'.format(log, msg_count_target,
                                                                                self.msg_counter[topic], topic))

        if len(self.errors) == 0:
            rospy.logwarn('{}: no errors detected'.format(log))
        else:
            for error in self.errors:
                rospy.logerr(error)

    def test(self):
        """Run the tester."""
        self.initialize()

        # init ros
        ros_master_uri = 'http://127.0.0.1:11311'
        env_variables = os.environ.copy()
        env_variables['ROS_MASTER_URI'] = ros_master_uri
        p_roscore = subprocess.Popen(['roscore'], env=env_variables)
        rospy.init_node('nmea_navsat_driver_tester')

        # init serial handler
        ports = self.create_vsps()
        self.serial_writer = serial.Serial(ports[0], 115200)

        # process all launch files
        for launchfile in self.cfg_launches:
            rospy.loginfo('========== Testing launch file {} =========='.format(launchfile))

            # process all logs
            for log in self.cfg_logs:
                # tcp interface doesn't support binary mode
                if self.cfg_launches[launchfile]['interface'] == 'tcp' and self.cfg_logs[log]['mode'] == 'binary':
                    rospy.loginfo('Skipping log {} for TCP driver'.format(log))
                    continue

                rospy.loginfo('---------- Processing logfile {} ----------'.format(log))
                self.errors = []
                self.msg_counter = {}
                msg_types = {}
                subscribers = []

                # create subscribers
                for topic in self.cfg_logs[log]['topics']:
                    msg_type = self.cfg_logs[log]['topics'][topic]['_type']
                    msg_types[topic] = msg_type
                    subscribers.append(
                        rospy.Subscriber(
                            '/' + topic,
                            getattr(sys.modules[__name__], msg_type),
                            self.cb,
                            [log, topic]
                        )
                    )

                # launch
                if self.cfg_launches[launchfile]['interface'] == 'serial':
                    p_driver = subprocess.Popen(
                        [
                            'roslaunch',
                            'nmea_navsat_driver',
                            launchfile,
                            '--wait'
                        ] +
                        self.cfg_logs[log]['parameters'] +
                        [
                            'port:=' + ports[1],
                            'baud:=115200'
                        ]
                    )
                elif self.cfg_launches[launchfile]['interface'] == 'tcp':
                    p_driver = subprocess.Popen(
                        [
                            'roslaunch',
                            'nmea_navsat_driver',
                            launchfile,
                            '--wait'
                        ] +
                        self.cfg_logs[log]['parameters'] +
                        [
                            'ip:=' + self.cfg_tester['tcp_host'],
                            'port:=' + str(self.tcp_port),
                        ]
                    )
                else:
                    rospy.logerr('Unknown interface: {}'.format(self.cfg_launches[launchfile]['interface']))
                    continue

                # wait until driver is running
                rospy.sleep(2.)

                # playback log
                self.playback_log(launchfile, log)

                # wait until last message is processed, assuming 2 seconds is enough
                rospy.sleep(2.)

                # stop driver
                p_driver.terminate()

                # let the driver shut down
                rospy.sleep(2.)

                # stop subscribers
                for subscriber in subscribers:
                    subscriber.unregister()

                self.print_errors(launchfile, log, msg_types)

                if len(self.errors) > 0:
                    self.test_failed = True

        self.serial_writer.close()
        p_roscore.terminate()

        self.assertFalse(self.test_failed)


if __name__ == '__main__':
    rostest.rosrun('nmea_navsat_driver', 'driver_tester', TestDriver)

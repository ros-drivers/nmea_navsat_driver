# Created by Lucas Neuber
# This is for you, Köter :P

import socket;
import sys;

import rclpy;

from libnmea_navsat_driver.driver import Ros2NMEADriver;

def main(args=None):
    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    try:
        gnss_ip = driver.declare_parameter('ip', '192.168.131.22').value
        gnss_port = driver.declare_parameter('port', 9001).value
        buffer_size = driver.declare_parameter('buffer_size', 4096).value
    except KeyError as e:
        driver.get_logger().err("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info("Using gnss sensor with ip {} and port {}".format(gnss_ip, gnss_port))

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    # Connect to the gnss sensor using tcp
    while rclpy.ok():
        try:
            # Create a socket
            gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect to the gnss sensor
            gnss_socket.connect((gnss_ip, gnss_port))
        except socket.error as exc:
            driver.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            sys.exit(1)

        # recv-loop: When we're connected, keep receiving stuff until that fails
        while rclpy.ok():
            try:
                data = gnss_socket.recv(buffer_size)

                # strip the data
                data_list = data.decode("ascii").strip().split("\n")

                # remove any incomplete messages
                data_list = [d for d in data_list if d[-1] == '*']

                for data in data_list:
                    try:
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        driver.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. Please report this issue to me. " % e)


            except socket.error as exc:
                driver.get_logger().error("Caught exception socket.error when receiving: %s" % exc)
                gnss_socket.close()
                break


        gnss_socket.close()

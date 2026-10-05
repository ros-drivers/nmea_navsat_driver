nmea_navsat_driver
===============

ROS driver to parse NMEA strings and publish ``gps_msgs/GPSFix`` on the ``fix`` topic,
along with standard ROS velocity and time messages. Does not require the GPSD daemon
to be running. True heading from HDT sentences is published on ``heading`` and updates
the ``dip`` field on ``fix``.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver

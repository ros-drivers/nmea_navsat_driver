^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nmea_navsat_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2020-02-23)
------------------
* Use Python's SocketServer rather than low level socket APIs. (`#92 <https://github.com/evenator/nmea_navsat_driver/issues/92>`_)
  This simplifies code and makes it easier to add TCP support in the future. The ``buffer_size`` parameter is no longer necessary because this is an internal detail of UDPServer.
* Add documentation that passes ``pydocstyle``. (`#88 <https://github.com/evenator/nmea_navsat_driver/issues/88>`_)
* Add an Option to Use GNSS Time and Improve Time Parsing. (`#79 <https://github.com/evenator/nmea_navsat_driver/issues/79>`_)

  - Add an optional parameter ``use_GNSS_time`` to use the time from the GPS sentences for ROS message time instead of using system time.
  - Improve GPS time parsing to support nanosecond precision on devices that support it.
  - Improve GPS time parsing to use RMC message for date when available.
  - Improve GPS time parsing to resolve ambiguities in date and century using system time.
* Refactor all nodes into entrypoint scripts. (`#76 <https://github.com/evenator/nmea_navsat_driver/issues/76>`_).
  This will reduce the difference between ROS 1 and ROS 2 code, because ROS 2 uses Python entry points to install executables.
* Fix PEP8 Violations (`#68 <https://github.com/evenator/nmea_navsat_driver/issues/68>`_). All Python modules and scripts now pass ``pycodestyle --max-line-length 120 src/libnmea_navsat_driver/ scripts/*``
* Add ``nmea_serial_driver`` launch file (`#60 <https://github.com/evenator/nmea_navsat_driver/issues/60>`_)
* Removed ``roslint`` as build depend. (`#59 <https://github.com/evenator/nmea_navsat_driver/issues/59>`_)
  ``roslint`` was accidentally re-added as a build dependency in `#25 <https://github.com/evenator/nmea_navsat_driver/issues/25>`_.
* Contributors: Ed Venator, Ryan Govostes, Tony Baltovski, Xiangyang Zhi, diasdm

0.5.1 (2018-12-30)
------------------
* Add support for IMU aided GPS systems like the Applanix POS/MV, whose NMEA strings typically begin '$IN'. (e.g. $INGGA). Add support for VTG messages, which contain Course Over Ground and Speed Made Good. These are useful when not using RMC messages and you don't have a heading sensor. (`#30 <https://github.com/ros-drivers/nmea_navsat_driver/issues/30>`_/`#58 <https://github.com/ros-drivers/nmea_navsat_driver/issues/58>`_)
* Add a NMEA socket driver node, which is like the existing serial driver node, but instead of attaching to a TTY handle from a serial port, it listens to a UDP port for NMEA sentences. (`#32 <https://github.com/ros-drivers/nmea_navsat_driver/issues/32>`_)
* Add code to handle serial exception to allow node to exit cleanly (`#52 <https://github.com/ros-drivers/nmea_navsat_driver/issues/52>`_)
* Cleanup CMakeLists, package.xml; using package format 2. (`#28 <https://github.com/ros-drivers/nmea_navsat_driver/issues/28>`_)
* Update maintainer to Ed Venator (`#38 <https://github.com/ros-drivers/nmea_navsat_driver/issues/38>`_)
* Add GLONASS support
* Updated driver to accept status of 9 which some novatel recievers report for a WAAS (SBAS) fix.
  See http://www.novatel.com/support/known-solutions/which-novatel-position-types-correspond-to-the-gga-quality-indicator/
* Contributors: Ed Venator, Edward Venator, Eric Perko, Loy, Mike Purvis, Patrick Barone, Timo Röhling, Vikrant Shah

0.5.0 (2015-04-23)
------------------
* Release to Jade.

0.4.2 (2015-04-23)
------------------
* Fix remaining parse problem with NovAtel receivers (empty field specified for num_satellite).

0.4.1 (2014-08-03)
------------------
* Add debug logging output to the parser (PR #8, Mike Purvis)
* Add queue size arguement to publishers to fix warning on Indigo (PR #9, Mike Purvis)
* Add support for roslint and some related cleanup (PR #10, Mike Purvis)

0.4.0 (2014-05-04)
-------------------
* Initial release for Indigo
* Fix #5: Empty fields spam rosout with warnings. Driver now outputs sensor_msgs/NavSatFix messages that may contain NaNs in position and covariance when receiving invalid fixes from the device.

0.3.3 (2013-10-08)
-------------------
* Allow the driver to output velocity information anytime an RMC message is received

0.3.2 (2013-07-21)
-------------------
* Moved to nmea_navsat_driver package
* Removed .py extensions from new-in-Hydro scripts
* Now uses nmea_msgs/Sentence instead of custom sentence type
* nmea_topic_driver reads the `frame_id` parameter from the sentence, not from the parameter server

0.3.1 (2013-05-07)
-------------------
* Removed incorrect find_package dependencies

0.3.0 (2013-05-05)
-------------------
* Initial release for Hydro
* Converted to Catkin
* nmea_gps_driver.py is now deprecated and will be removed in I-Turtle. Replacement node is nmea_serial_driver.py .
* Refactored code into NMEA parser, common ROS driver and separate nodes for reading directly from serial or from topic.
* Bugs fixed:
  - nmea_gps_driver crashes when a sentence doesn't have a checksum * character ( http://kforge.ros.org/gpsdrivers/trac/ticket/4 )
  - Add ability for nmea_gps_driver to support reading from string topic ( https://github.com/ros-drivers/nmea_gps_driver/issues/1 ). Use the nmea_topic_driver.py node to get this support.

0.2.0 (2012-03-15)
------------------
* Initial version (released into Fuerte)
* Supports GGA or RMC+GSA sentences to generate sensor_msgs/NavSatFix messages

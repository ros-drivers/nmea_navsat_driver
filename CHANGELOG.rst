^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nmea_navsat_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2022-06-25)
------------------

* Replace dependency on transforms3d pip package to tf_transformations. (`#147 <https://github.com/evenator/nmea_navsat_driver/issues/147>`_)
* Added log for successful connection. (`#149 <https://github.com/evenator/nmea_navsat_driver/issues/149>`_)
* Removed printing every time the checksum is checked. (`#148 <https://github.com/evenator/nmea_navsat_driver/issues/148>`_)
* nmea_socket_driver fixes for ROS2 (`#127 <https://github.com/evenator/nmea_navsat_driver/issues/127>`_)
  * Fix white space for PEP8 compliance
  * Fix socket driver for ROS2 Foxy
  * Add config file for socket driver
  * Decode bytes as ASCII
* Fix bug where ROS node is not assigned correctly (`#114 <https://github.com/evenator/nmea_navsat_driver/issues/114>`_)
  Fixes `#71 <https://github.com/evenator/nmea_navsat_driver/issues/71>`_
* Update launch file for ROS2 Foxy (`#110 <https://github.com/evenator/nmea_navsat_driver/issues/110>`_)
  * Remove get_default_launch_description, which no longer exists.
  * Update the parameters of the Node initializer to the Foxy API.
* Update pyserial dependency to make it findable with rosdep (`#109 <https://github.com/evenator/nmea_navsat_driver/issues/109>`_)
  This allows rosdep to find the correct serial package and install it.
* ROS2: Eloquent changes for python packages (`#101 <https://github.com/evenator/nmea_navsat_driver/issues/101>`_)
  Fix some warnings when installing packages that first appear in ROS2 Eloquent:
  - A missing the resource folder
  - Not explicitly installing package.xml in the share folder.
* ROS2 updates for Dashing (`#80 <https://github.com/evenator/nmea_navsat_driver/issues/80>`_)
  Change subscriptions and parameters to conform to API changes in ROS 2 Dashing.
* Remove scripts directory in favor of nodes subpackage (`#77 <https://github.com/evenator/nmea_navsat_driver/issues/77>`_)
  Since these modules will no longer be called as scripts in ROS 2,
  move them to a nodes subpackage, and remove the option to call them
  as executables.
  Addresses `#75 <https://github.com/evenator/nmea_navsat_driver/issues/75>`_ for ROS 2.
* Fix nmea_topic_serial_reader name (`#74 <https://github.com/evenator/nmea_navsat_driver/issues/74>`_)
  - Fix `#72 <https://github.com/evenator/nmea_navsat_driver/issues/72>`_ (no module name scripts.nmea_topic_serial_driver)
  - Fix exception handling in nmea_topic_serial_reader ('rclpy' has no attribute 'ROSInterruptException')
* Clean up launch file and make it runnable with ROS2 launch (`#73 <https://github.com/evenator/nmea_navsat_driver/issues/73>`_)
  Fixes `#70 <https://github.com/evenator/nmea_navsat_driver/issues/70>`_
  - Rename config file and put it into a config directory.
  - Make setup.py install the launch and config files.
  - Rename the launch file with the .launch.py suffix used by OSRF
  packages.
  - Refactor the launch file so that it works with `ros2 launch`.
* Fix PEP8 Violations and update setup.cfg file for pycodestyle. (`#69 <https://github.com/evenator/nmea_navsat_driver/issues/69>`_)
* Port to ROS 2 (`#64 <https://github.com/evenator/nmea_navsat_driver/issues/64>`_)
  Initial work to port nmea_navsat_driver to ROS2 by @klintan.
* Add nmea_serial_driver launch file (`#60 <https://github.com/evenator/nmea_navsat_driver/issues/60>`_)
  Add example nmea_serial_driver launch file.
* Remove automatic prefixing of forward slash to frame_id. (`#33 <https://github.com/evenator/nmea_navsat_driver/issues/33>`_/`#57 <https://github.com/evenator/nmea_navsat_driver/issues/57>`_)
  To be consistent with the current default behavior, the default frame_id has been set to /gps with the prepended forward slash.
* Add support for IMU aided GPS systems (`#30 <https://github.com/evenator/nmea_navsat_driver/issues/30>`_/`#58 <https://github.com/evenator/nmea_navsat_driver/issues/58>`_)
  * Add support for IMU aided GPS systems like the Applanix POS/MV, whose NMEA strings typically begin '$IN'. (e.g. $INGGA).
  * Add support for VTG messages, which contain Course Over Ground and Speed Made Good. These are useful when not using RMC messages and you don't have a heading sensor.
* Add support for publishing heading from GPHDT as a QuaternionStamped message on the topic /heading (`#25 <https://github.com/evenator/nmea_navsat_driver/issues/25>`_)
* Improve Covariance Estimation (`#46 <https://github.com/evenator/nmea_navsat_driver/issues/46>`_)
  Use GST covariance where available, otherwise uses default covariances estimated from fix type.
  The previous implementation set covariance to HDOP^2. Instead, it should multiply that by the measurement variance. HDOP should be greater than 1.0.
* Add Socket Driver (`#32 <https://github.com/evenator/nmea_navsat_driver/issues/32>`_)
  Add a NMEA socket driver node, which is like the existing serial driver node, but instead of attaching to a TTY handle from a serial port, it listens to a UDP port for NMEA sentences.
* Add code to handle serial exception to allow node to exit cleanly (`#52 <https://github.com/evenator/nmea_navsat_driver/issues/52>`_)
  - Catch Serial exceptions and exit cleanly, instead of printing Python stack trace.
  - Catch Serial exception when opening the serial port, log a FATAL message, and exit instead of printing Python stack trace.
* Remove MSL compensation (`#36 <https://github.com/evenator/nmea_navsat_driver/issues/36>`_)
  Fix for `#29 <https://github.com/evenator/nmea_navsat_driver/issues/29>`_ Altitude vs Elipsoid Height.
* Add GLONASS support
  GLONASS capable devices send different NMEA sentences, which are
  basically identical to the GPS sentences, but with other prefixes.
* Updated driver to accept status of 9 which some novatel receivers report for a WAAS (SBAS) fix.
  See http://www.novatel.com/support/known-solutions/which-novatel-position-types-correspond-to-the-gga-quality-indicator/

0.5.0 (2015-04-23)
------------------
* Release to Jade.

0.4.2 (2015-04-23)
------------------
* Fix remaining parse problem with NovAtel receivers (empty field specified for num_satellite).

0.4.1 (2014-08-03)
------------------
* Add debug logging output to the parser (PR #8, Mike Purvis)
* Add queue size argument to publishers to fix warning on Indigo (PR #9, Mike Purvis)
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

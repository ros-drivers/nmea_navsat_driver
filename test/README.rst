nmea_navsat_driver testing
==========================

This folder contains a tester for the nmea_navsat_driver_package as well as configurations and GNSS device logs for playback.

Configurations
--------------

* `config_launch.yaml` defines with launch files should be called, which interface they refer to and which messages are in scope. For `serial` a virtual serial port is created and for `tcp` a TCP-server is created.
* `config_logs.yaml` defines which pre-recorded logs shall be tested in which mode at which speed. And even more important which messages are to be expected from the driver in which quantity and which fields have to stay in which boundaries. These settings determine whether the test will result in pass or fail. Also defines which parameters shall be passed to the launch files.
* `config_tester.yaml` general settings of the tester.

Running the test
----------------

The tester can be automatically executed via Github Actions, running the nosetests defined in the CMakeLists.txt eventually calling the rostest. The workflow configuration is therefore located at `.github/workflows/ci.yaml`.

## Overview

This package provides a ROS interface for GPS devices that output
compatible NMEA sentences. See the
[GPSD documentation of NMEA sentences](http://www.catb.org/gpsd/NMEA.html)
for details on the raw format. Of the thousands of NMEA-compatible GPS
devices, we are compiling a list of
[devices known to be supported](#supportedhardware).

This package is compatible with the
[geographic_info](https://index.ros.org/p/geographic_info) project as
well as any other nodes that support
[sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)
and/or
[sensor_msgs/msg/TimeReference](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/TimeReference.html).

No C++ or Python API is provided, only a ROS API.

This package replaces the
[nmea_gps_driver](https://index.ros.org/p/nmea_gps_driver) package
present in Fuerte and Groovy.

## Sample Usage

To get up and running quickly, you can use the following command to
start outputting your GPS data onto ROS topics. This assumes your GPS is
outputting GGA NMEA sentences, is connected to `/dev/ttyUSB0` and is
communicating at 38400 baud.



```
$ rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=38400
```



## Roadmap

- Add support for additional NMEA sentences beyond GSA, RMC and GGA.
  Specific requests for support of individual sentences should be filed
  as enhancement tickets.
- Add sentence synchronization. Currently, there is little or no
  buffering, which prevents the driver from using multiple types of
  sentences to generate one cycle of topic outputs.

## Nodes

### nmea_topic_driver

NMEA GPS Topic driver node. Reads NMEA sentences from a ROS topic
instead of directly from e.g. a serial port.

#### Subscribed Topics

- `nmea_sentence`
  ([nmea_msgs/msg/Sentence](https://docs.ros.org/en/rolling/p/nmea_msgs/msg/Sentence.html)):
  NMEA sentences, 1 per message. The header.stamp should correspond to
  the time that the message was read from the device for accurate
  time_reference output. The header.frame_id on the messages on this
  topic will be used when setting frame_id on the output messages. These
  sentences do not have to be checksummed or otherwise validated before
  reaching the driver.

#### Published Topics

- `fix`
  ([sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)):
  GPS position fix reported by the device. This will be published with
  whatever positional and status data was available even if the device
  doesn't have a valid fix. Invalid fields may contain !NaNs.
- `vel`
  ([geometry_msgs/msg/TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)):
  Velocity output from the GPS device. Only published when the device
  outputs valid velocity information. The driver does not calculate the
  velocity based on only position fixes.
- `time_reference`
  ([sensor_msgs/msg/TimeReference](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/TimeReference.html)):
  The timestamp from the GPS device is used as the `time_ref`.

#### Parameters

- `~time_ref_source` (`string`, default: `<the value of ~frame_id>`):
  The value to use as the source in the
  [sensor_msgs/msg/TimeReference messages](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/TimeReference messages.html).
- `~useRMC` (`bool`, default: `False`): Whether to generate position
  fixes from GGA sentences or RMC sentences. If `True`, fixes will be
  generated from RMC. If `False`, fixes will be generated based on the
  GGA sentences. Using GGA sentences allows for approximated covariance
  output while RMC provides velocity information.

### nmea_serial_driver

NMEA GPS Serial driver node. Replaces nmea_gps_driver.py to reduce
naming conflicts and use new common parsing+driver backend.

#### Published Topics

- `fix`
  ([sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)):
  GPS position fix reported by the device. This will be published with
  whatever positional and status data was available even if the device
  doesn't have a valid fix. Invalid fields may contain !NaNs.
- `vel`
  ([geometry_msgs/msg/TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)):
  Velocity output from the GPS device. Only published when the device
  outputs valid velocity information. The driver does not calculate the
  velocity based on only position fixes.
- `time_reference`
  ([sensor_msgs/msg/TimeReference](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/TimeReference.html)):
  The timestamp from the GPS device is used as the `time_ref`.

#### Parameters

- `~port` (`string`, default: `/dev/ttyUSB0`): The device path
- `~baud` (`int`, default: `4800`): The baud rate to receive NMEA data.
- `~frame_id` (`string`, default: `gps`): The `frame_id` for the
  header of the
  [sensor_msgs/msg/NavSatFix](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/NavSatFix.html)
  and
  [geometry_msgs/msg/TwistStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/TwistStamped.html)
  output messages. Will be resolved with `tf_prefix` if defined.
- `~time_ref_source` (`string`, default: `<the value of ~frame_id>`):
  The value to use as the source in the
  [sensor_msgs/msg/TimeReference messages](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/TimeReference messages.html).
- `~useRMC` (`bool`, default: `False`): Whether to generate position
  fixes from GGA sentences or RMC sentences. If `True`, fixes will be
  generated from RMC. If `False`, fixes will be generated based on the
  GGA sentences. Using GGA sentences allows for approximated covariance
  output while RMC provides velocity information.

### nmea_topic_serial_reader

Reads NMEA sentences from the specified serial port and publishes them
to a ROS topic. This is a simple front-end for use with
nmea_topic_driver.py

#### Published Topics

- `nmea_sentence`
  ([nmea_msgs/msg/Sentence](https://docs.ros.org/en/rolling/p/nmea_msgs/msg/Sentence.html)):
  A single time-stamped NMEA sentence captured from the serial device

#### Parameters

- `~port` (`string`, default: `/dev/ttyUSB0`): The device path
- `~baud` (`int`, default: `4800`): The baud rate to receive NMEA data.
- `~frame_id` (`string`, default: `gps`): The `frame_id` for the
  header of the
  [nmea_msgs/msg/Sentence](https://docs.ros.org/en/rolling/p/nmea_msgs/msg/Sentence.html)
  and output message. Will be resolved with `tf_prefix` if defined.

## Troubleshooting

If you are using this package for the first time and encountering
problems you should first check that the package installed correctly and
in the right location. Then consult
[ROS/Troubleshooting](ROS/Troubleshooting), [FAQ](FAQ) and
<http://answers.ros.org/questions/> to see if a solution has already
been identified for your problem.

If you are having trouble with nmea_serial_driver and connecting to a
serial port, ensure that the user has permission to access the port by
checking to see if the user is part of the "dialout" group. See
[here](http://answers.ros.org/question/231957/connection-issue-using-nmea_serial_driver/?answer=231961#post-id-231961)
for more information.

## Report a Bug

## SupportedHardware

### Supported GPS Devices

These GPS devices have been tested with the ROS
nmea_navsat_driver.

*Please help by adding devices you have tried and the results obtained.
Only include sentences used by the driver as configured when you
reported your results. Not all devices output all supported sentences
properly.*

  -- ------------ -- ----------------------------------------------------------------------------------------------------------------- -- ------ -- -------------------------------- -- ------------------- -- ------------------------------------------------------------------------------------ --
     make            model                                                                                                                NMEA      available compatible sentences      SBAS/GBAS support      comments                                                                             
                                                                                                                                                                                                                                                                                                    
     Venus           [Venus634LPx](http://www.sparkfun.com/products/9133)                                                                 Yes       GGA, GSA, RMC                       SBAS                   works                                                                                
     Garmin          [GPS 18x](https://buy.garmin.com/en-US/US/oem/sensors-and-boards/gps-18x-oem/prod27594.html)                         Yes       GGA, GSA, RMC                       SBAS                   works                                                                                
     Septentrio      [mosaic-x5](https://www.septentrio.com/en/products/gnss-receivers/rover-base-receivers/receivers-module/mosaic)      Yes       GGA, GSA, RMC                       SBAS                   works see [this blog](https://msadowski.github.io/rtk-plug-n-play-with-septentrio)   
  -- ------------ -- ----------------------------------------------------------------------------------------------------------------- -- ------ -- -------------------------------- -- ------------------- -- ------------------------------------------------------------------------------------ --

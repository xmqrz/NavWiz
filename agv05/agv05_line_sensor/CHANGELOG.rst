^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_line_sensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------

3.1.0 (2024-10-16)
------------------

3.0.0 (2023-08-01)
------------------
* Merge branch 'nav-improve-p2-turn'

  * Change line sensor sensor_error into error code.

* Contributors: Farhan Mustar, Wong Tze Lin

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------
* Remove line sensor dyncfg and diagnostic if it is disabled by hardware plugin.
* Contributors: Patrick Chin

2.0.0 (2021-05-24)
------------------
* Reduce frequency diagnostic averaging window to 1s.
* Merge branch 'msb_embedded_process' into 'master'

  * Disable agv05_line_sensor through parameter instead of envvar 
    to prevent diagnostic stale.

* Merge branch 'obtain-heading-error-with-dual-line-sensor'

  * Compute heading error from dual line sensors.

* Contributors: Patrick Chin

1.14.2 (2020-10-19)
-------------------
* Merge branch 'node-consolidation'

  * Improve diagnostic display.
  * Merge agv05_line_sensor and agv_line_sensor into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------

1.13.16 (2020-01-12)
--------------------
* Add timeout for line sensor data.
* Contributors: Patrick Chin

1.13.15 (2019-11-21)
--------------------

1.13.14 (2019-05-21)
--------------------

1.13.13 (2019-03-22)
--------------------
* Migrate to package.xml v3 and add copyright file.
* Contributors: Patrick Chin

1.13.12 (2019-01-21)
--------------------

1.13.11 (2019-01-12)
--------------------

1.13.10 (2018-11-30)
--------------------

1.13.9 (2018-11-29)
-------------------

1.13.8 (2018-11-27)
-------------------
* Fix line sensor node still sending data via both COMs when one or both
  of the front and rear sensors are disabled.
* Contributors: nik3

1.13.7 (2018-10-19)
-------------------
* Compress the diagnostic entries for MSB raw data.
* Remove unnecessary CompositeDiagnosticTask and HeaderlessTopicDiagnostic.
  Use FrequencyStatus directly.
* Fix msb preferred side not published to diagnostic properly.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------
* Apply line calibration data directly to skip soft-reboot. Fix #27.
* Contributors: Patrick Chin

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------

1.13.2 (2018-05-18)
-------------------
* Merge line sensor topics and messages.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-06)
-------------------

1.12.4 (2018-03-27)
-------------------

1.12.3 (2018-02-27)
-------------------

1.12.2 (2018-02-23)
-------------------

1.12.1 (2018-01-03)
-------------------

1.11.6 (2017-11-06)
-------------------
* agv05_line_sensor: add iir filter for msb sensor output, value configurable in parameter
* Contributors: phtan

1.11.5 (2017-10-24)
-------------------
* agv05_line_sensor: add throttle to limit the failed message frequency
* Contributors: phtan

1.11.4 (2017-10-19)
-------------------
* agv05_line_sensor: add prefer side for line sensor
* Contributors: phtan

1.11.3 (2017-09-29)
-------------------
* Used MsbActivation.msg for activation data. Increase the data array size to 31
* use back MsbRaw.msg for activation for temporary
* pub activation
* add msb raw topic for hardware test
* Contributors: Ikhwannudin503939, nikfaisal

1.11.0 (2017-08-22)
-------------------
* agv_line_sensor: support for new core type
* agv05_line_sensor: update, add com to config,
* Update CMakeLists.txt.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.10.6 (2017-07-27)
-------------------

1.10.4 (2017-07-10)
-------------------

1.10.3 (2017-05-26)
-------------------

1.10.2 (2017-05-06)
-------------------

1.10.1 (2017-04-13)
-------------------

1.10.0 (2017-03-30)
-------------------

1.7.0 (2017-03-01)
------------------
* Fix diagnostic status text for MSB.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.6.0 (2016-11-30)
------------------

1.5.0 (2016-11-25)
------------------
* agv05_line_sensor: add diagnostic update for node, will publish all raw data and calculated output data
* Update CMakeLists.txt and package.xml files.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-10-31)
------------------

1.3.0 (2016-10-16)
------------------

1.2.0 (2016-08-25)
------------------
* agv05_line_sensor: add in additional configuration to enable/disable line sensor to support pgv
* Contributors: phtan

1.1.0 (2016-08-04)
------------------
* agv05_line_sensor: fix linear error moving average
  agv05_nav: fix forward flag slow down issue
* agv05_line_sensor: attemp to improve line sensor calculation result by adding moving average into linear error result
* Contributors: phtan

1.0.0 (2016-07-21)
------------------
* Agv05Line_sensor: Add testing filter but wont change other previous system
* agv05_line_sensor: add in moving average on the result of the line sensor raw data
* agv05_line_sensor: add in configuration activation % for front and rear MSB
* agv05_line_sensor: disable debug message
* agv05_line_sensor: add in calculation for angular error
* agv05_line_sensor: configuration file set chmod
* agv05_line_sensor: replace agv05_line_sensor using old embedded line sensor approach,
  saved original agv05_line_sensor into _bak folder
* agv05_line_sensor: change loop rate to 200Hz
* increase frequency to 50 Hz. Read outofline and junction detected status.
  Config sensor to North
* Treshold value after calibration
* Use activation to calculate the angle insteead of sensor average
* Sensor calculate add NORTH and SOUTH
* Remove sensor value inside jucntion detection and out of line
* Change address of saving data
* agv05_line sensor:publish output sensor value in radian
* agv05_line_sensor: add new msg MsbData that can publish front and rear sensor value at once
  rename node handler to scope_nh and scope_nh_peripheral
* agv_line_sensor: add dependency to get msgs
* Agv05_line_sensor data txt file
* Agv05_line_sensor - Agv05_line_sensor.cpp (need to adjust the publisher and subscriber)
* Agv05_line_sensor:
  -Processing the data and get the radian value
* Agv05_line_sensor:
  -Initialize the variables
* Agv05_line_sensor:
  -Read status and sensor value
* Agv05_line_sensor:
  -Calibration will add init_calibration to reset the data max and data min
  -Type to choose which file to save or load
* Agv05_line_sensor:
  - Load treshold value will load data treshold, data max, and data min
* Agv05_line_sensor:
  -Sensor init o reset the variable
  -Sensor config to write the height and widht of the agv
  - WriteSensorRaw to write the adc data
* Agv05_line_sensor:
  -Create new variables which use to get radian values
  -Create new functions which use to process to get radian values
* Agv05_line_sensor: include math.h to use atan2
* Agv05_line_sensor:
  - Recorrect the number of sensors to 31
  - Rename sensor_data\_ to sensor_data_average\_
* Agv05_line_sensor:
  -Add init variable to reset the data_max and data_min to default value
  -Recorrect the number of sensors to 31
* Agv05_line_sensor:
  Save and Load process will save and load data_treshold, data_max and data_min
* remove realtime tools
* agv05_line_sensor: agv05 Cmake file and backup Cmake file(Cmakeori)
* agv05_line_sensor: agv05_line_sensor.cpp
  Main function
* agv05_line_sensor: agv05_sensor_calculate
* agv05_line_sensor: agv05_calibrate
* agv05_line_sensor: agv05_calibrate
* agv05_line_sensor: agv05_save_load_data
* AGV05 : Create two package name power management and linesensor
* Contributors: Ikhwan, Ikhwannudin503939, Ikhwannudin503939 <dramzpj@gmail.com>, Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

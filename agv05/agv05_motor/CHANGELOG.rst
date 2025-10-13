^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_motor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'nav-omni' into 'master'

  * Separate swerve front and rear steering angle limit.

* Merge branch 'safety-renesas' into 'master'

  * Add heartbeat monitoring on IMU.
  * Add heartbeat monitoring on safety trigger.
  * Add safety heartbeat publishing.

* Merge branch 'nav-motor' into 'master'

  * Update swerve steering alignment angle.
  * Set steering target angle nearer to middle position during aligning steering.
  * Keep last command steering angle before stopping.
  * Loosen swerve steering angle tolerance during moving.
  * Flip steering angle base on previous target instead of feedback.
  * Add support to Omni without speed feedback.
  * Non-zero steering angle to signal hardware node to move steering.
  * Add configurable pre-path-following steering alignment angle.
  * Tighten steering angle checking before start moving.

* Contributors: Tan Kui An Andrew, Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Merge branch 'nav-improve-p3' into 'master'

  * Fix occasional motor fault speed error false triggering.
  * Fix steer angle is not following the given parameter to turn.

* Merge branch 'hal' into 'master'

  * Add back unbrake button to free motor.
  * Add custom drive type.

* Merge branch 'nav-omni' into 'master'

  * Allow empty motor feedback time stamp.
  * Add motor fault hint.
  * Add swerve drive.
  * Add mecanum drive.

* Merge branch 'nav-improve-p3' into 'master'

  * Add time stamp to MotorFeedback and SteeringFeedback message types.

* Merge branch 'nav-improve-p3-motor' into 'master'

  * Add FIR filter on speed error in detecting motor fault.
  * Extend the get twist service to have option to return maximum twist.
  * Update forward flag speed in the beginning of action according to latest motor command velocity.

* Add angular control speed compensation on asymmetrical drive.
* Limit drive speed instead of stop totally according to steering angle error.
* Fix tricycle drive speed in opposite direction of command velocity.
* Refactor code.
* Revert previous changeset as wheel acceleration command speed base on feedback speed is sluggish in line/path following.
* Calculate wheel acceleration command speed to hardware node base on feedback speed instead of open loop.
* Add free motor triggering from manual control.
* Navigation enable depends on feedback enable from hardware node.
* Remove motor off (idle) timeout, remove brake duration, remove button_unbrake\_, remove navigation_enable_delay, remove motor_relay\_ topic pub, remove output\_.enable topic pub.
* Add generic 2D-kinematic model.

* Contributors: Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'time-jump-safety-trigger'

  * Move time jump detection to safety node.
  * Trigger safety when time jump happens.

* Merge branch 'nav-improve-p2'

  * Add motor fault trigger min speed.

* Fix wheel calibration data not used in wheel speed control.
* Merge branch 'time-jump-fix'

  * Cap acceleration/deceleration period at twice of expected process period to avoid sudden change in motor speed.
  * Limit motor acceleration period in the event of time jump.

* Merge branch 'update-minimum-motor_off_timeout'

  * Reduced minimum motor_off_timeout from 60s to 1s.

* Contributors: Andrew Tan, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Merge branch 'track-odom'

  * Fix odometry calculation.

* Merge branch 'jerk-limit' into 'master'

  * Add moving average filter for unsynchronized wheels feedback from differential hardware node.

* Prioritize unbrake button over safety trigger brake duration.
* Add steering command angle entry in agv05_motor diagnostic.
* Merge branch 'odom-imu' into 'master'

  * Original odom TF topic is renamed into odom/wheel TF topic while new odom TF topic will be product of IMU data fusion.
  * Subscribe to topic /imu/data with any frame_id.
  * Fuse IMU data into odometry calculation.

* Merge branch 'decelerate-on-safety-trigger'

  * Decelerate motor on safety trigger.

* Contributors: Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------
* Publish motor error code.
* Contributors: Patrick Chin

2.0.0 (2021-05-24)
------------------
* Reduce frequency diagnostic averaging window to 1s.
* Add motor enabled feedback.
* Merge branch 'steering-drive' into 'master'

  * Improve inverse kinematics of steer control given the drive center offset.
  * Allow configuration of steer angle for turning left and right.
  * Add debug log for motor control output.
  * Switch rotate direction instantly by inversing steering drive speed.
  * Continue to publish last odom when motor feedback is unavailable.
  * Apply hysteresis for steering align state.
  * Publish odom zero when agv05_motor starts.
  * Flip steering angle for reverse motion.
  * Add tricycle steering drive controller.
  * Duplicate Agv05Motor to Controller class.

* Fix motor controller not capping negative max speed.
* Fix negative odom jump not detected.
* Fix bug last motor feedback not reset after odom jump.
* Fix parameter range for max acceleration and deceleration.
* Contributors: Patrick Chin, HazimGharib

1.14.2 (2020-10-19)
-------------------
* Ignore huge odom jump.
* Merge branch 'node-consolidation'

  * Improve diagnostic display.
  * Merge agv05_motor and agv_motor into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Add mileage publisher for Zalpha.
* Remove unused parameters.
* Contributors: Patrick Chin

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Add dynamic reconfigure for motor fault trigger.
* Contributors: Nik

1.13.14 (2019-05-21)
--------------------
* Enable motor on controller start.
* Fix motor fault single trigger.
* Add diagnostic for speed in m/s.
* Fix wrong speed reading for index.
* Change motor overload to motor fault.
* Add motor overload safety trigger.
* Add idle disable motor timeout.
* Add dynamic reconfigure ESCON type for current reading range handler.
* Add missing dependencies.
* Contributors: Nik, Patrick Chin

1.13.13 (2019-03-22)
--------------------
* agv05_motor: Add dynamic reconfigure to configure the calibration for
  linear distance for left and right, and angular distance for agv.
* agv05_motor: Fix inverted safety state for motor current reading.
* agv05_motor: Fix current reading for motor. ESCON must be configure to correct current output reading.
* Migrate to package.xml v3 and add copyright file.
* Contributors: Nik, Patrick Chin, df, phtan

1.13.12 (2019-01-21)
--------------------

1.13.11 (2019-01-12)
--------------------
* Add comment.
* Contributors: Patrick Chin

1.13.10 (2018-11-30)
--------------------
* Fix publish navigation enable on safety brake.
* Motor braking on safety trigger, enable signal always on parameter added.
* Add configurable motor off delay and braking options for new motor wiring.
* Contributors: nik3

1.13.9 (2018-11-29)
-------------------

1.13.8 (2018-11-27)
-------------------
* Increase precision of saved mileage.
* Move motor mileage to VariableStorage.
* Contributors: Patrick Chin

1.13.7 (2018-10-19)
-------------------
* Remove unnecessary CompositeDiagnosticTask and HeaderlessTopicDiagnostic.
  Use FrequencyStatus directly.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------
* agv05_motor: add in safety_brake trigger for agv05_motor, this trigger will only brake the motor but will not power off the driver
* Contributors: phtan

1.13.2 (2018-05-18)
-------------------
* Update motor topics.
* Merge power topics and messages. Remove the now-obsolete agv_translator.
* Merge safety topics and messages. Flip definition of safety flags.
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

1.11.5 (2017-10-24)
-------------------

1.11.4 (2017-10-19)
-------------------
* agv05_motor: publish motor current
* Contributors: Ikhwannudin503939

1.11.3 (2017-09-29)
-------------------

1.11.0 (2017-08-22)
-------------------

1.10.6 (2017-07-27)
-------------------

1.10.4 (2017-07-10)
-------------------

1.10.3 (2017-05-26)
-------------------
* fixes #12, agv05_msgs: change encoder message to 16 bit, agv05_core: publish only 16 bit data from the encoder, agv05_motor: process distance and odom only using 16 bit data
* Contributors: nikfaisal05 <nik@dfautomation.com>, phtan

1.10.2 (2017-05-06)
-------------------
* fix invert current to use config
* Contributors: nikfaisal05 <nik@dfautomation.com>

1.10.1 (2017-04-13)
-------------------
* Fix precision loss in distance value when using Float32.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.0 (2017-03-30)
-------------------

1.7.0 (2017-03-01)
------------------
* Fix lock mutex affecting straight distance publish rate.
* Increase delay current and invert motor current.
* Add delay current reading to wait for motor driver starting up.
* Update motor diagnostics listing.
* Publish motor current and mileage in diagnostics.
* Publish odom data in diagnostic.
* Update package metadata.
* agv05_motor: add in reset odom service
* Contributors: NikFaisal nikfaisal05@gmail.com, Patrick Chin <patrickcjh@gmail.com>, nikfaisal, nikfaisal <nikfaisal05@gmail.com>, phtan

1.6.0 (2016-11-30)
------------------

1.5.0 (2016-11-25)
------------------
* agv05_motor: add diagnostic updater
* agv05_motor: enable testing on CMake file
* Update CMakeLists.txt and package.xml files.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-10-31)
------------------
* agv05_motor: change navigation enable time to 1s, Navigation of the motor will be
  enabled after 1s
* Contributors: phtan

1.3.0 (2016-10-16)
------------------
* agv05_core: change frequency to 200Hz
  agv05_nav: change frequency to 200Hz
  agv05_motor: change frequency to 100Hz and remove debug message
* agv05_motor: add in publisher to publish the navigation_enable state of the motor
* agv05_motor: calculate maximum speed for each motor according to the motor type, for those speed that is exceeding 100% of the
  motor speed, the linear x velocity will be limited.
* Contributors: phtan

1.2.0 (2016-08-25)
------------------

1.1.0 (2016-08-04)
------------------

1.0.0 (2016-07-21)
------------------
* agv05_motor: add in safety_flag_trigger\_ will be activated upon subscribe of the data to ensure
  that the process wont miss out the subscribe.
* agv05_motor: add in function to publish motor control, will only publish if differences is found,
  add in control for motor power, will turn off motor after 0.5s safety activated
* agv05_motor: fix mutex variable name error
* agv05_motor: fix ineffective mutex lock,
  add in straght_distance and rotational_distance topic publish for nav use,
  add in check at the mileage to only update config if the mileage value changed,
* agv05_motor: remove config variable initialization (avoid overwrite to config callback),
  add in Lock(_mutek) for update mileage and config callback
* agv05_motor: disable mileage report on the output
* record travelled mileage and store in dynamic_reconfigure
* agv05_motor: add in dynamic reconfigure for motor configuration,
  change loop rate to 50Hz,
  add in distance topic publish by motor for nav purpose
* Fix catkin_lint issues.
* change left right wheel distance for TAG05
* agv05_motor: enable motor enable only on the safety portion, navigation portion only controls speed
* agv05_motor: always enable motor (brake condition on normal run), and add 0.5s timer
  after safet release before enable and run motor
* agv05_motor: add in function to publish odom, previously not included
* agv05_motor: fix left and right speed invert to match the changs of CW and CCW,
  clear left and right speed mem in process during safety trigger to avoid non zero
  speed value after safety trigger release
* agv05_motor: fix velocity timeout error, change from uint32_t to float
* agv05_motor: commit for completed agv05_node,
* agv05_motor empty package file commit
* Contributors: Ikhwan, Iwan, Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

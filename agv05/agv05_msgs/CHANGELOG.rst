^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Merge branch 'hal' into 'master'

  * HAL battery state diagram.
  * Add message type for custom marker dock/undock.

* Merge branch 'nav-omni' into 'master'

  * Add omni action types.
  * Add ROS message type of omni-drives.

* Merge branch 'nav-improve-p3-safety-area' into 'master'

  * Different safety area for different safety block speed range.

* Merge branch 'nav-improve-p3' into 'master'

  * Add time stamp to MotorFeedback and SteeringFeedback message types.

* Merge branch 'nav-improve-p3-omni-dock' into 'master'

  * Add omni docking action.
  * Add safety area for omni motion.

* Merge branch 'nav-improve-p3-motor' into 'master'

  * Extend the get twist service to have option to return maximum twist.
  * Update forward flag speed in the beginning of action according to latest motor command velocity.

* Merge branch 'nav-improve-p3-io' into 'master'

  * Add IO trigger distance in wait IO actions.

* Refactor code.
* Add agv05_modbus_client package.
* Contributors: Patrick Chin, Quan Bong, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'rotate3q'

  * Add three-quarter rotate action.

* Merge branch 'time-jump-safety-trigger'

  * Trigger safety when time jump happens.

* Merge branch 'nav-improve-p2'

  * Remove next motion of non-stop for dynamic path.
  * Add track curve line action navigation goals.

* Merge branch 'nav-improve-p2-costmap'

  * Update for motion paths and area.

* Merge branch 'nav-improve-p2-turn'

  * Trigger line sensor error only if in used.
  * Rename line sensor com error to line sensor error.
  * Change line sensor sensor_error into error code.
  * Check for msb calibration validation.

* Merge branch 'tune-pid'

  * Add PID tuning action.

* Contributors: Andrew Tan, Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Merge branch 'track-odom'

  * Add handling to Forward/Reverse (by distance) skills with odometry feedback.

* Merge branch 'jerk-limit' into 'master'

  * Add consolidation of track line speed limitations.
  * Add trackless path speed limitations.

* Contributors: Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------
* Refactor code.
* Merge branch 'bezier-deceleration' into 'master'

  * Handle deceleration when next motion is bezier type.

* Merge branch 'path-planner' into 'master'

  * Add next motion of non-stop dynamic path.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------
* Merge branch 'obstacle-hint'

  * Add hint parameter for obstacle sensor msg and update agv05_lidar to publish topic hint if lidar near block of mulfunction.

* Merge branch 'msb_embedded_process' into 'master'

  * add junction_count to LineSensor message type

* Use 16-bit representation for motor error code.
* Merge branch 'path-planner'

  * Add definition of default mid and goal junction tolerance.
  * Add handling to new dynamic NavxAction nav goal.
  * Update message and config for dynamic path planning.
  * Add new PolygonArray ROS message type.
  * Add `plan empty` nav action feedback.
  * Add dynamic forward action.

* Merge branch 'stacker-docking'

  * Extend forward wait IO action with error IO trigger.
  * Extend docking action with error IO trigger.

* Merge branch 'next-motion-hint'

  * Replace forward_flag with next_motion parameter.

* Add motor enabled feedback.
* Merge branch 'steering-drive' into 'master'

  * Add tricycle steering drive controller.

* Merge branch 'next-navigation-action-hint'

  * Use next nav action to set turn signal led and stopping offset.

* Merge branch 'merge-agv05x-repo' into 'master'

  * Import agv05_navx package from agv05x repo.

* Merge branch 'obtain-heading-error-with-dual-line-sensor'

  * Compute heading error from dual line sensors.

* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin, phtan

1.14.2 (2020-10-19)
-------------------
* Remove brake flag in PowerRelayControl msg.
* Add battery_percentage to PowerSource msg.
* Merge branch 'node-consolidation'

  * Merge agv05_msgs and agv_msgs into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------

1.13.14 (2019-05-21)
--------------------
* Update IO to use one topic per port.
* Contributors: Patrick Chin

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

1.13.7 (2018-10-19)
-------------------

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------

1.13.2 (2018-05-18)
-------------------
* Merge with Node Consolidation branch.
* Merge power topics and messages. Remove the now-obsolete agv_translator.
* Merge safety topics and messages. Flip definition of safety flags.
* Merge led topics and messages.
* Merge obstacle sensor topics and messages.
* Merge line sensor topics and messages.
* Move NavAction and NavControl definition into agv_msgs.
* Add LASER_MALFUNCTION nav feedback.
* Merge audio node and audio messages.
* Sort message list.
* agv_msgs and agv05_msgs: move LineSensor, LineSensorActivation and MsbRaw to agv_msgs
* Add dummy external safety 2.
* Upgrade laser and nav nodes to allow laser profile and area selection.
* Temporarily remove changes to default agv05.
* agv_line_sensor: completed first draft agv_line_sensor with line error and angular_error result
* agv_line_sensor: node publish, completed pub_sub part, pending processes, added new message for LineSensor
* agv_msb: remove sensor count inside Msb Raw variable
* agv05_msgs: add new message BatteryState
* agv_msb: fix segmentation fault, communication with MSB success
* agv_msb: add new node to process raw data for AGV MSB
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-06)
-------------------
* Add sensor_com_error feedback type in NavAction.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.4 (2018-03-27)
-------------------
* Allow forward_flag to be cancelled through NavControl. Fix #19.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.3 (2018-02-27)
-------------------
* Make wait_traffic one of the feedback type.
* Make led blink error on navigation failure.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.2 (2018-02-23)
-------------------

1.12.1 (2018-01-03)
-------------------
* Change disable obstacle pass junction to last junction laser area
* Add "Select Front Laser Profile" skills.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>

1.11.6 (2017-11-06)
-------------------

1.11.5 (2017-10-24)
-------------------

1.11.4 (2017-10-19)
-------------------
* agv05_motor: publish motor current
* agv05_nav: add new action for Forward Left, Forward Right, Reverse Left and Reverse Right
* Contributors: Ikhwannudin503939, phtan

1.11.3 (2017-09-29)
-------------------
* Used MsbActivation.msg for activation data. Increase the data array size to 31
* Contributors: Ikhwannudin503939

1.11.0 (2017-08-22)
-------------------
* add ComPacket for new core type
* Contributors: phtan

1.10.6 (2017-07-27)
-------------------
* agv05_msgs: file commit for Remote Io messages
* agv05_msgs: remove old remote io message
* agv05_msgs: update messages for remote io, agv05_remote_io: update new type of publish subscribe to have request and reply
* Contributors: phtan

1.10.4 (2017-07-10)
-------------------

1.10.3 (2017-05-26)
-------------------
* fixes #12, agv05_msgs: change encoder message to 16 bit, agv05_core: publish only 16 bit data from the encoder, agv05_motor: process distance and odom only using 16 bit data
* Contributors: phtan

1.10.2 (2017-05-06)
-------------------
* agv05_msgs: add new message for remote io input and output
* Contributors: phtan

1.10.1 (2017-04-13)
-------------------

1.10.0 (2017-03-30)
-------------------

1.7.0 (2017-03-01)
------------------
* Update nav action status number to be in sync with trackless'.
* agv05_nav: add in new action for search line
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.6.0 (2016-11-30)
------------------
* Improve audio player to play from different playlist.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* agv05_laser_sensor: add diagnostic publisher to laser sensor node
* agv05_msgs NavAction.action: rearrange status for correct status type
* agv05_msgs: remove unused laser sensor topics
* agv05_core & agv05_msgs: update output port topic type, addedd output_mask to control the output more flexibily
* agv05_msgs: rename voltage raw
* Update package meta files.
* agv05_core & agv05_driver: update for new AGV MCTRL V1.2 board.
  IO port will pub and sub only the raw data, processing & port selection will be on the peripheral node.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-10-31)
------------------

1.3.0 (2016-10-16)
------------------
* agv05_msgs NavAction: add new parameter for action called 'rotate_align_sensor', to select
  the sensor use for rotation alignment
* agv05_nav, agv05_msgs: add in new parameter to NavAction, io_trigger and io_trigger_type
  remove REVERSE_WAIT_IO action, now forward/reverse with wait IO will use standard forward/reverse
* Contributors: phtan

1.2.0 (2016-08-25)
------------------
* agv05_nav & agv05_msgs: create a new nav action called wait traffic
* Contributors: phtan

1.1.0 (2016-08-04)
------------------
* agv05_nav & agv05_msgs: add in new action, reverse with wait IO, will be use for
  hooking action
* Contributors: phtan

1.0.0 (2016-07-21)
------------------
* Power management diagnostic
* agv05_msgs NavAction: add in new nav status, STATUS_CHARGER_CONNECTED
* agv05_msgs SafetyTrigger & agv05_safety: change safety trigger from charging_state_trigger to charger_connected_trigger,
  rename topic of subscribe from charging_status to charger_connected
* agv05_msgs, SafetyTrigger: add in new flag for safety trigger, charging_state_trigger
* agv05_nav & NavAction: add in Free Motor navigation mode for free motor operation
* agv05_led, agv05_msgs & agv05_nav: change blink error to constant Red, and change safety trigger to blink red
* agv05_led: add in BLINK_PAUSE type, rename BLINK_TURNING to BLINK_NORMAL_WARNING
* agv05_laser_sensor: enable laser sensor node to have custom laser senosr config
* agv05_nav: add in nav_scan_laser_profile 1-5,
* agv05_nav: add in result for action, add in out_of_line_distance,
* agv05_nav: add in navigation profile 5 set of different profile
* agv05_nav: rename line calibration to line calibrate, add in calibrate action inside process
* NavAction: add in new action, line calibration
* Add UIPopup msg definition.
* Sort and remove duplicate msg.
* dynamic reconfigure and set volume and file by topic in audio player
* agv05_msgs/NavAction: add in new action NAV_MANUAL_CONTROL
* added battery low flag and rename battery_switch_percentage to battery_percentage
* NavAction: add in 2 new action for uturn
* agv05_msgs NavAction: minor edit on the number and update comment for status description
* NavAction: rename exe to nav
* agv05_msgs/MsbData: update message type to have linear_error and angular_error
* rewrite audio player (play all mp3 files in music playlist; set play, pause, stop, volume, filename)
* agv05_nav: file commit for nav initial framework
* agv05_safety:
  safety will not receive trigger from laser sensor,
  nav will instead trigger the "nav_trigger" to safety for the "software" safety control
* agv05_msgs: commit NavAction action
* agv05_msgs: add in new messages for laser sensor
* Clean up package metadata and CMakelists.txt.
* subscribe and display imu data from embedded
* add agvError
* add outofline flag and junction detected flag. data value no more front or rear
* New MsbData.msg publishe front and rear radian sensor value
* Addd MsbData.msg
* agv05_safety: file commit for agv05_safety node, with new SafetyTrigger new message.
* agv05_msgs: add in LedBlinkControl message into CMakeList
* agv05_led: file commit for tested agv05_led node (to control the blinking of led)
  agv05_msgs: add new message LedBlinkControl to control the led blink according to the predefined const
* agv05_core update with custom error message
* agv05_msgs: add in new AgvError message for all error
* agv05_core and MotorControl.msg: change motor control value to float32, motor speed control
  will be in %
* agv05_core: change motor control speed type to float
* change VoltageCurrentRaw message typt to have independent current value for both battery
* agv05_core: addin shutdowncontrol, and malfunction output for the laser.
  initialize the outputs\_ class variable.
* agv05_msgs: add two msg BatteryPercentage and VoltageCurrentData
* agv05_msgs: add in MsbRaw message type
* LaserAreaControl: rename the variable to area
* agv05_msgs: remove unused msg and update it with agv05_core topics variable type
* Update message definitions.
* Add agv05_msgs package.
* Contributors: Ikhwan, Ikhwannudin503939 <dramzpj@gmail.com>, Patrick Chin <patrickcjh@gmail.com>, nikfaisal, nuxail, phtan

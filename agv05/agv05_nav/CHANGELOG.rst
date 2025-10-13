^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'nav-omni' into 'master'

  * Limit manual control output angular-linear speed ratio to input angular-linear speed ratio.

* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.
  * Add heartbeat monitoring on safety trigger.
  * Add heartbeat monitoring on line sensors.
  * Add safety heartbeat publishing.
  * Add safety heartbeat monitoring.
  * Add heartbeat monitoring on obstacle sensors.

* Merge branch 'nav-improve' into 'master'

  * Fix safety area disabled during non-stop transition.
  * Keep manual control activate until task runner is resumed from paused.
  * Mute obstacle sensing including bumper during manual control.

* Merge branch 'manual-control-when-paused' into 'master'

  * Refactor code.
  * Fix safety area of manual control during paused.

* Merge branch 'manual-control-when-paused'

  * Handle manual control during paused state.

* Contributors: Patrick Chin, Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Remove header files from debian package.
* Merge branch 'nav-doa' into 'master'

  * Fix missing LED display and alarm during path blocked safety triggered.

* Merge branch 'nav-omni' into 'master'

  * Add motor fault hint.
  * Add handling to omni actions.
  * Add manual control in lateral direction.

* Merge branch 'nav-improve-p3-dynplan' into 'master'

  * Refactor code.

* Merge branch 'nav-improve-p3-safety-area' into 'master'

  * Add option to limit maximum speed if personnel detection means in travel direction is permanently muted according to ISO 3691-4.
  * Different safety area for different safety block speed range.

* Merge branch 'nav-improve-p3' into 'master'

  * Refactor code.

* Merge branch 'nav-improve-p3-omni-dock' into 'master'

  * Add omni docking action.

* Merge branch 'nav-improve-p3-motor' into 'master'

  * Extend the get twist service to have option to return maximum twist.
  * Update forward flag speed in the beginning of action according to latest motor command velocity.
  * Add junction miss checking with long forward/reverse junction distance.

* Merge branch 'nav-improve-p3-io' into 'master'

  * Add final undershoot timeout.
  * Add option to change IO trigger debouncing time.
  * Add IO trigger distance in wait IO actions.

* Fix noetic-roslint warning.
* Refactor code.
* Add free motor triggering from manual control.
* No safety triggered but hardware is not ready. Use small speed to wake up hardware.
* Fix search line success with out of line.
* Fix return-type warnings.
* Merge branch 'nav-improve-p3-nonstop-transition' into 'master'

  * Add obstacle sensor deactivation delay after non-stopping actions done.

* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'add-navigation-profile-into-diagnostic'

  * Add navigation profile into diagnostics.

* Merge branch 'fix-parameter-update'

  * Fix parameter reset failed with agv05_navx and agv05_obstacle_sensor.

* Merge branch 'protected-parameter'

  * Suffix reset profile config with underscore. Update default nav config values.

* Merge branch 'rotate3q'

  * Add three-quarter rotate action.

* Merge branch 'nav-improve-p2'

  * Align rosparam name in yaml file to be same as name in config.
  * Change turn_search_distance to be calculated from target line angle.
  * Adapt to agv_mctrl new standard (zalpha-3.3.0 & above hardware plugin).
  * Extend range of io trigger stop distance to be same as range of junction trigger stop distance.
  * Add handling of goal speed and enable sensor in manual control.
  * Add bumper blocked hints.
  * Fix speed become negative if deceleration and jerk limit are much larger than acceleration and jerk limit.
  * Add overshoot into nav diagnostic.
  * Add handling to track curve line action.

* Merge branch 'time-jump-safety-trigger'

  * Trigger safety when time jump happens.

* Merge branch 'sense-line-options'

  * Rename variable 'sense_line' to 'use_line_sensor' for clarity.

* Merge branch 'nav-improve-p2-turn'

  * Publish laser activation hint as safety internal message.
  * Add safety_internal_message for line sensor error message.
  * Trigger line sensor error only if in used.
  * Rename line sensor com error to line sensor error.
  * Change line sensor sensor_error into error code.
  * Fix final stopping position of MSB calibration.
  * Check for msb calibration validation.
  * Refactor code for turn actions.

* Merge branch 'nav-improve-p2-pid'

  * Update tuning value base on test result with R&D Zalpha 4.
  * Reduce back the heading gain during PID tuning to produce larger error to withstand localization noise.
  * PID tuning uses wheel encoder odometry only to avoid sensor noise.
  * Allow user to manually set ahead distance for PID tuning.
  * Change PID control line following heading gain to look ahead distance.

* Merge branch 'nav-improve'

  * During stopping at last junction, follow the line of last seen track from seen junction instead of last seen MSB error.
  * Add option to check out_of_line before start moving.
  * Add option to have traditional PID controller for path following. Update default value of navigation parameters.
  * Revert back to traditional PID controller for track line following. Split accumulation of angular out of line from linear distance.

* Fix build in ROS Indigo.
* Merge branch 'tune-pid'

  * Revert removal of g\_ as caller of goal_init() may need return of lvalue instead of rvalue.
  * Remove unused goal variable. Allow default heading gain greater than 0.5.
  * Add PID tuning action.

* Fix linter error.
* Contributors: Andrew Tan, Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Merge branch 'nav-profile' into 'master'

  * Run autoformat.
  * Add navigation profiles enumeration service for drop down list selection.
  * Round value during float to double conversion.
  * Add support to 3 preset profiles for Nav and NavX nodes.

* Merge branch 'track-odom'

  * Add handling to Rotate (by angle) skill with odometry feedback.
  * Add handling to Forward/Reverse (by distance) skills with odometry feedback.
  * Add subscription to 2D Odometry feedback for track base.

* Merge branch 'jerk-limit' into 'master'

  * Add turn_alignment_center_error into reaching speed calculation.
  * Add parameter of stopping deceleration which to be used in reaching kinematic calculation.
  * Nested tab display for navigation action and safety parameters.
  * Remove instant stop of command speed due to AMR status. Stop immediately if previous command speed is less than minimum speed and overshoot above threshold distance.
  * Reduce line following speed if linear error is larger than threshold.
  * Limit aligning speed to user-given value in case of jerk limiting.
  * Add consolidation of track line speed limitations.
  * Add parameters for customizing Bezier look ahead distance as in pure pursuit algorithm. Update default value of navigation parameters.
  * Implement Anti-Reset Windup and Derivative Kick according to https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralDerivative.
  * Add stopping offset to compensate overshoot due to minimum output speed deadband of Mabuchi motor.
  * Merge PID controller for linear and heading error.
  * Stop immediately if previous command speed is less than minimum motor speed when it is time to stop.
  * Add turn action overshot handling for inverse kinematic.
  * Change close-loop P control to inverse kinematic for turning actions in order to limit jerk.
  * Add utility class ReachingKinematic for converting target distance to-from target speed.
  * Add jerk limitation parameters and handling for tracked action.
  * Add jerk limitation to VelocitySmoother.

* Refactor code.
* Contributors: Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------

2.0.0 (2021-05-24)
------------------
* Refactor code.
* Reduce frequency diagnostic averaging window to 1s.
* Fix I coefficent scale in line and path follow PIDs.
* Merge branch 'msb_embedded_process' into 'master'

  * Change process junction count
  * Set on_junction flag to TRUE if junction detected
  * Refector init junction count
  * Use junction_count for junction detection

* Merge branch 'steering-drive'

  * Allow configuration for path follow angular output saturation.

* Add debug log for nav.
* Trigger line sensor error if it is disabled during navigation.
* Merge branch 'stacker-docking'

  * Adjust IO trigger junction stopping distance based on next motion.
  * Extend forward wait IO action with error IO trigger.

* Fix bug for io trigger update duration.
* Use last good line sensor data when out-of-line.
* Merge branch 'next-motion-hint'

  * Assert velocity smoother work with positive magnitudes only.
  * Remove ineffective asserts due to integer promotion.

* Replace forward_flag with next_motion parameter.
* Disable obstacle after safety triggered and add config to disable it.
* Merge branch 'steering-drive' into 'master'

  * Make nav publish a small value to motor as hint to align steering.

* Merge branch 'next-navigation-action-hint'

  * Use next nav action to set turn signal led and stopping offset.

* Merge branch 'manual_control_app_move_on_start_up' into 'master'

  * Fix manual control apps move on start up

* Merge branch 'obtain-heading-error-with-dual-line-sensor'

  * Split PID controller for linear and heading error.

* Contributors: Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin

1.14.2 (2020-10-19)
-------------------
* Fix typo of `deceleration`.
* Merge branch 'node-consolidation'

  * Move nav topics and msgs into agv05 namespace.

* Increase straight line speed limit for nav.
* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Add wheel-slippage related safety flag and nav action feedback.
* Use negative distance in forward-by-distance action goal to signal that it is measured after junction.
* Add obstacle sensing for manual control.
* Publish BLINK_NORMAL_LEFT or BLINK_NORMAL_RIGHT led status for rotate and branching actions.
* Contributors: Farhan Mustar, Patrick Chin, phtan

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Merge branch 'forward-flag-fix'.

  * Stop motor on wait traffic action.
  * Use normal obstacle area when ending with forward flag.

* Merge branch 'missed-safety-feedback'.

  * Ensure the safety error is published at least once by nav action server.

* Saturate line-follow angular speed based on linear speed.
* Merge branch 'turn-alignment-fix'

  * Prevent turn alignment timeout when safety triggerred.

* Fix shifted topic subscriptions.
* Add option for auto-resume after external safety trigger.
* Increase number of laser profiles to 10.
* Contributors: Patrick Chin

1.13.14 (2019-05-21)
--------------------
* Remove redundant safety braking feature.
* Change motor overload to motor fault.
* Handle motor overload safety trigger.
* Modify nav to handle multi-port IO.
* Add missing dependencies.
* Contributors: Nik, Patrick Chin

1.13.13 (2019-03-22)
--------------------
* Apply junction deceleration value after moving pass junction.
  Instead of the straight profile deceleration value.
* Migrate to package.xml v3 and add copyright file.
* Contributors: Patrick Chin

1.13.12 (2019-01-21)
--------------------

1.13.11 (2019-01-12)
--------------------
* Fix bug out-of-line trigger ignored by forward- and reverse-by-distance actions.
* Contributors: Patrick Chin

1.13.10 (2018-11-30)
--------------------

1.13.9 (2018-11-29)
-------------------
* Fix accel and decel values swapped in manual control clockwise direction.
* Contributors: Patrick Chin

1.13.8 (2018-11-27)
-------------------
* Implement forward-by-distance action.
* Contributors: Patrick Chin

1.13.7 (2018-10-19)
-------------------
* Remove the ability to pause and resume nav through physical buttons. Fix #6.
* Separate safety resume from action resume.
* Remove unnecessary CompositeDiagnosticTask and HeaderlessTopicDiagnostic.
  Use FrequencyStatus directly.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------
* Add option to follow through line sensor error when moving over junction.
* Reduce forward distance after passing junction when forward flag is set.
* Fix reverse-wait-io not using io-trigger-stop-distance.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------
* agv_msgs & agv_nav_track: add additional safety flag safety_io_1 and safety_io_2
* Fix bug in printing diagnostic message.
* Update nav topics.
* Allow button actions to be configurable in nav. Fix #20.
* Stop button should mute the nav alarm.Fix #21
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.13.2 (2018-05-18)
-------------------
* Merge with Node Consolidation branch.
* Merge safety topics and messages. Flip definition of safety flags.
* Modify definition values for io_trigger and io_trigger_type in NavAction.
  - io_trigger will use zero-based indexing.
  - io_trigger_types will start from 1 and some enum values are swapped.
* Merge io topics.
* Merge led topics and messages.
* Merge obstacle sensor topics and messages.
* Move NavAction and NavControl definition into agv_msgs.
* Add LASER_MALFUNCTION nav feedback.
* Merge audio node and audio messages.
* Add dummy external safety 2.
* Upgrade laser and nav nodes to allow laser profile and area selection.
* agv_nav_track: Fix wrong topic name for laser profile.
* agv_nav_track: Rearrange feedback safety sequence.
* agv_nav_track: Fix build fail.
* agv_nav_track : Change inverted safety state.
* agv_nav_track: add safety sensor lost communication handler
* agv_nav_track: Message merge only.
* Rename agv05_nav into agv_nav_track.
* Remove agv_nav_track package.
* Remove unused file.
* agv_nav_track:Improve safety and led function.
* Fix compile error.
* agv_nav_track : Merge safety, lcd, remap topic and message type.
* agv_nav_track : Initial merge with agv05_nav.
* agv_nav_track: last commit before merge.
* agv_nav_track : Use agv_pub_sub
* agv_safety: Remove sensor area control.
* agv_nav_track: Commit after forward_flag testing.
* agv_nav_track: Add safety message popup.
* agv_nav_track: Refactor code.
* agv_nav_track : Add sensor profile dynamic reconfigure.
* agv_nav_track : Handle straight speed limit.
* agv_nav_track: Fix node crashing.
* agv_nav_track: Working forward, reverse and turn action(without safety).
* agv_nav_track: Change Utils under Action Class.
* agv_nav_track: Add all action class.
* File commit for agv_ll300. Node is not completed but added CATKIN_IGNORE to avoid build fail.
* agv_nav_track: update class inheritance.
* agv_nav_track: initial commit.
* agv_nav_track: initial commit for agv_nav_track
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.13.1 (2018-05-08)
-------------------
* Fix AGV still moving after safety trigger and paused.
* Contributors: nikfaisal

1.13.0 (2018-04-06)
-------------------

1.12.4 (2018-03-27)
-------------------
* Fix equation for junction_stopping_distance.
* Fix bug out-of-line error not being cleared.
* Allow forward_flag to be cancelled through NavControl. Fix #19.
* Reset forward_flag speed when stop goal (NAV_IDLE) is received.
* Refactor code and make straight with forward_flag complete at actual junction distance.
* Update CMakeLists.txt
* Add missing install directive.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.3 (2018-02-27)
-------------------
* Refactor code.
* Refactor code to add VelocitySmoother class.
* Refactor code by moving init code into constructor.
* Make action_paused state resumable by safety_resume.
* Make wait_traffic one of the feedback type.
* Refactor code by adding typedef.
* Make led blink error on navigation failure.
* Remove unnecessary publishStatus right before completeNav.
* Make diagnostic publish STATUS_NORMAL when nav is idle.
* Avoid repeated publish of the same data.
* Clean up formatting.
* Improve cleanup after completing nav goal. Fix #17.
* Refactor code to add DigitalInputFilter and FlipFlop utility classes.
* Refactor code to leverage class scoping.
* Refactor agv05_nav to simplify safety check and fix #5.
* Refactor code to restrict member variable access.
* Refactor code to improve nav_control handling.
* Refactor code to improve mutex.
* Refactor code to add derived classes of ActionProcessor.
* Refactor code by renaming variables.
* Rename file.
* Refactor agv05_nav to move more responsibility into ActionProcessor.
* Refactor code to add ActionProcessor class.
* Refactor agv05_nav by grouping goal variables.
* Refactor code to use assertion.
* Refactor agv05_nav to add NavConfig and Diagnostic class.
* Refactor code by adding typedefs.
* Refactor agv05_nav to add components library.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.2 (2018-02-23)
-------------------

1.12.1 (2018-01-03)
-------------------
* Change disable obstacle pass junction to last junction laser area
* Add "Select Front Laser Profile" skills.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, nikfaisal

1.11.6 (2017-11-06)
-------------------

1.11.5 (2017-10-24)
-------------------
* agv05_nav: fix issue with branching
* Contributors: phtan

1.11.4 (2017-10-19)
-------------------
* agv05_nav: add new action for Forward Left, Forward Right, Reverse Left and Reverse Right
* Contributors: phtan

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

1.10.2 (2017-05-06)
-------------------
* Update cmake build dependencies.
* agv05_nav: add mutex lock to start nav and process function Fix #11
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.10.1 (2017-04-13)
-------------------
* Fix precision loss in distance value when using Float32.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.0 (2017-03-30)
-------------------

1.7.0 (2017-03-01)
------------------
* agv05_nav: remove safety_trigger check during junction detection, to fix issue with junction skipped after obstacle sensor activated, but junction skip will still remain after emergency button activated.
* agv05_nav: add io_trigger_stop_deceleration parameter for io trigger type stopping
* agv05_nav: action Rotate, Uturn, and Search Line will have success (line found), and failed (line not found) result
* agv05_nav: add in new action for search line
* Contributors: phtan

1.6.0 (2016-11-30)
------------------
* Fix topic name mismatch.
* Improve audio player to play from different playlist.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* agv05_nav: start and stop button can now be use to control the straight and turn movement
* agv05_nav: add in diagnostic updater
* agv05_nav: fix build error on the typo for h file include
* Merge with default
* agv05_nav: remove h file, add the ros.h include directly on the cpp file
* Merge with default
* agv05_nav: update on expansion io to use the new io node published topics
* Update package meta files.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-10-31)
------------------

1.3.0 (2016-10-16)
------------------
* agv05_core & agv05_nav: reduce running frequency to 100hz
* agv05_nav: update rotate left and rotate right action to use the new rotate_align_sensor
  parameter, selection to use front or rear sensor for rotation alignment
* agv05_nav: comment debug message
* agv05_nav: improve junction stopping accuracy for speed 0.1-0.5m/s
* agv05_nav: change io_trigger_type from 1-4 to 0-3
* agv05_nav nav.cpp: add in disable laser sensor after passing junction on the condition
  that pass junction obstacle sensor is disable
  attemp to fix the issue of near wall turning obstacle sensor detected issue.
* agv05_nav, agv05_msgs: add in new parameter to NavAction, io_trigger and io_trigger_type
  remove REVERSE_WAIT_IO action, now forward/reverse with wait IO will use standard forward/reverse
* agv05_nav: cfg file, rearrange cfg into groups
* agv05_nav: add in 2 new parameter to configure turning alignement center min time and center error
  change turning alignment to start after in-line for 0.1s
* agv05_nav: improve junction stopping algorithm
* agv05_nav: add in max angular speed limit in turning align
* agv05_nav nav: attemp to improve junction stop, adjusting acceleration/deceleration
* agv05_core: change frequency to 200Hz
  agv05_nav: change frequency to 200Hz
  agv05_motor: change frequency to 100Hz and remove debug message
* agv05_nav: add in turn align function to improve turning alignment
* agv05_nav: remove max_speed_zero method on preventing jerky movement,
  add in subscriber for navigation_enable topic from motor,
  movement only start to accelerate when navigation_enable is true
* agv05_nav: add turn alignment variable into callbackConfig
* agv05_nav: add turning alignment and new parameter to tune the turning alignment
* agv05_nav cfg: set max limit of last junction to 1.0
* agv05_nav: move navigation function from agv05_nav.cpp to independent nav.cpp files
* Contributors: phtan

1.2.0 (2016-08-25)
------------------
* agv05_nav & agv05_msgs: create a new nav action called wait traffic
* Contributors: phtan

1.1.0 (2016-08-04)
------------------
* agv05_nav: add in reverse stop io distance parameter, to be configurable
* agv05_nav: fix issue with new reverse with wait io action, new action now tested working
* agv05_nav & agv05_msgs: add in new action, reverse with wait IO, will be use for
  hooking action
* agv05_line_sensor: fix linear error moving average
  agv05_nav: fix forward flag slow down issue
* agv05_nav: add new parameter for nav called last_junction_forward_speed_limit,
  this parameter will control the maximum speed for the straight navigation if the straight nav is non forward flag type,
* Contributors: phtan

1.0.0 (2016-07-21)
------------------
* agv05_nav: disable laser sensor after junction passed, (to add this into dynamic reconfigure)
* agv05_nav: add in error led blink upon charger connected status
* agv05_nav: add in safety_charger_connected_flag, which will be triggered if manual charger or auto charger is connected/docked.
  this safety flag will be same as the other safety like bumper and emergency button which require user to acknowledge the release
  of this safety before continue on the operation.
* agv05_nav: variable rename, should not affect any algorithm or logic
* agv05_nav: add in new dynamic reconfigure variable, each nav profile will have maximum speed limit,
  which will overwrite the speed given by executor.
* agv05_nav & NavAction: add in Free Motor navigation mode for free motor operation
* agv05_nav: turn off alarm upon completion of the task
* agv05_nav: add in control on alarm during safety trigger
* agv05_led, agv05_msgs & agv05_nav: change blink error to constant Red, and change safety trigger to blink red
* agv05_nav: activate max_speed_zero on start for straight and turn function,
* agv05_nav: add in zero speed control, that will set the speed for zero for a 0.8s
  if agv come back from safety trigger of cold start
* agv05_nav: add in safety nav trigger publish is safety_trigger is still set, this solve the motor driver error issue
* agv05_nav: add in filtering for safety_trigger, that safety_trigger will clear itself laser flag release,
  only if the trigger is caused by laser sensor
* agv05_nav: remove the safety_trigger clear
* agv05_nav: change turning sensing angular error, make turning stop earlier to give
  more adjustment space for turn deceleration
* agv05_nav: change the status to obstacle in range in the condition that far area for laser sensor is activated
* remove safety_trigger bypass in nav
* agv05_nav:add in led blink control into publish status function
* agv05_laser_sensor: fix issue on not channelling front laser sensor result to the laser sensor output
  agv05_nav: edit delay time for laser scanning
* agv05_nav: complete changes on the acceleration and deceleration profile for agv movement (straight and turn)
* agv05_nav: temp commit  on acceleration & deceleration changes in dynamic reconfigure (not tested)
* agv05_nav cfg file: change acceleration and deceleration for straight and turn into profile (total 5 profile,
  same as line follow PID)
* agv05_nav: add in publishWarning start and stop function
* agv05_nav: enable forward flag for forward and reverse action
* agv05_nav: add in safety nav trigger for turn, manual control and line calibration
* agv05_nav: add in nav_scan_laser_profile 1-5,
* agv05_nav: add in result for action, add in out_of_line_distance,
* agv05_nav: add in navigation profile 5 set of different profile
* agv05_nav: remove unused OUT_OF_LINE submode, seperate safety nav trigger to laser and out_of_line,
  and out_of_line will not trigger after OVER_JUNCTION submode, use normall_acc for acceleration at
  ON_JUNCTION and OVER_JUNCTION to prevent error of AGV stop at junction if safety trigger happen there,
  remove PID output print to terminal
* agv05_nav: control line sensor calibration process inside line calibrate action
* agv05_nav: rename line calibration to line calibrate, add in calibrate action inside process
* agv05_nav: add in new action, line calibration
* agv05_nav: add in manual control acc & dec for dynamic reconfigure,
  add new topic to subscribe "agv05/control/nav/manual_cmd_vel" will be use for speed control during manual control,
  add in new action in nav, manual control for UI to perform manual control without directly accessing cmd_vel topic.
* agv05_nav: add in more variable configuration for dynamic reconfigure,
  subscribe straight_distance and rotational_distance from motor node,
  straight function add in junction stop,
  add in turn function
* agv05_nav_test: enable control for uturn
* agv05_nav_test: update chmod for the file
* agv05_nav: update message for action received,
  update line follow to have direction,
* agv05_nav:
  add in dynamic configuration for line following variables,
  fix front and rear msb topic naming to follow new line sensor topic
  add rate for action status update,
* agv05_nav_test: update with status display and
* agv05_nav: file commit for nav initial framework
* Contributors: nuxail, phtan

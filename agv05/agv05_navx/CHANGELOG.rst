^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_navx
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'nav-omni' into 'master'

  * Limit manual control output angular-linear speed ratio to input angular-linear speed ratio.

* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.
  * Add safety heartbeat publishing.
  * Add safety heartbeat monitoring.
  * Add heartbeat monitoring on map to odom TF.

* Merge branch 'nav-improve' into 'master'

  * Re-accelerate lateral speed from 0 during low straight speed.
  * Fix safety area disabled during non-stop transition.
  * Limit next speed with approximated next Bezier path curvature.
  * Keep manual control activate until task runner is resumed from paused.
  * Mute obstacle sensing including bumper during manual control.

* Merge branch 'nav-omni' into 'master'

  * Reduce path speed for omni aligning.
  * Add omni turn actions.
  * Add velocity smoother in lateral movement of omni action.

* Merge branch 'manual-control-when-paused' into 'master'

  * Handle manual control during paused state.

* Contributors: Patrick Chin, Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Remove header files from debian package.
* Merge branch 'nav-doa' into 'master'

  * Fix dynamic action false end during on-spot rotation.
  * Deceleration stop before prompting path blocked.
  * Add new parameter of dynamic path following tolerance.

* Merge branch 'nav-improve-p3' into 'master'

  * Add parameter to change find marker timeout duration during (un)docking.

* Merge branch 'nav-omni' into 'master'

  * Update omni align reaching distance threshold.
  * Add motor fault hint.
  * Allow fine lateral correction at low moving speed.
  * Fix Io Trigger Distance in Omni (wait IO) (trackless) skill does not work in lateral movement.
  * Add handling to omni actions.
  * Add omni action types.

* Merge branch 'nav-improve-p3-dynplan' into 'master'

  * Calculate align reaching distance threshold base on maximum path speed instead of current speed.
  * Calculate next via-point of dynamic path using range-based instead of angle-based threshold.
  * Refactor code.
  * Reserve dynamic plan only for non-stop motion.
  * Bind mid goal flag to plural goal paths.
  * Fix dynamic action slow moving speed if parameter path_follow_heading_error_max >90deg.
  * Fix dynamic action final rotation >180deg.

* Merge branch 'nav-improve-p3-safety-area' into 'master'

  * Add option to limit maximum speed if personnel detection means in travel direction is permanently muted according to ISO 3691-4.
  * Different safety area for different safety block speed range.

* Merge branch 'nav-improve-p3' into 'master'

  * Refactor code.

* Merge branch 'nav-improve-p3-omni-dock' into 'master'

  * Add omni docking action.

* Merge branch 'nav-improve-p3-motor' into 'master'

  * Update forward flag speed in the beginning of action according to latest motor command velocity.

* Merge branch 'nav-improve-p3-io' into 'master'

  * Add final undershoot timeout.
  * Add option to change IO trigger debouncing time.
  * Change IO trigger distance to use wheel odometry instead of trackless localization.
  * Add IO trigger distance in wait IO actions.

* Fix noetic-roslint warning.
* Fix for CGAL before version 4.10.
* No safety triggered but hardware is not ready. Use small speed to wake up hardware.
* Fix return-type warnings.
* Merge branch 'nav-improve-p3-nonstop-transition' into 'master'

  * Add obstacle sensor deactivation delay after non-stopping actions done.

* Merge branch 'nav-improve-p3-dim' into 'master'

  * Limit path curvature of dynamic actions according to robot radius.

* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'add-navigation-profile-into-diagnostic'

  * Add navigation profile into diagnostics.

* Merge branch 'fix-parameter-update'

  * Fix parameter reset failed with agv05_navx and agv05_obstacle_sensor.

* Merge branch 'zalpha-3.3.0'

  * Align rosparam name in yaml file to be same as name in config.

* Merge branch 'protected-parameter'

  * Suffix reset profile config with underscore. Update default nav config values.

* Merge branch 'fix-turn-360'

  * Fix rotation >350deg if localization jump crossing target right before start turning a small angle.

* Merge branch 'nav-improve-p2'

  * Add bumper blocked hints.
  * Use reaching deceleration instead of normal deceleration for next speed.
  * Remove next motion of non-stop for dynamic path.

* Merge branch 'time-jump-safety-trigger'

  * Trigger safety when time jump happens.

* Merge branch 'sense-line-options'

  * Add new sense_line mode in NavX which requires sensing full junction.
  * Rename variable 'sense_line' to 'use_line_sensor' for clarity.

* Allow setting of preferred zone without path.
* Merge branch 'nav-improve-p2-path-blocked'

  * Fix 'Path Blocked' is not clear-able although a new path is found if another safety triggered is gone but latched beforehand.
  * Fix unintended clearing of 'Path Blocked' due to racing condition.
  * Fix 'Path Blocked' not shown in manual resume dialogue if other auto-resumed safety triggered beforehand.
  * Allow user to manually clear all registered obstacle after timeout of dynplan_retry_time.

* Merge branch 'nav-improve-p2-dynplan'

  * Increase the output limit during on-spot rotation for dynamic action.
  * Add duration of costmap update and make plan into diagnostic.
  * Add parameter to change planner frequency.
  * Fix dynamic action false end if navigation failed.
  * Dynamic action handles path towards final goal.
  * Dynamic action publishes motion paths and area.
  * Refactor planner thread.
  * Fix occasional dynamic path skipped.

* Merge branch 'nav-improve-p2-costmap'

  * Add selection of obstacle sensors which are used during Dynamic Path Planning.
  * Change variable type from std::map to std::vector to skip searching time during enqueue.
  * Refactor and optimize path layer.
  * Add option to change path border cost.
  * Costmap path layer updates with received paths and area which are cleared upon stop.
  * Update to new message type of motion paths and area.

* Merge branch 'nav-improve-p2-turn'

  * Safety trigger for MSB calibration error if trackless sense line is enabled.
  * Publish laser activation hint as safety internal message.
  * Rename line sensor com error to line sensor error.
  * Change line sensor sensor_error into error code.
  * Check for msb calibration validation.

* Merge branch 'nav-improve-p2-pid'

  * Change PID control line following heading gain to look ahead distance.

* Merge branch 'nav-improve'

  * Add option to change straight out of path distance.
  * Fix AMR circulate around stopping target with dynamic path.
  * Calculate Bezier path linear error by using 2D-pose of current goal instead of estimation line from current point to current goal.
  * Add option to have traditional PID controller for path following. Update default value of navigation parameters.

* Add linear, angular & heading error to navx status.
* Fix typo in parameter description.
* Contributors: Andrew Tan, Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Merge branch 'nav-profile' into 'master'

  * Add navigation profiles enumeration service for drop down list selection.
  * Round value during float to double conversion.
  * Add support to 3 preset profiles for Nav and NavX nodes.

* Merge branch 'track-odom'

  * Add handling to Forward/Reverse (by distance) skills with odometry feedback.

* Merge branch 'jerk-limit' into 'master'

  * Add turn_alignment_center_error into reaching speed calculation.
  * Add parameter of stopping deceleration which to be used in reaching kinematic calculation.
  * Nested tab display for navigation action and safety parameters.
  * Add overshoot into diagnostic status.
  * Consolidate reaching distance calculation into function computeReachingDistance(1).
  * Add localization error tolerance into reaching distance and speed calculation.
  * Calculate path curvature and distance base on path speed limit.
  * Remove instant stop of command speed due to AMR status. Stop immediately if previous command speed is less than minimum speed and overshoot above threshold distance.
  * Reset path following variables after passing inflection point of Bezier path.
  * Limit aligning speed to user-given value in case of jerk limiting.
  * Nesting trackless stopping configuration.
  * Add trackless path speed limitations.
  * Add parameters for customizing Bezier look ahead distance as in pure pursuit algorithm. Update default value of navigation parameters.
  * Implement Anti-Reset Windup and Derivative Kick according to https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralDerivative.
  * Add stopping offset to compensate overshoot due to minimum output speed deadband of Mabuchi motor.
  * Merge PID controller for linear and heading error.
  * Calculate Bezier path points heading by using formula instead of estimation.
  * Stop immediately if previous command speed is less than minimum motor speed when it is time to stop.
  * Add turn action overshot handling for inverse kinematic.
  * Change close-loop P control to inverse kinematic for turning actions in order to limit jerk.
  * Add utility class ReachingKinematic for converting target distance to-from target speed.
  * Add jerk limitation parameters and handling for trackless action.

* Ignore original target distance once sensing line.
* Merge branch 'refactor'

  * Refactor planner thread.
  * Refactor code.

* Merge branch 'path-planner' into 'master'

  * Fix NAV_DYNAMIC_FORWARD and NAV_DYNAMIC_REVERSE do not work with goal tolerance of -1.
  * Fix AMR not moving when heading_error > threshold while heading_error and angular_error cancel out each other.
  * Allow AMR to stop within the given tolerance even if the stopping junction is clear from obstacle.
  * Fix wrong condition checking for stopping costmap.

* Contributors: Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------
* Merge branch 'bezier-deceleration' into 'master'

  * Decelerate and rotate on-spot if AGV with high load and high speed failed to follow Bezier curve.
  * Revert files attribute changed after rebase.
  * Handle deceleration when next motion is bezier type.

* Merge branch 'path-planner' into 'master'

  * Add parameter to select direction of on spot rotating.
  * Fix error in estimation of the Bézier arc-length.
  * Fix linear motion during rotating on-spot.
  * Add parameter dynplan_search_time to allow path searching for an interval by on spot rotating before reporting "Path blocked".
  * If dynamic path planning feature flag is set, initialize costmap and planner when localization is ready.
  * Add trackless action initialization duration into diagnostic.
  * Fix missing status string.
  * Stop costmap from running if planner is shutdown.
  * Limit speed of forward flag set by MOTION_NONSTOP_DYNAMIC.
  * Make plan once in startPlanner and delay planner thread closing by 5 cycle time after stopPlanner to avoid stopping at every mid junction due to planner thread restarting and waiting for a new plan.
  * Add next motion of non-stop dynamic path.
  * Refactor code.
  * Update default value of agv05_navx configuration parameters.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------
* Refactor code.
* Reduce frequency diagnostic averaging window to 1s.
* Fix I coefficent scale in line and path follow PIDs.
* Merge branch 'steering-drive'

  * Allow configuration for path follow angular output saturation.

* Merge branch 'path-planner'
  * Skip fine positioning to mid junction goal.
  * Add dynamic configuration of default mid and goal juntion tolerance.
  * Add support to multiple static paths at a time.
  * Latch STATUS_PLAN_EMPTY instead of reporting STATUS_NAVIGATION_FAILED after time out.
  * Update definition of target distance, angular error and heading error of dynamic actions. Limit dynamic actions running speed according to distance to sub goal for better path following.
  * Update static path layer default config parameters.
  * Update obstacle sensor profile handling for dynamic forward and reverse. Refactor code.
  * Fix end point heading is not processed in dynamic action for next motion is idle. Negative end point heading will be ignored.
  * Add checking of dynamic path planning retry interval before navigation failed.
  * Change name of class "dynamic forward" to "dynamic" as a generic class for all dynamic action goal nav.
  * Add publishing of static path topic for costmap plugin and checking for maximum distance of dynamic constrained path.
  * Add handling to new dynamic NavxAction nav goal.
  * Add handling to NavxAction "goal_tolerance".
  * Update message and config for dynamic path planning.
  * Add new costmap plugin layer of static pre-planned path.
  * Add individual dynamic_reconfigure config file for each plugin layer.
  * Add new costmap plugin layer of Forbidden Zone.
  * Clear existing plan when stopping planner.
  * Add `plan empty` nav action feedback.
  * Add dynamic forward action.
  * Update costmap and planner update by combining them in a single thread.
  * Add planner.
  * Add costmap.
  * Add debug log for nav.

* Trigger line sensor error if it is disabled during navigation.
* Merge branch 'stacker-docking'

  * Extend docking action with error IO trigger.

* Fix bug for io trigger update duration.
* Merge branch 'next-motion-hint'

  * Assert velocity smoother work with positive magnitudes only.
  * Remove ineffective asserts due to integer promotion.

* Replace forward_flag with next_motion parameter.
* Disable obstacle after safety triggered and add config to disable it.
* Merge branch 'steering-drive' into 'master'

  * Make nav publish a small value to motor as hint to align steering.

* Merge branch 'next-navigation-action-hint'

  * Use next nav action to set turn signal led and stopping offset.

* Merge branch 'merge-agv05x-repo' into 'master'

  * Migrate changelogs.
  * Import agv05_navx package from agv05x repo.

* Add missing diagnostic status for docking actions.
* Generate fully straight docking plan when alignment distance is zero.
* Split PID controller for linear and heading error.
* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin

1.14.2 (2020-10-20)
-------------------
* Merge branch 'docking'

  * Use marker_localization node instead of handleReflector.
  * Add undocking actions using reflectors.
  * Use reflectors for docking.

* Differentiate forward and reverse stopping distance after line sensed.
* Fix typo of `deceleration`.
* Make the transition from tf to tf2.
* Merge branch 'node-consolidation'

  * Increase straight line speed limit for nav.
  * Move topics and msgs from agv to agv05 namespace.
  * Handle rename of agv_nav_track to agv05_nav.

* Contributors: Patrick Chin, Tang Swee Ho

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-15)
-------------------
* Add wheel-slippage related safety flag and nav action feedback.
* Add robot pose to diagnostic in agv05x_nav.
* Add line sensing option to stop after some distance past the line.
* Blink left or right LED when rotating.
* Contributors: Farhan Mustar, Patrick Chin

1.13.16 (2020-01-12)
--------------------
* Fix bug agv position offset to the right of the path not triggerring nav failure.
* Contributors: Patrick Chin

1.13.15 (2019-11-22)
--------------------
* Merge branch 'forward-flag-fix'.

  * Use normal obstacle area when ending with forward flag.

* Merge branch 'missed-safety-feedback'.

  * Ensure the safety error is published at least once by navx action server.

* Merge branch 'turn-alignment-fix'

  * Prevent turn alignment timeout when safety triggerred.

* Add option for auto-resume after external safety trigger.
* Increase the number of laser profiles to 10.
* Contributors: Patrick Chin

1.13.14 (2019-05-15)
--------------------
* Remove redundant safety braking feature.
* Change motor overload to motor fault.
* Handle new motor overload safety trigger in navx.
* Contributors: Nik, Patrick Chin

1.13.13 (2019-04-05)
--------------------
* Add deceleration to prevent abrupt stop at reaching stop.
* Avoid updating AGV pose twice in a Bezier process loop.
* Remove unused parameters.
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
* Advance bezier cursor based on distance.
* Contributors: Farhan

1.13.7 (2018-10-19)
-------------------
* Remove the ability to pause and resume nav through physical buttons. Fix #2.
* Separate safety resume from action resume.
* Remove unnecessary CompositeDiagnosticTask and HeaderlessTopicDiagnostic.
  Use FrequencyStatus directly.
* Contributors: Patrick Chin

1.13.6 (2018-09-29)
-------------------

1.13.5 (2018-09-07)
-------------------
* Remove unnecessary files.
* Contributors: Patrick Chin

1.13.4 (2018-08-18)
-------------------
* Handle safety flag safety_io_1 and safety_io_2 in navx.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------
* Allow button actions to be configurable in nav.
* Flip the definition value of nav_trigger.
* Add select_laser_profile action for agv05x_nav.
* Add NavAction feedback types including line_sensor_com_error, laser_malfunction and drive_overlimit_error.
* Adapt to changes in safety message input.
* Handle rename of agv05_nav into agv_nav_track.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.1 (2018-05-08)
-------------------
* Fix AGV still moving after safety trigger and paused.
* Contributors: nikfaisal

1.13.0 (2018-04-23)
-------------------

1.12.4 (2018-03-12)
-------------------
* Allow forward_flag to be cancelled through NavControl.
* Reset forward_flag speed when stop goal (NAV_IDLE) is received.
* Reimplement agv05x_nav.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.3 (2018-02-28)
-------------------

1.12.2 (2017-12-30)
-------------------
* Fix dynamic reconfigure stuck issue.
* Contributors: nikfaisal

1.11.4 (2017-10-09)
-------------------

1.11.3 (2017-09-30)
-------------------

1.11.0 (2017-08-14)
-------------------
* Fix bug: navx doesn't terminate on abort.
* Reset nav_control_callback_trigger\_ in startNav. Fix #3.
* Add mutex to ensure atomic assignment in startNav. See #3.
* Reduce the amount of log from navx. Fix #4.
* Fix laser speed limit area control #5
* Improve various navigation.
* Add navigation failure safety.
* Add agv05x nav diagnostics
* Remove old rotate function
* Fix reset linear_error_angle\_ during rotate
* Fix bezier skip after nav switch
* Fix bezier movement.
* Add pid module and handle acceleration.
* Handle forward flag in nav_straight.
* Improve target error calculation.
* Fix published safety status.
* Fix straight and rotate movement.
* Add missing rotate_heading parameter.
* Add complete_nav() function.
* Add laser sensor and safety check.
* Add new dynamic reconfigure parameters for agv05x_nav.
* Add navigation profile for line follow and linear position
* Add safety msgs and topic
* Add navigation forward, bezier forward, and rotate.
* Add agv05x_nav package stub.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

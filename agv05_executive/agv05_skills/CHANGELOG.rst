^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_skills
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'nav-improve' into 'master'

  * Add Wait Resume skill for user to trigger manual control or human follower manually.

* Merge branch 'compose-string-action'

  * Add action to compose string and set string from other type.

* Add CheckDiagnosticStatus action.
* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin

3.1.0 (2024-10-14)
------------------
* Handle bytearray in python3.
* Remove unnecessary code.
* Merge branch 'python-six' into svelte-webapp

  * Update dict view.
  * Update string check.
  * Use zip from six for compatibility.
  * Update hashlib md5 and zip for python 3 compatibility.

* Merge branch 'fix-transition-trigger-bug' into 'master'

  * Fix rotate_align not following instructed transition. When actual angle difference != 180 at u-turn junction.
  * Fix transition is not triggered after teleport with heading changed.

* Merge branch 'twtc'

  * Skip sending occupation request in SelectDimensionProfile if previous task aborted.

* Merge branch 'hal' into 'master'

  * Wait until Idle state before failure return to avoid user spamming on Dock Charger skill.
  * Change dock_charger_timeout from dynamic reconfigure to ROS parameter.
  * HAL battery state diagram.

* Fix test.
* Merge branch 'fix-map-tracker-confused' into 'master'

  * Fix MapTracker is confused during transition from trackless to tracked path.

* Add option whether to disable laser sensor when heading into home.
* Merge branch 'nav-omni' into 'master'

  * Increase the docking target distance for stacker. Refactor code.

* Merge branch 'fix-transition-trigger-bug' into 'master'

  * Fix transition trigger angle comparison bug.

* Merge branch 'nav-improve-p3-omni-dock' into 'master'

  * Add omni docking action.

* Merge branch 'nav-improve-p3' into 'master'

  * Add option for junction miss checking with long forward/reverse junction distance.

* Merge branch 'nav-improve-p3-io' into 'master'

  * Add IO trigger distance in wait IO actions.

* Fix noetic-roslint warning.
* Add SetIntFromRegister skill.
* Merge branch 'modbus-skills'

  * Add modbus skills.

* Merge branch 'prehandle-transition-cancel-non-stop' into 'master'

  * Pre-handle transition trigger cancel non-stop.

* Fix cancel non-stop dynamic motion when change in moving direction.
* Contributors: Farhan Mustar, Patrick Chin, Quan Bong, Tan Kui An Andrew, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Add seen reply in traffic ack.
* Send occupation request for final update.
* Enforce correct position tracking after teleport or transition.
* Merge branch 'hot-reload'

  * Add reloadable from fms.

* Merge branch 'traffic-communication-protocol'

  * Discard out-of-order traffic reply.
  * Update location format in traffic request.
  * Handle error field in traffic reply.
  * Fix next_motion value at swapping junction.
  * Add rotate motion to distance cost.
  * Add skill mutex of traffic type.
  * Resend occupation request when DFleet's run id changes.
  * Make AGV send nav acknowledgement to DFleet.
  * Refactor transition-trigger code.
  * Include AGV current dimension profile in nav_request.

* Remove motion calibration skills to avoid triggered by task runner.
* Merge branch 'bail-out-top-level-task'

  * Add error message parameter and trigger error so that it is loggable.
  * Add abort task skill.

* Merge branch 'no-rotate-zone'

  * Accept plan transitions from DFleet.
  * Update NavigateTo skill to rotate in the correct direction.
  * Update check_transition to consider no-rotate zones.
  * Add rotate three-quarter action.

* Merge branch 'transition-trigger'

  * Sync changes from DFleet for transition_trigger.

* Cleanup after test.
* Sync changes from DFleet.
* Merge branch 'path-forward-reverse-motion'

  * Allow trailer AGV to rotate at teleport start if teleport has non-stop transition.
  * Update NavigateTo skill to observe path forward or reverse motion.
  * Add tests.

* Merge branch 'sense-line-options'

  * Add new sense_line mode which requires sensing full line in NavX actions.

* Merge branch 'swapping-direction'

  * Execute the intermediate swapping direction sent by DFleet.

* Merge branch 'tracked-branching-bug-fix'

  * Sync changes on plan_x.py with plan.py.

* Merge branch 'allow-trackless-home'

  * Undock whenever travel out from home station.
  * Fix forward flag of flip motion.
  * Refactor code. Accept all path types to-from home in low level functions of navigate skill.

* Merge branch 'nav-improve-p2'

  * Remove next motion of non-stop for dynamic path.
  * Add triggering on track curve line action.

* Refactor code and replace magic number.
* Merge branch 'tracked-branching-for-trackless'

  * Add tracked branching support.

* Merge branch 'nav-improve-p2-dynplan'

  * NavigateToX skill to include all continuous paths into goal.
  * Skip rotation in the beginning of dynamic path.

* Merge branch 'nav-improve-p2-turn'

  * Add search line during transition from trackless to track with retry of parameter search_line_trial_max times.
  * Add argument of target angle to track line into search line skills.

* Merge branch 'robot-svg'

  * Add SelectDimensionProfile skill.

* Merge branch 'global-variable'

  * Update variable skill ui name.
  * Add variable skills and fix execution error.

* Merge branch 'teleport-reset-position-tracker'

  * Add option to reset position or reset position tracker after teleport.

* Merge branch 'teleport-align'

  * Add align_station_type option for teleport.

* Fix missing info value for CompareAndSetRegister.
* Merge branch 'tune-pid'

  * Add PID tuning action.

* Merge branch 'realtime-pose-update-on-simulator-robot'

  * Fix test DB setup.

* Update test settings to use local memory cache backend.
* Contributors: Andrew Tan, Chow Yeh Loh, Farhan Mustar, Patrick Chin, Rui En Lee, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Fix bug non-stop transition from trackless to tracked path during final dock to home.
* Merge branch 'track-odom'

  * Add Rotate (by angle) skill with odometry feedback.
  * Add option to Forward/Reverse (by distance) with odometry feedback.

* Merge branch 'jerk-limit' into 'master'

  * Update name and description of parameter forward_speed_limit_by_next_path_length.
  * Fix _get_path_speed() function does not work for teleport path.
  * Add next_speed as skills input argument.
  * Add next_speed and next_distance sending in goal.

* Merge branch 'trackless-tracked-fix'

  * Sanitize next_motion input parameter if final path is tracked.
  * Fix bug in sending trackless next_motion for tracked path.

* Merge branch 'reset-agv-position-tracker-map-switching' into 'master'

  * Fix ResetAgvPositionTracker failing to switch to another map properly.

* Prevent raising error for status code 409 in http skills.
* Merge branch 'refactor'

  * Add option whether to always rotate align at the destination.
  * Refactor code.
  * Flip heading for GoToMarkerWaypoint skill.

* Fix sense_line wrongly issued when waiting traffic instead of during final path.
* Merge branch 'set-parameters-skill'

  * Add generic SetParameters skill.

* Merge branch 'reflector-matching'

  * Add match reflector skill.

* Merge branch 'track-rotate-between-tracked-path'

  * Use tracked rotate for transition between tracked path.

* Contributors: Chow Yeh Loh, Farhan Mustar, Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------
* Eliminate the use of ujson library.
* Merge branch 'bezier-deceleration' into 'master'

  * Handle deceleration when next motion is bezier or dynamic type.

* Merge branch 'new-skills'

  * Add option to disable ssl verification in HTTP skills.
  * Add CompareStationPrefix and CompareStationSuffix skills.
  * Add CheckAgvPositionPrefix and CheckAgvPositionSuffix skills.
  * Add "Rotate to Heading" skill.

* Merge branch 'path-planner' into 'master'

  * Send appropriate goal_tolerance and next_motion for dynamic path.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------
* Refactor code.
* Remove delay waiting for traffic reply.
* Fix skill message.
* Merge branch 'path-planner'

  * Send appropriate goal tolerance for mid and final junctions.
  * Disable dynamic path planning unless feature flag is set.
  * Add handling to default mid and goal junction tolerance.
  * Force station heading alignment.
  * Provide next_motion to teleport action.
  * Add GoToMarkerWaypoint skill.
  * Update NavigateTo action to handle dynamic path in map.
  * Update default values of dynamic path planning skills.
  * Add end point heading into linear and Bezier motions. Change end point heading to positive value as negative value will be ignored in dynamic action.
  * Update GoToWaypoint and GoToStationWaypoint skills to allow setting of moving direction and goal tolerance.
  * Add trackless dynamic line and bezier nav skills.
  * Change 'forward_flag' to 'next_motion' as per update in NavxAction.action.
  * Add NavigateToStationWaypoint skill.
  * Amend previous changeset.
  * Add path planning skill.

* Merge branch 'stacker-docking'

  * Fix missing import.
  * Allow disabling IO detection in wait IO skills.
  * Add trackless nav wait IO skills.
  * Remove error outcome in trackless nav skills.
  * Extend forward wait IO skill to add error trigger.
  * Extend docking skill to add error trigger.

* Merge branch 'find-marker' into 'master'

  * Add skill to find marker position relative to AGV base.

* Merge branch 'next-motion-hint' (!35)

  * Compute next_motion for final dock to home.
  * Compute next_motion for initial undock from home.
  * Replace forward_flag with next_motion parameter in NavigateTo skills.
  * Replace forward_flag with next_motion parameter in nav skills.

* Merge branch 'registers-arithmetic'

  * Add unit test for register skills.
  * Add skills for registers arithmetic.

* Merge branch 'merge-agv05x-to-agv05-repo'

  * Merge agv05x_msgs into agv05_msgs package.

* Fix bug position reset not skipped when teleport is aborted.
* Fix typo in docking skill.
* Fix typo in HTTP Post skill.
* Add skill to reset only AGV position tracker.
* Contributors: Farhan Mustar, Patrick Chin, Tang Swee Ho, Wong Tze Lin, Nik Faisal

1.14.2 (2020-10-23)
-------------------
* Merge branch 'docking'

  * Modify NavAction msg and refactor code.
  * Rename string of marker_type.
  * Add marker_type.
  * Add dock and undock actions.
  * Add DockToMarker action.

* Refactor nav and navx skills.
* Increase navigation skills speed limits.
* Merge branch 'node-consolidation'

  * Provide shutdown service.
  * Move topics and msgs into agv05 namespace.

* Improve skill param description.
* Contributors: Patrick Chin, Tang Swee Ho

1.14.1 (2020-06-03)
-------------------
* Improve skill param description.
* Contributors: Patrick Chin

1.14.0 (2020-05-16)
-------------------
* Fix position checking bug in trackless mode.
* Merge branch 'navigate-sense-line'

  * Add sense_line option in various trackless nav actions.
  * Add sense_line option in trackless NavigateTo action.

* Merge branch 'forward-by-distance-offset'

  * Add option in forward and reverse by distance skills to be measured only after junction.

* Add timestamp in nav_request message.
* Merge branch 'branching-improvement' into 'master' (`!23 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/23>`_)

  * Add flag to supress tracking the branch direction to avoid map tracker warning.
  * Update plan to use branching for merging path.

* Merge branch 'validate-rfid-after-teleport'. Fix `#50 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/50>`_.

  * Add check for pause signal before teleport.
  * Add option to validate RFID after teleport.

* Merge branch 'forward-flag-input-for-teleport'. Fix `#48 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/48>`_.

  * Disable forward flag if resetting position after teleport in trackless mode.
  * Add forward flag input as teleport parameter.

* Merge branch 'map-params'

  * Update NavigateTo action to use map parameter for path speed.
  * Add customizable path speed levels for map editor.

* Merge branch 'allowed-motion'

  * Fix ambiguity in determining plan's end.
  * Validate path constraints before accepting it from FMS.
  * Update traffic request to include start and end heading.
  * Implement exhaustive search for constrained path.
  * Add extra AGV executor setting (allowed_motions).

* Contributors: Farhan Mustar, Patrick Chin, Xin Wei Lee

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-22)
--------------------
* Fix bug in hint_pos equality comparison. Make hint compulsory for trackless map tracker.
* Add RebootAGV skill.
* Implement skills for HTTP requests.
* Increase the number of laser profiles to 10.
* Contributors: Farhan Mustar, Patrick Chin

1.13.14 (2019-05-27)
--------------------
* Modify forward and reverse wait IO skill to handle multi-port IO.
* Add port number for IO skills.
* Contributors: Nik, Patrick Chin

1.13.13 (2019-03-24)
--------------------
* Migrate to package.xml version 3 and add copyright file.
* Fix incorrect logging from skill.
* Add atomic compare-and-set register action.
* Guarantee atomicity of all register operations.
* Contributors: Farhan, Patrick Chin

1.13.12 (2019-01-21)
--------------------

1.13.11 (2019-01-12)
--------------------
* Fix UpdateTaskProgress log.
* Skip rfid validation if NavigateTo action has been preempted.
* Add fail_task option in ReadRFIDAndCompareString skill.
* Write rfid error into custom log.
* Fix bug autoResetAgvPosition in teleport causing unreservation in traffic controller.
* Contributors: Farhan, Patrick Chin

1.13.10 (2018-11-30)
--------------------

1.13.9 (2018-11-28)
-------------------

1.13.8 (2018-11-27)
-------------------
* Add forward-by-distance and reverse-by-distance skills.
* Use actual pose to determine direction of rotation in NavigateToX.
  Fix AGV rotate 360-deg in the wrong direction.
* Remove auto-dock charger in NavigateTo skill.
* Contributors: Farhan, Patrick Chin

1.13.7 (2018-10-20)
-------------------
* Fix bug ResetAgvPosition within teleport causing unreservation in traffic controller.
* Add LogMessage skill. Fix #46.
* Update led blink control when task runner is paused.
* Fix trackless NavigateTo action to check for paused state after waiting for traffic.
* Remove unused rotation speed parameters from NavigateTo skills.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------
* Add is_paused method for skill and update NavigateTo action to check for
  pause state after waiting for traffic.
* Contributors: Farhan Mustar

1.13.4 (2018-08-18)
-------------------
* Reset RFID on all forward and reverse actions.
* Send nav_request to FMS after position reset.
* Reset RFID on forward and reverse action.
* Make FMS navigation request asynchronous to prevent AGV miscount.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.13.3 (2018-07-16)
-------------------
* Fix ScanLaserArea skill.
* Add RFID validation for NavigateTo skill.
* Improve clarity when displaying skill error message.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, Xin Wei Lee, farhan

1.13.2 (2018-05-18)
-------------------
* Add CheckAgvPositionRfid skill. Fix #35.
* Update power topics and messages.
* Update remoteIo device id to start from 1.
* Modify definition values for ForwardWaitIo and ReverseWaitIo skills.
  - io_trigger will use zero-based indexing.
  - io_trigger_types will start from 1 and some enum values are swapped.
* Change IoRead and IoWrite skills to use 0-based indexing for pins. Fix #25.
* Move NavAction and NavControl definition into agv_msgs.
* Add power off AGV skill.
* Add RFID compare string skills.
* Change selectFrontLaserProfile to selectLaserProfile skill.
* Change minimum speed for nav and nav_x skill to 0.01.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-20)
-------------------
* Ensure stop_wait_traffic before sending the next action to agv05_nav.
* Send nav request to FMS in two parts, ie when agv leave and reach a junction.
* Increase timeout for DockCharger skill.
* Reduce sleep cycle time in skills.
* Accept plan_hint from traffic controller.
* Reduce loop delay for remote io skill.
* Modify ReadRFIDAndResetAgv skill to use junction RFID instead of station name.
* Add proper forward and reverse action in NavigateTo skill based on branching.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, farhan

1.12.4 (2018-03-12)
-------------------
* Send cancellation of forward_flag when skill is preempted. Also fix #24.
* Make nav skills non-preemptible.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.3 (2018-02-27)
-------------------

1.12.2 (2018-02-23)
-------------------
* Improve reset function for RFID.
* Fix bug in RemoteIoWriteRegister skill.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.12.1 (2018-01-03)
-------------------
* Add copy register skills.
* Add SelectFrontLaserProfile skill.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>

1.12.0 (2017-12-03)
-------------------
* Add `non-stop-transition` feature in teleport.
* Fix bug where NavigateTo action is preempted when forward flag is still active.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.7 (2017-11-16)
-------------------

1.11.6 (2017-11-06)
-------------------
* Add `prev_location`, `motion`, and `velocity` to AGV status API.
* Fix bug where task hangs when there is not a valid path to station (FMS mode).
* Fix bug where executor hangs when it fails to receive traffic approval from FMS.
* Add task progress field and add UpdateTaskProgress skill.
* Update description of some skills.
* Add non_stop_final_junction option for NavigateTo skill.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.5 (2017-10-24)
-------------------
* Add in branching skills.
* Contributors: phtan

1.11.4 (2017-10-19)
-------------------

1.11.3 (2017-10-02)
-------------------
* Make map_tracker_x more robust to custom trackless navigation commands.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.2 (2017-09-26)
-------------------

1.11.1 (2017-09-13)
-------------------

1.11.0 (2017-08-22)
-------------------
* Add RFID skills.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>

1.10.6 (2017-07-25)
-------------------
* Update skill to support new remote io node.
* Contributors: phtan

1.10.5 (2017-07-20)
-------------------

1.10.4 (2017-07-12)
-------------------
* Implement teleport action and pre-action execution.
* Ensure every skill is preemptible.
* Add resource mutexes to skill meta.
* Remove the effect of align_station_type on the alignment prior to home-docking.
* Reorder skill list.
* Add remote io read/write register skills.
* Add Io read/write register skills.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.10.3 (2017-05-18)
-------------------

1.10.2 (2017-05-08)
-------------------
* Add skills for remote io read and write.
* Add skill CheckAgvPosition.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.10.1 (2017-04-14)
-------------------
* Add the "Sleep" skill.
* Refactor code to change reference of "ccs" to "fms". Fix #6.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.0 (2017-03-29)
-------------------
* Increase max IO count to 16.
* Contributors: phtan

1.8.0 (2017-03-03)
------------------
* Add the CheckTime skill.
* Fix bug in Counter skill.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.7.0 (2017-02-28)
------------------
* Avoid forward_flag when switching between tracked and trackless paths.
* Make align_station_type affect also the alignment prior to home-docking.
* Add delay in set_initial_pose so amcl can update tf pose before next action.
* Add hybrid tracked path navigation.
* Add auto dock and undock for NavigateToX.
* Rename continuous_movement parameter into forward_flag.
* Add NavigateToX skill.
* Add trackless skills for forward, bezier and rotate.
* Remove separate rotate and uturn skills for rear sensor, and add rear
  sensor alignment parameter for rotate, uturn and search line skills.
* Add search line skills.
* Filter skill plugins by tracked or trackless robot.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.6.0 (2016-12-02)
------------------
* Update description for IO input and output skills.
* Upload audio skills to match new audio player with selectable playlist.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.5.0 (2016-11-28)
------------------
* Add parameter align_station_type for NavigateTo skill.
* Increse input output ports to 8.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-11-02)
------------------
* Add new skill ResetAgvPosition.
* Fix ReverseNavigateTo to use rear sensor when aligning to station direction.
* Disable sensor during charging dock in NavigateTo.
* Remove speed parameter from Rotate and Uturn skills.
* Remove failed outcome from undock charger skill.
* Remove timeout from check battery skill.
* Change audio playback and alarm playback skills to only have 1 outcomes.
* Change name for I/O skills.
* Fix bug when index of approved_destination is zero.
* Fix crash when formatting match some format specifier other than "%d".
* Add ReverseNavigateTo skill.
* Avoid set_confused when stopping in the context of waiting traffic.
* Fix bug with forward flag being reset each time _traverse_plan_ccs is called.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.3.0 (2016-10-16)
------------------
* Add skill for uturn using rear sensor alignment.
* Switch to rear sensor alignment for rotation prior to reverse action in NavigateTo skill.
* Add skills for RotateLeft and RotateRight with rear sensor alignment parameter.
* Fix incompatible change in robot base.
* Update charger dock and charger undock skills.
* Improve power management skill to suite new power management.
* Fix issue with forward wait IO and reverse wait io.
* Change io_trigger_type to use value 0-3.
* Update Reverse wait IO skill with additional parameter.
* Add skill for forward wait io.
* Add skill to recalibrate battery.
* Remove str() because it is unnecessary and it breaks unicode.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.2.0 (2016-08-25)
------------------
* Add indicator when waiting for traffic.
* Ensure traffic controller is updated even when task is preempted.
* Update NavigateTo skill to swapping-based traffic controller.
* Improve ccs_comm and task runner.
* Adapt NavigateTo skill for CCS mode.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal05 <nikfaisal05@gmail.com>

1.1.0 (2016-08-04)
------------------
* Add skill for reverse with wait io.
* Limit input read and output write to 1-2 (changed from 1-8).
* Contributors: phtan

1.0.0 (2016-07-21)
------------------
* Make NavigateTo preempt at next junction if current junction has forward_flag set.
* Fix NavigateTo skill not preempting properly.
* Add skill to play music and alarm.
* Add automatic handling of home charging in NavigateTo skill.
* Add skill IsAgvHomed.
* Add skills for manipulating registers.
* Make location initialization necessary.
* Fix suspected bug in charging status.
* Fix comparison of index zero and none.
* Fix speed not converted to meter/sec.
* Modify enable charger publisher inside robot.
* Add comparator skills.
* Add in more parameter for all navigation skills.
* Add ScanLaserProfile skill.
* Add SelectNavProfile skill.
* Add skills to write IO.
* Add minimum for counter parameter.
* Align to station heading at destination.
* Add skills for DockCharger, UndockCharger and CheckBattery
* Add skills to read IO.
* Implement skill NavigateTo.
* Add defaults for certain skills params.
* Add skill for line calibration.
* Add various wait types linked with ui popup.
* Fix and implement some skills.
* Publish to nav_control for stopping navigation.
* Add basic navigation skills.
* Add skill_test module.
* Update attribute name to skill_plugin.
* Initial commit with module stub.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

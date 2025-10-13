^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'nav-improve' into 'master'

  * Clear previous task runner stopped location upon applying new changes.
  * Add Wait Resume skill for user to trigger manual control or human follower manually.
  * Add option to start Task Runner with previous map tracker location.

* Merge branch 'location-hint'

  * Fix get_junction expect to only get x and y value.
  * Add location hint.
  * Add validator for location hint.

* Merge branch 'panel-dashboard'

  * Expose overlay path in cache.

* Add lateral direction in task runner manual control.
* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.
  * Update time-jump safety trigger message to be system error.

* Enable human_follower in manual control.
* Merge branch 'map-tracker-send-robot-radius' into 'master'

  * Fix task runner manual control delay.
  * Map tracker publish robot_radius.

* Merge branch 'hot-reload'

  * Cache models to ensure consistency after hot reload.
  * Proceed to hot reload with validated data even if there are further edits.

* Fix tracked branching map generation.
* Fix hot-reload regression bug.
* Merge branch 'task-runner-manual-control' into 'master'

  * Trigger manual control action in task runner.

* Merge branch 'lazy-state-machine'

  * Simplify state machine introspection.
  * Fix state_machine for inheriting cooked variable.
  * Construct state machine lazily.

* Merge branch 'task-runner-manual-control'

  * Handle manual control command in task runner.

* Add missing db_auto_reconnect.
* Merge branch 'hw-app'

  * Handle assets path for overlay node.
  * Add option to enable for tracked or trackless only.
  * Handle exception raise by HWApp plugin.
  * Add warning if hwapp init take longer time.
  * Connect hw app api with the hw app module.
  * Add hw-app module and load available hw app.
  * Validate plugin id to match url regex.
  * Add assets entry validation.
  * Update plugin api to obtain assets path.
  * Add load app plugin capability and save to cache.

* Optimize message conversions.
* Fix linter error.
* Fix touching polygons not intersecting sometimes.
* Merge branch 'fix-missing-variable-list' into 'master'

  * Add REST API endpoint for variables list.

* Contributors: Farhan Mustar, Patrick Chin, Tan Kui An Andrew, Wong Tze Lin

3.1.0 (2024-10-14)
------------------
* Merge branch 'fix-modbus-write-register' into 'master'

  * Mask data to ensure uint16 compatibility.

* Refactor code.
* Merge branch 'fix-rfid-skills'

  * Handle python2 and python3 compatibility for rfid rostopic callbacks.
  * Return empty string for SimRfid in trackless mode.

* Handle bytearray in python3.
* Fix skillset to be compatible between python versions.
* Remove unnecessary code.
* Merge branch 'license'

  * Read agv_uuid from agv05 database only.
  * Add agv's UUID to diagnostic.
  * Update to use new license format.

* Update rabbitmq connection to use transport_options.
* Fix producer publish for redis connection.
* Fix python3 shebang.
* Merge branch 'python-six' into svelte-webapp

  * Update python_2_unicode_compatible from six module.
  * Update publisher.
  * Update for python3 compatibility.
  * Update package.xml.
  * Update dict view.
  * Update string check.
  * Use zip from six for compatibility.
  * Update hashlib md5 and zip for python 3 compatibility.
  * Import queue using six and update type constructor to use str.

* Merge branch 'svelte-validation-msg' into svelte-webapp

  * Update validators ValidationError.
  * Update set validation error to use validation data format.

* Merge branch 'variables'

  * Update variables init in fms mode.
  * Fix missing variables in skill test, make variable behaviour similar to register.
  * Add variables list to skill test module.
  * Restart variable manager if not hot reloadable

* Merge branch 'twtc'

  * Skip sending occupation request in SelectDimensionProfile if previous task aborted.
  * Add .001 distance weight for non-stop junctions.
  * Skip sending occupation request after task aborted.

* Fix agv models init due to sim timer not running.
* Fix bug in state machine call stack publisher.
* Merge branch 'hal' into 'master'

  * Add start and stop button LED indication.
  * Change dock_charger_timeout from dynamic reconfigure to ROS parameter.
  * HAL battery state diagram.

* Block on hardware plugin error.
* Add option whether to disable laser sensor when heading into home.
* Merge branch 'invalidate-last-modified'

  * Add timeout to last modified.
  * Take first last modified only when polling.

* Merge branch 'rabbitmq-cluster'

  * Add connection fallback for RabbitMQ cluster.

* Fix popup press not working.
* Merge branch 'system-overload'

  * Update time-jump safety trigger message to include system overload.

* Merge branch 'nav-omni' into 'master'

  * Add motor calibration for mecanum drive.
  * Add manual control in lateral direction.
  * Add yaw calibration for tricycle steering drive.

* Prevent direct external access to web_video_server.
* Fix test.
* Defaults to ros_compressed stream for web_video_server.
* Merge branch 'nav-improve-p3-omni-dock' into 'master'

  * Add omni docking action.

* Merge branch 'nav-improve-p3' into 'master'

  * Add option for junction miss checking with long forward/reverse junction distance.

* Fix noetic-roslint warning.
* Merge branch 'modbus-skills'

  * Add modbus skills.

* Merge branch 'task-template-variables-api' into 'master'

  * Add GetVariable and SetVariable service

* Merge branch 'fix-false-reloadable' into 'master'

  * Fix false reloadable due to dimension modified

* Merge branch 'task-runner-set-pose' into 'master'

  * Add set pose feature in task runner

* Use wall-time as the simulation clock might not be running.
* Merge branch 'nav-improve-p3-dim'

  * Skip setting costmap parameter for sim agv.
  * Fix costmap profile not updated after syncing from DFleet.
  * Fix error in robot radius calculation.
  * Ensure proper cleanup during exception.

* Fix to allow unicode in variables.
* Fix validation not triggered if changes made during executor startup.
* Merge branch 'nav-improve-p3-dim' into 'master'

  * Move robot radius and margin to AGV dimension profiles.

* Merge branch 'nav-improve-p3-nav2d-scan' into 'master'

  * Calibrate Wheels base on nav2d_scan_topics instead of all scan topics.
  * Hardware Test - Yaw tab only shows nav2d_scan_topics instead of all scan topics.

* Fix mutable method parameters pointing to shared variable.
* Fix mutable state_machine function parameters pointing to global variables.
* Contributors: Farhan Mustar, Patrick Chin, Quan Bong, Tan Kui An Andrew, Tan Wai Liang, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'station-init'

  * Filter out headless stations from being shown for "Allowed at all stations".

* Sync global_param validator from dfleet.
* Merge branch 'hot-reload'

  * Add more models read lock for reloadable provide.
  * Add hot reloaded signal after models hot reload.
  * Remove unnecessary code.
  * Handle hot-reload check when task_runner idle.
  * Disable hot reload in fms mode.
  * Add hot-reload downloadables to prevent replacing non reloadable.
  * Handle reloadable in fms mode.
  * Handle ocg reloadable checking.
  * Update reloadable handling for standalone mode.
  * Add reloadable from fms.

* Merge branch 'add-transitiontriggers-duplicate-check'

  * Add duplicate check to TransitionTrigger validator.

* Merge branch 'traffic-communication-protocol'

  * Add rotate motion to distance cost.
  * Reduce memory usage.
  * Clear location tracking after sync changes from DFleet.
  * Obtain occupation approval when initializing task runner.
  * Add skill mutex of traffic type.
  * Validate safety margin in AGV dimension config.
  * Resend occupation request when DFleet's run id changes.
  * Include AGV current dimension profile in nav_request.
  * Handle check_transition for same spot navigation.

* Merge branch 'map-annotation-feature'

  * Rename mapannotation fields.
  * Remove map annotation plumbing.
  * Merge map annotations with map structure.
  * Handle dfleet map annotation display.
  * Add map_annotation into downloadables.
  * Handle default maps without map annotation.
  * Integrate mapx-annotation into webapp.
  * Integrate map-annotation into webapp.

* Merge branch 'update-transitiontrigger-and-teleport-form'

  * Allow inherit from variables.
  * Validate variable type.
  * Validate global params in TransitionTrigger & Teleport
  * Validate variables in TransitionTrigger & Teleport

* Merge branch 'calibration-motor'

  * Fix Motor Calibration - Rotation, Cancel button is not functioning once Done button is pressed.

* Merge branch 'secs-gem'

  * Update status to paused if paused while in between task.
  * Disable transaction polling if not enabled.
  * Update transaction_list update to have executing_transaction state and keep working status if currently executing transaction.
  * Add transaction_enabled as fms downloadables.
  * Update task runner to poll transaction list.

* Merge branch 'custom-init'

  * Refactor code due to form field changes.
  * Allow user to limit the station choices of "AGV is Parked at a Station".
  * Filter out headless stations from being shown under "AGV is Parked at a Station".
  * Indicate that initial dock to charger is for tracked mode.
  * Allow user to hide predefined initialization.
  * Remove top-level requirement for pre-init and custom-init task templates.

* Fix to allow unicode station name in task template.
* Short the validators.
* Merge branch 'no-rotate-zone'

  * Optimize validator.
  * Replace IntEnum which is not JSON-serializable.
  * Update NavigateTo skill to rotate in the correct direction.
  * Construct lgraph for each dimension profile.
  * Update check_transition to consider no-rotate zones.
  * Construct no-rotate nodes.
  * Add rotate three-quarter action.
  * Add validator for no-rotate zone.

* Merge branch 'live-task-template'

  * Handle live task template view in fms mode, set value to cache on apply downloadables.
  * Add skill_id info to skill and StateMachine and add call_stack info on action update.

* Fix false invalid branch data trigger for mapx validator and fix typo.
* Merge branch 'transition-trigger'

  * Handle change in key type during livesync.
  * Add is_enabled to transition_trigger validator.
  * Sync changes from DFleet for transition_trigger.

* Cleanup after test.
* Sync changes from DFleet.
* Merge branch 'path-forward-reverse-motion'

  * Allow trailer AGV to rotate at teleport start if teleport has non-stop transition.
  * Add ConstrainedMapXValidator to trackless live app.
  * Enforce valid heading for teleport end station.
  * Update NavigateTo skill to observe path forward or reverse motion.
  * Update skillset summary.
  * Enforce correct path forward or reverse motion when traversing home station.
  * Add tests.
  * Update validator to include path forward or reverse motion setting.

* Publish scan topic params in sim robot.
* Merge branch 'time-jump-safety-trigger'

  * Add Time Jump to manual_resume_list
  * Add STATUS_TIME_JUMP_HAPPENED to panel_control
  * Add time jump safety trigger in hardware test

* Merge branch 'map-quality-score'

  * Rename topic.

* Merge branch 'bug-fix'

  * Fix not waiting for node termination when stopping robot.
    Requires package: capabilities>=0.3.2
  * Fix bug setting dimension profile when not synced to DFleet.

* Merge branch 'tracked-branching-bug-fix'

  * Bug fix for tracked branching prefer wrong side.

* Refactor to match code in agvccs repo.
* Merge branch 'allow-trackless-home'

  * Allow home station to be placed on trackless path.
  * Prevent home station overlap with other station.

* Merge branch 'nav-improve-p2'

  * Remove next motion of non-stop for dynamic path.
  * Add triggering on track curve line action.

* Merge branch 'vscode-support'

  * Handle if DEBUGPY is not defined.
  * Update launch json using attach type.

* Merge branch 'tracked-branching-for-trackless'

  * Add tracked path branch id validation.
  * Update validator to point to junction branching error location.
  * Move tracked validation before graph construction.
  * Add missing import and unhandled cases.
  * Handle in case of empty list.
  * Separate tracked bezier paths validators.
  * Add tracked branching support.

* Merge branch 'map-quality-score'

  * Add new topic of Task Runner initialized status.

* Merge branch 'nav-improve-p2-turn'

  * Consolidate obstacle hint into safety internal message.
  * add safety_internal_message for line sensor error message
  * Update panel_control to show line sensor error messages.
  * Rename line sensor com error to line sensor error.
  * Add search line during transition from trackless to track with retry of parameter search_line_trial_max times.

* Merge branch 'nav-improve-p2-pid'

  * Allow user to manually set ahead distance for PID tuning.

* Merge branch 'tracked-and-trackless-share-parameters'

  * Store variables in agv05 database only.
  * Merge tracked and trackless dyncfg.
  * store parameters in agv05 database only

* Merge branch 'task-template-allowed-groups'

  * Backward compatible with older cache.
  * Clear allowed groups when sync from FMS.
  * Add prefetch.
  * add allowed_groups to task runner module
  * allow task-templates to be assigned to user groups

* Rename "FMS" text to "DFleet".
* Rename line_sensor_error error code enum.
* Fix typo.
* Merge branch 'robot-svg'

  * Send robot SVG in live app.
  * Add validator and publish robot SVG.

* Merge branch 'teleport-reset-position-tracker'

  * Add option to reset position or reset position tracker after teleport.

* Merge branch 'global-variable'

  * Update VariableManager for fms mode handling.
  * Rename variable field name.
  * Update validator and state_machine to allow for inheriting variable params.
  * Add state_machine variable type checking.
  * Add variable skills and handle execution error.
  * Update variable validator to validate data from editor and fix VariableManager execution error.
  * Update SkillManager to support variable type.
  * Add variable validator and hook data to VariableManager.
  * Add variable manager and hook it up to executor and update state_machine to handle variables state update.

* Merge branch 'teleport-align'

  * Add align_station_type option for teleport.

* Merge branch 'agv-task-action'

  * Add agv action status, update action in task runner and add action to diagnostic.

* Fix task polling last modified.
* Merge branch 'unicode-fix'

  * Handle unicode chars in external safety message.
  * Fix crash in diagnostic when user message contains unicode chars.

* Merge branch 'shutdown-improvement'

  * increase shutdown connection checking & reduce shutdown_success checking timeout
  * wait for shutdown_success before poweroff IPC

* Merge branch 'tune-pid'

  * Move PID tuning profile validation to agv05_executor.
  * Add PID tuning into Calibration App.
  * Add PID tuning action.
  * Add calibration failure outcome.
  * Add general error flag for calibration app.
  * Fix motor calibration is triggered even though the motion type is not allowed.
  * Fix tricycle feedback topic name.
  * Skip waiting for final stopping motor encoder value if motor calibration is cancelled.
  * Fix initial display value of motor calibration on Track AMR.
  * Allow cancel in the middle of motor calibration.
  * Add motor calibration.

* Refactor to match code in agvccs repo.
* Merge branch 'rfid-data-decoding-error'

  * Avoid decoding raw bytes from RFID topic.

* Refactor to match code in agvccs repo.
* Fix CMakeLists.txt.
* Merge branch 'realtime-pose-update-on-simulator-robot'

  * Prevent huge time gap between collision checks (for DFleet only).
  * Fix robot pose after aborting action.
  * Fix bug.
  * Reset pose when simulator robot controller stops.
  * Refactor code.
  * Simulate deceleration delay.
  * Fix bugs.
  * Publish real-time robot pose for display in UI.
  * Fix test DB setup.
  * Ignore outdated linter warnings.
  * Simulate action to run up to the actual duration needed.
  * Fixed tracked actions under trackless_sim robot.
  * Refactor code for trackless_sim robot.
  * Add tests for trackless_sim robot.
  * Refactor code for tracked_sim robot.
  * Add tests for tracked_sim robot.
  * Add path shape in graph construct for tracked map.
  * Add junction X and Y in graph construct for tracked map.

* Handle IO output topic update in hardware test.
* Contributors: Andrew Tan, Chow Yeh Loh, Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Rui En Lee, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Fix wifi test log when not in fms mode.
* Fix diagnostic error when models and cache invalid.
* Add default next_speed param for homing module.
* Merge branch 'jerk-limit' into 'master'

  * Update name and description of parameter forward_speed_limit_by_next_path_length.
  * Add next_speed and next_distance sending in goal.

* Merge branch 'external-safety-message'

  * Handle changing external safety message not updated in popup.
  * Display external safety message if available.

* Fix homing app not stopping during teleport.
* Increase wait for components to start.
* Merge branch 'rest-api-pause-agv'

  * Add subscriber for task runner command.

* Merge branch 'set-pose'

  * Allow relocalizing agv pose in manual control app.

* Fix typo.
* Merge branch 'web-video-server'

  * Run web_video_server node.

* Merge branch 'map-live-edit'

  * Force auto softreboot for live app controller to fix for patched robot and module.
  * Move default nav to params to executor module.
  * Update live app module only for trackless mode.
  * Send reflector data to map editor.
  * Add reverse navigate to and go to action.
  * Send live robot working status.
  * Rename live program to live app.
  * Rename robot to live robot.
  * Replace worker thread to spawn thread when executing.
  * Rearrange so that live app is part of controller start option.
  * Add laser data publisher in live program module.
  * Refactor and handle get agv home location.
  * Clone robot and model, add executor thread and execute navto skill using factory.
  * Update set map ocg and pose to exeuctor robot and update to load ocg from database.
  * Rename live_program module robot to trackless_agv.
  * Add reset agv pose.
  * Add live program module.

* Fix ResetAgvPositionTracker failing to switch to another map properly.
* Publish initialpose_bag topic to facilitate rosbag recording.
* Merge branch 'validation-msg-link'

  * Add link to related configuration for validation message.

* Add db_auto_reconnect.
* Merge branch 'io_rename_fix'

  * Update io name variable field.
  * fixed spacing errors from jenkins build
  * added io_rename function

* Adaptations for amcl map-quality-score. Ensure amcl receive map_layout topic before set_map call.
* Merge branch 'support-skill-plugins-in-hardware-plugin'

  * Run executor in chroot in order to load skill plugins from hardware plugin.

* Increase the delay between publishing and the actual shutdown.
* Merge branch 'refactor'

  * Add option whether to always rotate align at the destination.
  * Refactor code.
  * Remove unnecessary imports.

* Fix missing sense_line and next_motion in homing_skill_param_defaults.
* Merge branch 'odom-imu' into 'master'

  * Add Yaw tab to hardware test.

* Merge branch 'track-rotate-between-tracked-path'

  * Add tracked_heading indicator in map tracker.

* Encode unicode characters.
* Contributors: Chow Yeh Loh, Farhan Mustar, Lee Rui En, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------
* Eliminate the use of ujson library.
* Merge branch 'simulator-expansion'

  * Set default DISPLAY environment variable.
  * Use SimulatorMobileRobot semantic capability interface.
  * Rename trackless_stage robot to trackless_simulator.
  * Rename stage_bringup provider to simulator_bringup.
  * Emulate steering align input for nav in Stage simulation mode.
  * Fix ocg timestamp always zero when using sim_time.

* Force output of all nodes to screen.
* Fix bare except and run auto format.
* Contributors: Farhan Mustar, Patrick Chin

2.0.0 (2021-05-24)
------------------
* Refactor code.
* Replace newline character in diagnostic value.
* Fix tracked agv direction in diagnostic.
* Use goal junction tolerance instead of mid junction tolerance as default for dynamic waypoint. Refactor code.
* Merge branch 'obstacle-hint'

  * Add obstacle hint to safety popup.
  * Add obstacle hint to hardware test.

* Remove unused imports.
* Ensure consistent order in skill plugin package discovery.
* Merge branch 'path-planner'

  * Disable dynamic path planning unless feature flag is set.
  * Add path blocked error code.
  * Provide next_motion to teleport action.
  * Remove end line heading of linear and Bezier motions as it will be superseded by heading of target station. Fix bug in overwriting constant reference arguments.
  * Update NavigateTo action to handle dynamic path in map.
  * Publish forbidden zones.
  * Add support of headless target and reverse motion for Manual Control App.
  * Add end point heading into linear and Bezier motions. Change end point heading to positive value as negative value will be ignored in dynamic action.
  * Update GoToWaypoint and GoToStationWaypoint skills to allow setting of moving direction and goal tolerance.
  * Add trackless dynamic line and bezier nav skills.
  * Use default_tolerance as goal_tolerance for dynamic_forward.
  * Add handling for dynamic goal sent from Manual Control App.
  * Add NavigateToStationWaypoint skill.
  * Handle `plan empty` nav action feedback.
  * Add path planning skill.

* Merge branch 'ujson-update'

  * Default to allow all motions prior to syncing with FMS.
  * Fix json serialization.

* Fix bug task triggger error when empty in FMS mode.
* Fix logging.
* Merge branch 'next-motion-hint' (!35)

  * Clear next_motion when transition is less than 45deg.
  * Replace forward_flag attribute in apps and sim robot.
  * Replace forward_flag with next_motion parameter in NavigateTo skills.
  * Publish module_running flag.

* Allow suspended agv to reserve current location.
* Merge branch 'registers-arithmetic'

  * Add skills for registers arithmetic.

* Update error message in boot control.
* Merge branch 'display-current-action' into 'master'

  * Throttle action channel data.
  * Get current action from smach and send to ui

* Fix unhandled exception in hardware test in simulator mode.
* Merge branch 'merge-agv05x-to-agv05-repo'

  * Merge agv05_capabilities from agv05x repo.
  * Merge agv05x_msgs into agv05_msgs package.

* Fix bare except.
* Fix missing import.
* Fix missing full stop in validation msg.
* Remove unnecessary introspection server.
* Merge branch 'ocg-optimization'

  * Publish ocg in png format to reduce size.

* Add skill to reset only AGV position tracker.
* Contributors: Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Tang Swee Ho, Wong Tze Lin

1.14.2 (2020-10-23)
-------------------
* Merge branch 'hardware-plugin'

  * Add chroot before starting capability server.
  * Check for robot_description parameter when starting MobileRobot capability.

* Fix exec_depend.
* Add slam_toolbox as the default mapping method.
* Prevent crash on AMQP connection error.
* Merge branch 'docking'

  * Modify NavAction msg and refactor code.
  * Add marker_type.
  * Add dock and undock actions.
  * Add DockToMarker action.

* Increase timeout waiting for shutdown topic.
* Ensure traffic reservation of AGV position when AGV idle.
* Omit topic names in rosout logs for ROS Melodic.
* Improve fms task polling to ensure 1s delay after each polling.
* Show error when server endpoint has changed between http and https.
* Merge branch 'node-consolidation'

  * Provide shutdown service.
  * Update the greater mileage of the tracked and trackless mode during startup.
  * Improve hardware test.
  * Update launch files for MobileRobot provider.
  * Use active high button inputs.
  * Remove fuse status from hardware test.
  * Move topics to agv05 namespace.

* Improve amqps detection.
* Merge branch 'path-search-compute-optimization'

  * Skip copying networkx lgraph.
  * Skip freezing networkx graphs.

* Update storage url to use specific MySQL user account.
* Contributors: Patrick Chin, Tang Swee Ho

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-16)
-------------------
* Merge branch 'task-triggers' into 'master' (`!28 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/28>`_)

  * Fix abortable task will not abort after recover from low battery.
  * Fix battery recovered not trigger check new task due to task triggers.

* Merge branch 'task-triggers' into 'master' (`!27 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/27>`_)

  * Update task_runner module for task_trigger and refactor.
  * Add task_triggers validator, remove auto_homing validator and refactor.

* Fix battery level for stage robot.
* Fix position checking bug in trackless mode.
* Merge branch 'min-battery'

  * Prevent task execution when battery is below min_level.
  * Add validator for min_battery_level setting.

* Add wheel-slippage safety message.
* Fix bug when stopping dev_mode app.
* Improve diagnostic display.
* Add obstacle sensing for manual control.
* Merge branch 'oee'

  * Fix activity tracker thread not stopping on program exit.
  * Rename lock id.
  * Clear downtime activity tracker at executor start.

* Improve validation message.
* Merge branch 'task-runner-improvement'

  * Fix smach construction lock unreleased on exception.
  * Allow starting task runner in paused state. Fix `#31 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/31>`_.
  * Allow parameters for pre-init and custom-init tasks.
  * Refactor and improve error text on task runner.

* Merge branch 'oee' into 'master'
  Overall Equipment Effectiveness (OEE) (`!25 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/25>`_)
  Closes `#15 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/15>`_

  * Add downtime activity tracker.
  * Move diagnostic from FMS manager to module manager to avoid staleness.
  * Log AGV activity into database.
  * Add error codes for FMS-related errors.
  * Move error code definition to agv05_webserver package.
  * Add diagnostic for AGV status update. Fix `#15 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/15>`_.

* Add timestamp in nav_request message.
* Merge branch 'reflectors' into 'master' (`!24 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/24>`_)

  * Add reflector validator.

* Fix IO for simulator.
* Merge branch 'charging-status-api'

  * Add charging status in AGV API.

* Merge branch 'branching-improvement' into 'master' (`!23 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/23>`_)

  * Add flag to supress tracking the branch direction to avoid map tracker warning.
  * Improve map tracker to handle invalid branching direction.
  * Fix branching bug for reverse direction.

* Merge branch 'https-network' into 'master' (`!22 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/merge_requests/22>`_)

  * Enable ssl connection for communication with broker port 5671.
  * Allow request to fms without verification.

* Merge branch 'mileage-api'

  * Add mileage in AGV API.

* Merge branch 'validate-rfid-after-teleport'. Fix `#50 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/50>`_.

  * Add option to validate RFID after teleport.

* Merge branch 'forward-flag-input-for-teleport'. Fix `#48 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/48>`_.

  * Add forward flag input as teleport parameter.

* Merge branch 'preempt-fms-skill'. Fix `#30 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/-/issues/30>`_.

  * Send fms preempt request when fms_skill preempt.

* Merge branch 'error-code'

  * Add error code in AGV status API.

* Add diagnostic for FMS connection status.
* Merge branch 'map-params'

  * Sync map parameter from FMS.
  * Update skillset description.
  * Update map validator to accept parameter for path speed.
  * Add validator for map parameter.
  * Remove speed parameters from Executor.cfg.
  * Add customizable path speed levels for map editor.

* Merge branch 'allowed-motion'

  * Validate path constraints before accepting it from FMS.
  * Update traffic request to include start and end heading.
  * Sync allowed motions from FMS.
  * Implement exhaustive search for constrained path.
  * Use a newer version of networkx library from pip.
  * Check allowed motion in apps.
  * Add extra AGV executor setting (allowed_motions).

* Contributors: Farhan Mustar, Patrick Chin, Tang Swee Ho, Xin Wei Lee

1.13.16 (2020-01-12)
--------------------
* Trigger agv_update webhook event on agv status change.
* Include trackless dependencies.
* Contributors: Patrick Chin

1.13.15 (2019-11-22)
--------------------
* Merge branch 'missed-safety-feedback'.

  * Increase nav feedback queue size to 10.

* Merge branch 'fms-manager-refactor'.

  * Avoid sending older, out-of-order nav_request.
  * Refactor code to use time.time() instead of timezone.now().

* Merge branch 'simplify-robot-config'.

  * Skip validation of robot config.
  * Allow setting `auto_start` and `robot_name` in env_var.

* Fix branching bug.
* Fix bug in hint_pos equality comparison. Make hint compulsory for trackless map tracker.
* Merge branch 'agv-license' into 'master'. (`!19 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/merge_requests/19>`_)

  * Add license validation for NavWiz software

* Merge branch '56-network-is-down-after-saving-robot-configuration', close #56.

  * Fix network interface validation.
  * Add set network configuration to configurator, remove unused import and refactor.

* Merge branch 'teleport-action-versioned-param' into 'master' (`!17 <https://gitlab.com/dfautomation/product/navwiz/agv05_executive/merge_requests/17>`_)

  * Validate param range in teleport action.
  * Fix versioned param in teleport action not being validated correctly.

* Increase the number of laser profiles to 10.
* Contributors: Farhan Mustar, Patrick Chin

1.13.14 (2019-05-27)
--------------------
* Fix incorrect AGV velocity in REST API.
* Enable configurable robot type.

  * Add debug option.
  * Update launch file.
  * Skip bringup validation if capability server not available and refactor.
  * Remove preset config.
  * Fix capability server cannot start nav2d without default bringup.
  * Prevent trackless mapping from running when robot is invalid.
  * Automatically apply network when changing robot config.
  * Add extra options to network configuration.
  * Remove default provider so that it will not run if configurator provider is not available.
  * Change cache to provide robot_cfg_desc.
  * Block start-robot if robot cfg invalid.
  * Add env_var validation.
  * Add auto_start option.
  * Remove package_whitelist for capabilities server in base launcher.
  * Add bringup_provider.
  * Add validation and set_robot service.
  * Add robot configurator.

* Disable heartbeat timeout for capability server bond. Fix #54.
* Handle new motor overload feedback status.
* Add launch file for Suki 0.3.
* Modify hardware test to handle multi-port IO.
* Add port number for IO skills.
* Contributors: Farhan Mustar, Nik, Patrick Chin

1.13.13 (2019-03-24)
--------------------
* Prevent crashing when custom.log file is not writable.
* Remove network settings from `Development Mode` app.
* Fix bug cannot stop task runner when fms is disconnected. Fix #53.

  * Monkey-patch amqp library to set transfer timeout at 7s.
  * Set amqp connect timeout at 3s.
  * Switch from librabbitmq to py-amqp library.
  * Reduce fms reconnection delay.
  * Add read timeout for fms sync operation.
  * Fix task might not be sync'd to fms if task runner is stopped and fms is disconnected.

* Fix bare except statement which suppress KeyboardInterrupt.
* Fix validator.
* Prevent robot controller being started repeatedly. Fix #52.
* Ensure teleport action is preempted when task is aborted.
* Prevent execution of new task during paused state.
* Validate against task template having `Aborted` as outcome.
* Migrate to package.xml version 3 and add copyright file.
* Avoid showing exception log message on task-runner.
* Add atomic compare-and-set register action. Fix #17.
* Guarantee atomicity of all register operations.
* Contributors: Farhan, Patrick Chin, Ubuntu

1.13.12 (2019-01-21)
--------------------
* Validate against task template having `Preempted` as outcome. Fix #49.
* Optimize the retrieval of persistent registers from database.
* Skip logging wait_traffic in custom log.
* Contributors: Patrick Chin

1.13.11 (2019-01-12)
--------------------
* Fix pause not propagate to sub state.
* Add option to record task failure and safety trigger in custom log. Fix #51.
* Improve validator for raw map.
* Fix skill-test module not preempting teleport.
* Write rfid error into custom log.
* Allow executor to perform soft-reboot even with `stopasgroup` flag.
* Contributors: Farhan, Patrick Chin

1.13.10 (2018-11-30)
--------------------
* Update setup.py
* Contributors: Patrick Chin

1.13.9 (2018-11-28)
-------------------
* Add manual control function in map creator module.
* Add Qube launch file.
* Contributors: Farhan, Patrick Chin, nik3

1.13.8 (2018-11-27)
-------------------

1.13.7 (2018-10-20)
-------------------
* Fix agv mistakenly send "Sync Pending" status after reconnection to FMS.
* Publish log message to FMS through AMQP protocol.
* Add LogMessage skill. Fix #46.
* Update led blink control when task runner is paused.
* Add 2-second task runner resuming countdown.
* Allow start and stop buttons to resume and pause execution and action. Fix #45.
* Allow stop button to mute alarm (migrated from nav).
* Move dyncfg parameters into groups.
* Remove unused rotation speed parameters from NavigateTo skills.
* Combine dyncfg parameter generation to avoid separate builds for tracked and trackless.
* Track feedback irrespective whether it is linked to the current goal. Fix #44.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------
* Add option whether to clear pending tasks when task runner starts. Fix #43.
* Handle RFID in hardware test.
* Contributors: Patrick Chin, nik3

1.13.5 (2018-09-07)
-------------------
* Fix missing db_auto_reconnect in fms live_sync function since v1.13.4.
* Add is_paused method for skill and update NavigateTo action to check for
  pause state after waiting for traffic.
* Contributors: Farhan Mustar, Patrick Chin

1.13.4 (2018-08-18)
-------------------
* Implement simulated rfid.
* Emulate safety inputs for nav in Stage simulation mode.
* Fix bugs in hardware test.
* Make FMS navigation request asynchronous to prevent AGV miscount.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------
* Add titan and suki launch file.
* Convert task template mutexes from set to list for consistent md5 hash.
* Prevent default_init after fms re-sync if it has been cancelled.
* Fix exception not handled if fms connection timeout in task_runner.
* Add validator for default-app.
* Implement default app feature.
* Fix ScanLaserArea skill.
* Fix hardware test cannot display due to msb_activation array.
* Update nav topics.
* Improve clarity when displaying skill error message.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, Xin Wei Lee, farhan

1.13.2 (2018-05-18)
-------------------
* Update rfid suffix to use colon instead of dash as separator.
* Detect duplicate RFIDs across multiple maps. Fix #23.
* Update NavAction feedback types to include laser_malfunction and drive_overlimit.
* Update motor topics.
* Update power topics and messages.
* Update panel topics.
* Update safety topics and messages.
* Fix action-test module not working when FMS fails to sync.
* Change IoRead and IoWrite skills to use 0-based indexing for pins. Fix #25.
* Update io topics.
* Update led topics and messages.
* Add select_laser_profile action for trackless agv.
* Update laser sensor topics and messages.
* Update line sensor topics and messages.
* Update audio topics and messages.
* Move NavAction and NavControl definition into agv_msgs.
* Add wifi-test app.
* Add power off AGV skill.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-20)
-------------------
* Fix bug where obsolete downloadables keys are not removed from AGV.
* Add default app (reversed for future) and consolidate options into executor_cfg.
* Add app control for FMS control.
* Update fms manager to consolidate panel control and boot control.
* Enable pre-init local-sync to self-invalidate the cached downloadables. Fix #34.
* Add mutex lock when updating downloadables.
* Refactor code and move popup handling from ui to panel_control in executor.
* Include agv_uuid in get_agv_status.
* Add versioning for skill param.
* Add pause resume ability to task runner, both from ui and fms.
* Validate against pure-integer task template outcome. Fix #18.
* Allow AGV to resend FMS skill execution request until reply is received.
* Add default init in fms manager downloadables keys.
* Add default init for task runner and agv validator.
* Add pre_init key to fms manager downloadable keys.
* Add pre init validator and update task runner to handle pre init.
* Add sensor comunication error feedback.
* Fix potential race condition bug.
* Keep FMS traffic controller updated through nav request even when AGV is idle.
* Add get_module_running service.
* Add set_module_running service.
* Add custom_init to downloadable keys for fms sync.
* Add calibration app.
* Add launch file for Suzuki AGV.
* Refactor code for global params and add 'g' suffix to avoid name collision.
* Prevent teleport from using global params.
* Add global_param in downloadable key for fms sync.
* Add global parameter validator.
* Enforce maximum value for teleport distance cost.
* Update task template validator to check for unreachable action and outcome.
* Implement FMS task runner (in AGV mixed mode), i.e FMS skills.
* Refactor to store skillset in cache instead of ros topic to match FMS implementation.
* Refactor to access persistent register through django orm to match FMS implementation.
* Add RFID support in validator and get_location method.
* Add branching support in validator, map tracker and robot base.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, ammar95, farhan, kxlee, nikfaisal

1.12.4 (2018-03-12)
-------------------
* Send cancellation of forward_flag when skill is preempted. Also fix #24.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.3 (2018-02-27)
-------------------
* Add safety popup message for wait_traffic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.2 (2018-02-23)
-------------------
* Improve reset function for RFID.
* Fix missing db_auto_reconnect decorator.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.12.1 (2018-01-03)
-------------------
* Fix rfid bug.
* Reset RFID topic instead of internal flag.
* Add launcher for Stage simulation robot.
* Add SelectFrontLaserProfile skill.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>

1.12.0 (2017-12-03)
-------------------
* Publish robot motion to live map UI.
* Prevent error message when retrieving robot pose before controller starts.
* Add persistent registers.
* Add `create_suspended` flag in task template.
* Group various task triggers into change_task service.
* Sync to fms when removing all tasks.
* Add task listing from FMS and add secondary update of unsynced tasks to FMS.
* Improve mutex lock in task runner to fix race condition when aborting task.
* Enable `abort_task` operation from FMS.
* Add `aborting` task status.
* Enable `create_task` operation from agv in FMS mode.
* Add `non-stop-transition` feature in teleport.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.11.7 (2017-11-16)
-------------------
* Fix bugs.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.6 (2017-11-06)
-------------------
* Include safety and user message in fms update data.
* Add `prev_location`, `motion`, and `velocity` to AGV status API.
* Fix #3: Button press not detected during user message.
* Remove resume button and fix bug motor wont run during hardware test by auto=enabling motor at start.
* Fix homing app functionality in FMS mode.
* Add `Running App` agv status.
* Create get_agv_status service for API usage.
* Handle new task status enum in _remove_all_tasks.
* Add task progress field and add UpdateTaskProgress skill.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.11.5 (2017-10-24)
-------------------
* Fix executor not reconnecting to rabbitmq by using agv_feedback as heartbeat.
* Add in branching skills.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.11.4 (2017-10-19)
-------------------
* Display parameters for running tasks.
* Fix bug auto_homing not being triggered.
* Reserve initial position with FMS traffic controller when starting task runner.
* Flip the front MSB sensor left to right in hardware test.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.11.3 (2017-10-02)
-------------------
* Add delay after amcl's set_map service call.
* Make map_tracker_x more robust to custom trackless navigation commands.
* Display activation instead of MSB raw value in hardware test.
* Fix sync bug when content has unicode character.
* Prevent crashes when downloadables has errors.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.11.2 (2017-09-26)
-------------------
* Update ocg downloadables format.
* Increase fms update frequency for better real-time live map view.
* Add trackless agv pose in fms update message.
* Fix bugs.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.1 (2017-09-13)
-------------------
* Add map name to cached data.
* Publish executor status.
* Add get_register and set_register services.
* Clear local and global registers when stopping task and task runner respectively.
* Read model version from package.xml.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.0 (2017-08-22)
-------------------
* Add RFID skills.
* Modify task_counter behavior and fix race condition bug.
* Enable live-sync without stopping task runner app.
* Add parameter variable_storage_url.
* Convert cached task templates from OrderedDict to list.
  Because json doesn't preserve order and doesn't honor integer-based keys.
* Auto-apply downloadables when module is starting, stopping or not running.
* Allow most modules to start without prior syncing with fms.
* Setup periodic live-sync to update on agv_ip.
* Add live-endpoint and download sync.
* Add fms channel pipes.
* Publish current executor info through Django cache instead of ROS topic.
* Split portion of FmsComm into FmsManager class.
* Allow agv's rabbitmq client to declare its own queues.
* Implement skillset summary which encompass all capabilities of the agv.
* Drop pika in favor of kombu.
* Update fms validator.
* Disable certain validators in FMS mode.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.10.6 (2017-07-25)
-------------------
* Add custom initialization task template to task runner.
* Update skill to support new remote io node.
* Contributors: nikfaisal, phtan

1.10.5 (2017-07-20)
-------------------
* Fix deadlock in acquiring mutex when stopping some apps.
* Extend duration of sim robot's action for demo purpose.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.4 (2017-07-12)
-------------------
* Use amcl's set_map service for a more deterministic update time.
* Make trackless_sim robot publish its transform.
* Validate mutexes in teleport action and pre-action.
* Implement teleport action and pre-action execution.
* Fix possible concurrency bug.
* Update multi-map junction indexing.
* Add validation for recursive call in nested task templates. Fix #2.
* Improve validators.
* Add resource mutexes to skill meta.
* Add MODELS_VERSION to ease cache invalidation.
* Switch to ujson for performance.
* Fix task status not set to "Aborted" after exception.
* Fix hardware test to clear motor_enable flag after terminate.
* Allow reset to a station to start task runner.
* Add validation for multi map and teleport.
* Load tracked agv's map from active_map.
* Add Io read/write register skills.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>, phtan

1.10.3 (2017-05-18)
-------------------
* Prevent executing start on robot that is already started, and on a running module.
* Publish tracked robot location.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.2 (2017-05-08)
-------------------
* Add development mode app.
* Add skills for remote io read and write.
* Add missing files for installation.
* Fix #14: action client's goal being removed wrongly.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.10.1 (2017-04-14)
-------------------
* Invert left and right turning area scan in hardware test.
* Fix bug in charging status of hardware test.
* Add led blink control, expansion io, error led in hardware test.
* Refactor code to change reference of "ccs" to "fms". Fix #6.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.10.0 (2017-03-29)
-------------------
* Increase max IO count to 16.
* Add Fuse status check in hardware test.
* Add left and right speed control in hardware test.
* Clear motor relay and charger relay once hardware test is stopped.
* Contributors: Ikhwannudin503939, phtan

1.8.0 (2017-03-03)
------------------

1.7.0 (2017-02-28)
------------------
* Modify "stop module" to be forcefully terminate.
* Stop music, alarm and led on abort task.
* Create resume button to clear safety flag in hardware test.
* Fix missing skill param in homing_x module.
* Always trigger charging when task runner starts.
* Add map layout publisher on all robots.
* Implement select_nav_profile for trackless robot.
* Improve map tracker to handle divergent paths with the same start heading.
* Publish map layout data.
* Add delay in set_initial_pose so amcl can update tf pose before next action.
* Add hybrid tracked path navigation.
* Add validation for hybrid tracked path.
* Add NavigateToX skill.
* Add log message for trackless_sim.
* Patch map_tracker methods onto robot's models.
* Partially implement map tracker (trackless).
* Implement robot's set_initial_pose.
* Add additional start option specifically for running map creator.
* Extends robot startup wait tolerance.
* Add trackless skills for forward, bezier and rotate.
* Run amcl and publish map when TaskRunner and SkillTest module starts.
* Add validators for trackless.
* Update charger and motor voltage control to publish to power management node in hardware test.
* Fix bug in task template validator.
* Add task runner (trackless) stub.
* Move map broadcast to map channel in UI to improve performance and code reuse.
* Refactor code by separating out map tracker, validator, and construct_graph from models.py.
* Remove separate rotate and uturn skills for rear sensor, and add rear
  sensor alignment parameter for rotate, uturn and search line skills.
* Add search line skills.
* Add other tracked base's methods to trackless base.
* Add reset_odom.
* Throttle map saving.
* Implement start, stop and saving of map to database.
* Fix color palette of map.
* Trigger gencfg rebuild if TRACKLESS flag changes.
* Allow changing mapping_method using dyncfg.
* Implement map creator module (display only).
* Filter skill plugins by tracked or trackless robot.
* Import only relevant module files for tracked or trackless robot.
* Add map creator stub.
* Add override of SimpleActionClient methods in TracklessAgv05Base.
* Set envvar "TRACKLESS" in launch file.
* Enable executor to launch trackless_agv05 robot.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.6.0 (2016-12-02)
------------------
* Make executor wait for nav action server when starting robot.
* Update hardware test so that it subscribe to topic only when it is started.
* Disable sensor when perform initial docking via task runner.
* Fix missing charging_status at initial startup.
* Fix missing param in homing skill.
* Upload audio skills to match new audio player with selectable playlist.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-28)
------------------
* Add Relay control in hardware test.
* Add out of line and junction detected flag in hardware test.
* Improve motor test: to increase speed on button hold and decrease speed until zero on release.
* Add motor, panel and obstacle laser test.
* Add bumper and emergency button test.
* Add Hardware Test module.
* Increse input output ports to 8.
* Rename MySQL database from 'test' into 'agv05'.
* Add missing functions in TrackedSim robot.
* Split tracked and trackless robot files to separate dependencies.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-11-02)
------------------
* Avoid set_confused when stopping in the context of waiting traffic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.3.0 (2016-10-16)
------------------
* Prevent executor from crashing if models are invalid.
* Prevent a skill test from starting when another is in progress.
* Fix incompatible change in robot base.
* Fix issue with forward wait IO and reverse wait io.
* Update Reverse wait IO skill with additional parameter.
* Add skill for forward wait io.
* Make agv05_executor a required process.
* Add db_auto_reconnect to the background app. Fix #1.
* Fix owner field in task list.
* Switch ValidateModel action server to service.
* Add allowed_users field to TaskTemplate cache.
* Persist task to database.
* Add skill to recalibrate battery.
* Remove str() because it is unnecessary and it breaks unicode.
* Fix register_list ordering.
* Clear previous popup at start.
* Modify stop robot behaviour to force stop the running module directly.
* Improve loop hole in task template validation message.
* Fix call to set_initial_state which should accept list of string.
* Fix missing items in tracked_sim robot including audio class and wait_traffic function.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.2.0 (2016-08-25)
------------------
* Add indicator when waiting for traffic.
* Add auto-homing.
* Make AGV reinitialize its location every time task runner is started.
* Update NavigateTo skill to swapping-based traffic controller.
* Update tracked_sim.
* Fix bug in Agv05Power.
* Improve CcsComm and task runner.
* Adapt NavigateTo skill for CCS mode.
* Publish AGV idle or working status to CCS.
* Add communication between task runner and CCS for task status and task assignment.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, nikfaisal05 <nikfaisal05@gmail.com>

1.1.0 (2016-08-04)
------------------
* Add skill for reverse with wait io.
* Contributors: phtan

1.0.0 (2016-07-21)
------------------
* Add publish and subscribe to CcsComm.
* Fix bug in validator.
* Add ccs_comm stub.
* Remove restrictions that requires delayed import of models.
* Add global param agv_home for task template.
* Trim down cache objects and add downloadables_md5.
* Update the cached objects.
* Add agv_name to models.
* Add logging messages.
* Fix bug: Executor should set the dynamic storage params, because webserver will start up later.
* Add missing lock.
* Auto clear previous task list when task runner restarts.
* Add skill to play music and alarm
* Add free motor function.
* Add automatic handling of home charging in NavigateTo skill.
* Add validation for agv home direction.
* Add logging messages.
* Add homing module.
* Add logging for errors.
* Make AGV location-unaware initially.
* Make location initialization necessary.
* Modify enable charger publisher inside robot
* Add start and stop management for each module.
* Make robot reservation necessary for starting a module.
* Fix bug in skill test.
* Fix passing register as inherited params.
* Improve validation message.
* Allow nesting task template as action.
* Fix issue with duration in the waiting for result
* Add ScanLaserProfile skill.
* Add SelectNavProfile skill.
* Add auto_start argument for executor launch file.
* To steal Agv05Panel for Sim robot.
* Fix bug in task_runner that causes 100% cpu cycle.
* Add navigation stub for tracked_sim robot.
* Fix bug in construct_graph.
* Add linux wifi manager into launch file.
* Implement skill NavigateTo.
* Set minimums for dyncfg params.
* Add validation for map and task template data.
* Add skill for line calibration.
* Add various wait types linked with ui popup.
* Fix unicode issue.
* Initialize registry in task runner.
* Fix task_runner bug.
* Fix parameter mismatch.
* Publish to nav_control for stopping navigation.
* Fix robot unable to stop issue.
* Add task_runner module.
* Add skill_test module.
* Add models validator stub.
* Add publishing to register_list topic.
* Change keyword checking of skill parameter.
* Add dynamic reconfigure server.
* Add register manager.
* Make robot base inherit from ActionClientMixin.
* Attach skill manager to robot.
* Fix some bugs when starting and stopping robot controller.
* Remove autostart of robot controller.
* Stricten skill plugin check.
* Update package metadata.
* Update attribute name to skill_plugin.
* Initial commit with module stub.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

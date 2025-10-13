^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_webapp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'update-marker-type-configuration'

  * Fix marker type configuration UI not fully shown on fullscreen.
  * Update marker configuration UI.

* Merge branch 'nav-improve'

  * Add option to start Task Runner with previous map tracker location.

* Merge branch 'location-hint'

  * Add location hint zone for tracked editor.
  * Display zone name on map.
  * Add location hint zone editor on map layout editor.

* Merge branch 'ui-improvement'

  * Update TopBar to hide more info for mobile view.
  * Update app style for mobile screen.
  * Make hardware test accessible on mobile phone.
  * Add permission to view live monitor tools in task runner.
  * Remove invalid task history button and change task history drawer direction.
  * Make boot screen viewable on phone.
  * Fix not found display for various page.

* Merge branch 'component-tracking'

  * Allow data upload and sync from DF Hub.
  * Add AMR part and service-log pages.
  * Improve version display.

* Merge branch 'panel-dashboard'

  * Add task runner as default dashboard if no dashboard defined and fix bottom bar z index.
  * Show error message in bottom bar if fms connection broken.
  * Fix keyboard spacer background.
  * Rearrange open task runner callback.
  * Add open task runner and task history drawer api.
  * Add status channel to dashboard api.
  * Add task runner channel to dashboardAPI.
  * Fix camera-stream custom-elements bug.
  * Add keyboard spacer in dashboard.
  * Fix custom element for dev server.
  * Add dashboard template and dev server.
  * Fix hwapp dev server.
  * Add CameraStream custom elements.
  * Make BottomBar as custom element.
  * Rearrange dashboard drawer file.
  * Move task outcome toast to dashboard.
  * Link task runner display status to bottom bar.
  * Add bottom bar design.
  * Shrink side rail for short display.
  * Add dashboard page and rearrange layout.
  * Fix topbar height while loading.
  * Add log channel for app sockserver.

* Add lateral direction in task runner manual control.
* Show refresh button on AGV panel only.
* Enable human follower in manual control app.
* Merge branch 'angular-speed-based-on-robot-radius' into 'master'

  * Update manual-control min speed to 0.1.
  * Manual control angular speed based on robot radius.

* Fix camera stream not stop when navigate away.
* Fix custom log layout.
* Remove unnecessary depth information from the status page.
* Merge branch 'system-log-downloader'

  * Update log display theme.
  * Add log list view using new log api.
  * Improve diagnostic log api and view.
  * Add progress update when bundling log.
  * Monitor file status using PUT request, and download when ready.
  * Update frontend to poll for download log task.
  * Remove fetch parameter that does not exist in downloadFileAPI.

* Merge branch 'camera'

  * Synchronize depth and color images.
  * Rotate image based on camera orientation.
  * Add multiple depth cloud coloring modes.
  * Fix canvas sizing bug.

* Skip unnecessary update and fix html escape.
* Fix missing title info.
* Prefers hot reload.
* Change svelte dev server port to 8080.
* Fix missing title for network scanner and update user panel title display.
* Run auto format.
* Merge branch 'add-missing-protected-parameter-option'

  * Fix tree view to not override checkbox style.
  * Minor update to disabled checkbox opacity.
  * Add protected parameter option in restore feature.

* Merge branch 'fix-uncleared-background-interval'

  * Refactor code.
  * Handle race condition due to joystick data change.
  * Fix uncleared interval for Manual Control.

* Fix camera regression error.
* Add missing table style.
* Merge branch 'task-runner-manual-control'

  * Add manual control in task runner.
  * Improve styles.

* Display only pending tasks which are runnable by the current AGV in FMS mode.
* Merge branch 'fix-live-app-error'

  * Fix error calling getContext outside component initialization.

* Merge branch 'hw-app'

  * Update hwapp dev port.
  * Add install statement for hwapp_build directory.
  * Add more api function.
  * Replace custom element api with init function api to fix cannot redefine custom element.
  * Simplify dev server api response to make it easier to customize.
  * Add redefine-custom-elements package to allow hot reload custom elements.
  * Display hw app name similar to navwiz display.
  * Add websocket api for template file.
  * Add hw_app_dev node.
  * Init keyboard controller.
  * Rearrange file and add AppLayout.
  * Add initial project to build hwapp template for dev server use.
  * Attach hw app api to the dom attribute when load.
  * Handle shutdown and update app id for session tracking.
  * Add hw app into app list and add base hw app page.svelte.

* Fix to close softRebootSpinner when navigate away.
* Merge branch 'lidar-display'

  * Change traces using path element.

* Add action swap feature.
* Fix search for stations of disabled maps.
* Fix html escape in task template manual layout.
* Add convenience links.
* Handle invalid params value in task-completed page.
* Merge branch 'laser-downsample'

  * Add laserDownSample option and enable for agv panel user.

* Merge branch 'fix-missing-initial-status'

  * Add tip title for Go To button.
  * Fix missing initial status.

* Fix area got accumulated.
* Fix hover error.
* ParseInt to ensure data type compatible.
* Fix agv activity update navigation.
* Fix cannot edit first agv activity on the list.

* Contributors: Farhan Mustar, Patrick Chin, Tan Kui An Andrew, Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Update to sveltekit webapp.

  * Update login page.
  * Simplify npm_install script.
  * Improve dynamic dropdown UI.
  * Hide omni svg option.
  * Update system monitor to be able to refresh when click the same button again.
  * Fix bug when using pathname, sometimes it is suffix with slash.
  * Invalidate all when unauthorized to refresh user store data.
  * Use elevation for unauthorized startModule and stopModule requests
  * Revoke ObjectURL after use.
  * Add usb update ui form.
  * Add software-update page.
  * Add loading page if blank screen more than 3 seconds.
  * Add diagnostic view to completed task view.
  * Update diagnostic-view.
  * Make download api failed msg more robust.
  * Fix @hpcc-js/wasm version.
  * Change navwiz logo to svg format.
  * Add spinner when restoring file.
  * Use API to get robotname instead of store.
  * Add spinner when uploading restore file.
  * Ensure start menu closed when robot state change.
  * Add robot name to title and init robot name with empty string.
  * Add spacing for static ip checkbox setting.
  * Mute warning and run autoformat.
  * Add soft reboot and hot reload in validation message.
  * Increase on-screen keyboard size.
  * Add disconnected wifi icon in notification.
  * Prevent npm run build to remove the build folder for overlay.
  * Enable on-screen keyboard for marker editor in skill test app.
  * Add marker type ui to skill test app.
  * Add marker type configuration UI
  * Hide validation message on dashboard page.
  * Hide map quality for tracked mode.
  * Add Omni svg.
  * Remove virtual-keypad property.
  * Add task template param tab and update tab theme.
  * Add navwiz theme based on previous color.
  * Sanitize agv init data.
  * Add elevated request and handle callback.
  * Allow toast on top of modal and keyboard.
  * Validate form before start skill test.
  * Add on-screen keyboard for calibration module input.
  * Add variable in skill test app.
  * Allow password visibility toggle without losing focus for on screen keyboard.
  * Add ip keyboard.
  * Add global onscreen keyboard.
  * Enable svelte inspector plugin.
  * Add transaction panel for task runner.
  * Switch modal notification to toast and change toast position to prevent overlapping action buttons.
  * Add confirmation to stop agv controller.
  * Add downtime tracker service.
  * Add priorityTrigger for modalStore to fix queue consideration and handle elevation error indication internally.
  * Rearrange elevated api call to allow for username password elevation.
  * Update to obtain csrf_cookie_name dynamically.
  * Add license edit page and update info for help page.
  * Configure webapp install directory.
  * Change fallback to use index.html.
  * Add pin input modal for agv panel and add http interceptor for using pin protected permission.
  * Add new endpoint for agv panel login using token.
  * Add chrome extension to load token to browser.
  * Add DisconnectedIndicator and error page for user config.
  * Prevent ModalLoadingSpinner from being closed by the user.
  * Add system log downloader.
  * Handle map and task templates form error.
  * Handle form error for users and groups.
  * Enable button-has-type rule, add type attribute to all button and run autoformat.
  * Add page title tag.
  * Add view my auth token.
  * Add change my password.
  * Add protected parameter feature to parameter editor and endpoint.
  * Add warning for parameter endpoint issue.
  * Add more LicenseVoidMixin to api endpoints.
  * Add backend offline handling
  * Add basic form validity check
  * Add License Void Mixin
  * Add FmsVoidMixin and handling
  * Replace pako with fflate.
  * Add Map Quality Score page
  * Make agv activities report reactive.
  * Make task completed page reactive.
  * Add bar charts for daily task count and mileage
  * Add bar charts to Agv Activity Report
  * Add bar charts for Task Completed Report
  * Add pie charts for Agv Activities
  * Add pie charts for Task Completed Report
  * Make Pie Chart tooltip show value instead of ratio
  * Add download diagnostics feature for task completed
  * Add download feature for monthly task completed
  * Add download feature for monthly agv activity
  * Add Agv Activities download view
  * Add Task Completed download view
  * Add task completed list
  * Add agv-activities list and edit feature
  * Add csv download.
  * Add agv activity configuration.
  * Add agv and task report.
  * Disable darkmode and merge dark css using tailwindcss dark: prefix.
  * Add custom style for DatePicker.
  * Add toast info for dfleet active status on submit agv config.
  * Add agv config page
  * Add diagnostics page
  * Delay channel disconnect to avoid socket disconnect on page navigation.
  * Add available filter options in task templates endpoint and add filter widget to task templates page.
  * Add task templates mass delete page.
  * Add task templates mass users update page.
  * Add mass category update page and show more info in task templates mass udpate.
  * Add mass update page and fix enpoint to only update provided data.
  * Add tooltip for users and groups page.
  * Add tooltip offset config and fix label tooltip offset.
  * Update tooltip for task template and laser sensors, make default tooltip placement to bottom.
  * Add maps editor tooltip.
  * Add tooltip for map annotation.
  * Add tooltip for map layout.
  * Change tooltip attributes name to prefix with tip-.
  * Handle tooltip for keyboard tab event.
  * Add tooltip feature.
  * Add date-time page
  * Add Custom Log page.
  * Add icon for each search result item type.
  * Add description and url to search results.
  * Make clicking SideRail refresh the page.
  * Add system monitor page
  * Add advanced restore feature
  * Add restore feature
  * Add backup and advanced backup feature
  * Add group config page
  * Add validation data channel and display validation msg.
  * Update config layout and config dashboard to use white_label copyright_label.
  * Add white-label form view.
  * Add software patch page.
  * Add hardware plugin config page.
  * Update api and link to ui for permission update and reset.
  * Add permissions view.
  * Add assembly info and preventive maintenance page.
  * Add IO configuration page.
  * Add laser-sensors editor page.
  * Add parameter reset and remove option.
  * Add ClearableFileField on audio form.
  * Add audio form page.
  * Add set pose modal.
  * Add manual control modal.
  * Add map creator and live app.
  * Add tracked map live view.
  * Add mapx live view.
  * Add floating submit button for parameter page.
  * Add submit parameter form.
  * Add list and edit form view for parameter page.
  * Add monkey patch for d3
  * Add user config page.
  * Add camera module page.
  * Add keyboard to wifi config modals.
  * Add numeric keypad handling.
  * Sort wifi list based on signal strength.
  * Add svelte-keyboard for task param and action test.
  * Add ip and netmask validators.
  * Add Wifi config modals and services.
  * Add Action Live View modal.
  * Handle dark mode display and tidy up styles.
  * Handle user panel redirect.
  * Add spinner to start and stop robot operation.
  * Add diagnostics page.
  * Add help page.
  * Add Wifi Test modal.
  * Add ManualLineFollow modal.
  * Add Homing modal.
  * Add Hardware Test page.
  * Move calibration page to modal.
  * Add dev-mode page and move action-test to modal.
  * Add calibration page.
  * Add Action Test page.
  * Get notification from popup service.
  * Add remote IO live view.
  * Add IO live view.
  * Relocate rest api services for agv05Boot and agv05Module.
  * Add task history page.
  * Handle task queue and task params.
  * Add Task Runner UI.
  * Add ModuleChannel.
  * Add variable editor and set prettier to use bracketSameLine rule.
  * Update /config/webhooks/add and /config/webhooks/[id=integer].
  * Update /config/webhooks.
  * Add upload/download file handling of robot.json.
  * Update /config/robot-config.
  * Add map ocg editor.
  * Add ocg list view and update ocg endpoint.
  * Add ocg selection in mapx layout editor.
  * Add mapx annotation editor.
  * Rearrange map changeset endpoint, update mapx changeset view and update layer panel.
  * Add ocg endpoint.
  * Add env checking, add mapx layout editor and rearrange code.
  * Add map active editor tabs.
  * Add map param editor.
  * Add map transition trigger editor and add multiple option in Select component.
  * Add map teleport editor view.
  * Add active map editor.
  * Update map changeset-view and add backend api for it.
  * Add map edit properties page and add tabs.
  * Add map annotation editor.
  * Add checking for updated while editing map layout.
  * Add map layout editor.
  * Add map list view.
  * Use skeleton modal store to display /config/network pop-up boxes.
  * Add trackless env variable inside auth endpoint.
  * Update /config/network/eap.
  * Update /config/network/https.
  * Update /config/network/hostname.
  * Update /config/network/identify.
  * Update /config/network/configuration.
  * Update /config/network.
  * Add /config/network layout.
  * Add global param editor.
  * Add task template users edit page.
  * Add copy and delete task template ability.
  * Add last-cached and last-saved task template view.
  * Add checking for unsaved change on navigation away form task template editor.
  * Add task template editor.
  * Add task-templates metadata endpoint for task templates editor.
  * Add agv05 boot service.
  * Add ModalLoadingSpinner.
  * Update top bar and add notification panel.
  * Add tailwindcss forms plugin.
  * Add prettier plugin for tailwindcss.
  * Add task templates list view and api.
  * Add search api, refactor fetch api and linkup search api to search modal.
  * Add search modal.
  * Add agv activity stats builder, add billboard.js and plot pie chart for agv activity card.
  * Add celery task for mileage stats and plot the data on mileagecard.
  * Add battery stats celery task and update battery card to display the data.
  * Add d3js, add graph components and display the hourly task graph.
  * Add today's task statistic.
  * Add server-clock store.
  * Add skillset image.
  * Add dashboard data.
  * Add help page.
  * Add white-label store and update page to use it.
  * Redirect to login if not login.
  * Add Perm components to check for permissions.
  * Add sockjs with interface and example.
  * Add config-panel layout.
  * Add login interface.
  * Configure to work with django backends.
  * Update build directory and dev server port.
  * Replace agv05_webapp with sveltekit project.

* Merge branch 'license' into svelte-webapp

  * Update license page.
  * Use machine ID as AGV UUID.
  * Update LicenseVoidMixin.
  * Add license page.

* Rename transition label.
* Merge branch 'hal' into svelte-webapp

  * Support 5 laser topics and 6 lidars for docking target display.

* Merge branch 'nav-omni' into svelte-webapp

  * Add motor calibration for mecanum drive.
  * Add omni drive flag into environment variable and allowed motion.
  * Add manual control in lateral direction.

* Add yaw calibration for tricycle steering drive.
* Merge branch 'map-10mm' into svelte-webapp

  * Force consistent rounding error at 1cm resolution.
  * Revert "Fix cursor coordinate is not snap to the grid".
  * Fix draggables snap factor.

* Merge branch 'task-runner-set-pose' into 'master'

  * Add set pose feature in task runner

* Merge branch 'nav-improve-p3-nav2d-scan' into 'master'

  * Hardware Test - Yaw tab only shows nav2d_scan_topics instead of all scan topics.

* Contributors: Farhan Mustar, Patrick Chin, Tan Kui An Andrew, Tan Wai Liang, Wong Tze Lin

3.0.0 (2023-08-02)
------------------
* Fix npm install issue.
* Merge branch 'minor-ui-fix'

  * Fix manual control right button UI.

* Merge branch 'update-df-logo'

  * Center the copyright label.
  * Update df logo.

* Merge branch 'map-annotation-feature'

  * Modify live map for new cache format.
  * Modify style of polygon annotation.
  * Add hud to control visibility of map annotation.
  * Integrate map-annotation into webapp.
  * Soft link map-annotation to webapp shared.

* Merge branch 'secs-gem'

  * Add io and live task template button in task transaction-runner view.
  * Add resume transaction option on transaction panel.
  * Use display name for port in transaction panel.
  * Add permission for transaction abort and cancel.
  * Add abort and cancel option.
  * Update to receive executingTransaction flag from server.
  * Update transaction list display and add current action display.
  * Handle transactions list comming from server and add initial transaction mode layout.

* Merge branch 'custom-init'

  * Add loading spinner in webapp.
  * Filter out headless stations from being shown under "AGV is Parked at a Station".
  * Indicate that initial dock to charger is for tracked mode.
  * Allow user to hide predefined initialization.
  * Remove top-level requirement for pre-init and custom-init task templates.

* Hide gateway timeout error when starting robot controller.
* Merge branch 'manual-layout'

  * Add manual layout to task template editor.

* Merge branch 'live-task-template'

  * Update theme for fms skill in live view.
  * Update button icon.
  * Add call_stack navigation.
  * Hide viz when no action.
  * Add task template name header.
  * Update live data on action update and add models api to render.
  * Subscribe to action and link modelsUpdated event.
  * Add task template live view and obtain templates data from task api.
  * Add task template panel in task runner view.

* Merge branch 'remote-io-ui'

  * Update remote io monitor button icon.
  * Add RemoteIoChannel and remote IO live view UI.

* Merge branch 'time-jump-safety-trigger'

  * Add time jump in hardware test display.

* Merge branch 'hssid_eap'

  * Fix HiddenSSID selection popup bug on touch screen.
  * Get identity from user input.
  * Add scan_ssid.
  * Hide hidden ssid in wifi list.
  * UI improvement for hidden ssid and refactoring.
  * Support for hiddenSSID and WPA2-EAP.

* Merge branch 'no-rotate-zone'

  * Add no-rotate zone for tracked agv.
  * Add no-rotate zone to live map.

* Update UI text.
* Disable auto open browser in gulp.
* Merge branch 'time-synchronization'

  * Refresh AGV panel time display using its own system time.

* Merge branch 'task-templates-allowed-groups'

  * Consider allowed_groups in home controller.

* Fix watchify.
* Merge branch 'nav-improve-p2'

  * Allow user to manually set ahead distance for PID tuning.
  * Allow cancel in the middle of line sensor calibration.

* Rename "FMS" text to "DFleet".
* Merge branch 'robot-svg'

  * Update robot SVG in live map.

* Merge branch 'global-variable'

  * Add handling for passing variable as top level params.

* Merge branch 'ui-glitch-fix'

  * Refactor to avoid reusing variable.
  * Remove views transition animation.
  * Fix selection popup and virtual keypad not auto-dismissed when app modal closed.
  * Fix app modal dismissed when clicked after dismissing a popup.
  * Remove animation for app modal.

* Merge branch 'io-monitor-popup-dialog'

  * Move IO panel view into task-runner app.
  * Fix linting and unittest.
  * Added IoChannel & IO monitor popup.

* Merge branch 'notification-ui'

  * Remove volume icon from UserPanel.
  * Shorten date display on UserPanel.

* Merge branch 'tune-pid'

  * Move PID tuning profile validation to agv05_executor.
  * Add PID tuning into Calibration App.
  * Run js-beautify.
  * Add hints to motor calibration sequences.
  * Allow cancel in the middle of motor calibration.
  * Add motor calibration.

* Merge branch 'realtime-pose-update-on-simulator-robot'

  * Display robot pose from topic for tracked_sim robot.

* Contributors: Andrew Tan, Chow Yeh Loh, Farhan Mustar, Lee Rui En, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Rearrange html.
* Fix typo.
* Merge branch 'set-pose'

  * Disable set-pose when map is unavailable.
  * Add controls to relocalize agv pose in manual control app.

* Rearrange code.
* Merge branch 'web-video-server'

  * Add permission object.
  * Display actual pixel size of camera if possible with border padding.
  * Add live camera streaming into AGV panel.

* Merge branch 'map-live-edit'

  * Display reflectors in live app panel and editor.
  * Add map quality score display on live app.
  * Rename live program to live app.
  * Fix missing particle cloud in live program mode.
  * Rearrange so that live app is part of controller start option.
  * Add live-map on live program app and add ocg interface to set map from client.
  * Add live-program channel module and add live app connector on layout editor.

* Manual Control minspeed changed to 0.1.
* Merge branch 'io_rename_fix'

  * Update io form, update io hardware display and add data to backup restore.
  * Add a function to rename the IO name used in hardware_test.

* Merge branch 'map-multi-select'

  * Add tracked multi select feature.
  * Update webapp activeObject to match webserver.

* Add visualization for laser intensity threshold.
* Merge branch 'odom-imu' into 'master'

  * Add Yaw tab to hardware test.

* Merge branch 'sockjs-fix'

  * Drop support for other SockJS transports except Websocket for performance and reliability reasons.

* Merge branch 'virtual-joystick'

  * Invert angularSpeed on reverse.
  * Link the manual control command for joystick data.
  * Set joystick as default controller and rearrange layout.
  * Add joystick onchage, allow rotate and fix styling.
  * Replace with virtual joystick directive.
  * Added virtual joystick in map creation section.
  * Add the virtual joystick.

* Contributors: Farhan Mustar, Lee Rui En, Ng Yi Liang, Patrick Chin, Wong Tze Lin, YC Lee

2.1.0 (2021-11-24)
------------------
* Optimize UI update in webapp.
* Separate normal and locked task templates in task runner.
* Add a button on live map to trigger auto focus on the agv.
* Fixed position of negative sign inside keypad
* Merge branch 'path-planner'

  * Update hardware test to cater for dynamic navigation laser area.

* Contributors: Farhan Mustar, Ng Yi Liang, Patrick Chin, Justin Teo

2.0.0 (2021-05-25)
------------------
* Refactor code.
* Fix build error resolving babelify's preset module.
* Merge branch 'obstacle-hint'

  * Add obstacle hint data to hardware test.

* Run js-beautify. Replace duplicated file with symlink.
* Handle wifi signal level which comes in percentage format.
* Merge branch 'path-planner'

  * Disable dynamic path planning unless feature flag is set.
  * Add forbidden zone in trackless map.
  * Add dynamic reverse motion and headless goal in Manual Control App.
  * Change ng-if to ng-show to avoid child scopes issues.
  * Add dynamic goal sending and displaying in Manual Control App.
  * Add map displaying in Manual Control App.

* Merge branch 'ui-refactor'

  * Exclude hidden files in npm build stamp.
  * Fix gulp error does not return errorcode.

* Move nodejs into buildtool_depend.
* Merge branch 'display-current-action' into 'master'

  * Add request action on load and update action data from channel.
  * Show current action.

* Merge branch 'docking-visualization'

  * Add display of marker tf in live map.
  * Add display of docking cloud in live map.
  * Add display for docking reflectors in live map.

* Fix typo in docking plan display.
* Fix typo in mapx live view.
* Contributors: Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

1.14.2 (2020-10-23)
-------------------
* Display docking plan on live map.
* Fix task buttons missing when pin required for add task on agv panel.
* Freeze version of browserify. Newer version breaks shims in dependent libraries.
* Increase path speed limits.
* Merge branch 'path-planner'

  * Add costmap display on live map.
  * Display planned path on live map.

* Add maintenance notification.
* Merge branch 'node-consolidation'

  * Improve hardware test app.
  * Remove fuse status from hardware test.
  * Adapt the sytem info display based on the new HardwareInfo msg.
  * Move topics and msgs into agv05 namespace.

* Merge branch 'forward-fix-bionic'

  * Upgrade to node v10 and remove npm dependency which is included by default.
  * Prevent npm modifying npm entry in package.json.
  * Apply some forward fix for ubuntu bionic.

* Contributors: Patrick Chin, Tang Swee Ho

1.14.1 (2020-06-03)
-------------------
* Add server clock in UserPanel.
* Speed up task template rendering using webassembly GraphViz.
* Contributors: Patrick Chin

1.14.0 (2020-05-15)
-------------------
* Merge branch 'alt-param-display' into 'master' (`!32 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/32>`_)

  * Add alternative display for completed tasks with only boolean parameters.
  * Add alternative display for task with only boolean parameters.

* Merge branch 'min-battery'

  * Indicate low battery level in red in task runner.
  * Refactor to read manual charging state from battery_state topic.

* Add confirmation for agv parked at station initialization.
* Add obstacle sensing for manual control.
* Improve reflector UI.
* Merge branch 'task-runner-improvement'

  * Allow starting task runner in paused state.
  * Allow parameters for pre-init and custom-init tasks.

* Merge branch 'oee' into 'master'
  Overall Equipment Effectiveness (OEE) (`!30 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/30>`_)

  * Add downtime activity tracker.

* Merge branch 'white-label' into 'master' (`!29 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/29>`_)

  * Update logo url without using api call and update logo layout.
  * Add favicon and logo api view and update html for new image url.
  * Update webserver and webapp to display address and copyright label from database.
  * Add white-label config view, REST API and service on angularjs.

* Merge branch 'reflectors' into 'master' (`!28 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/28>`_)

  * Display reflectors data and landmarks in live map.

* Merge branch 'allowed-motion'

  * Check allowed motions in manual line follow app.

* Contributors: Farhan Mustar, Patrick Chin, Tang Swee Ho

1.13.16 (2020-01-13)
--------------------
* Add low disk space alert. (`!24 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/24>`_)
* Fix certain MSB states not showing in hardware-test.
* Improve npm_install.sh to include symlinked files and dirs.
* Contributors: Patrick Chin

1.13.15 (2019-11-22)
--------------------
* Merge branch 'task-columns'.

  * Add params and progress column in task history on UserPanel.

* Update wifi signal level.
* Fix hidden popup intercept mouse event.
* Merge branch 'wifi-strength-indicator', close #123.

  * Fix hostspot mode show no signal and adjust signal level to match wifi requirement of -75 db.
  * Fix layout.
  * Update notifcation to display wifi icon based on signal level.
  * Add wifi svg icon.

* Contributors: Farhan Mustar, Patrick Chin

1.13.14 (2019-05-27)
--------------------
* Improve npm_install script.

  * Make npm_install intelligently choose to run npm install or gulp build.
  * Fix npm_install not running although the build is outdated.
    This happens after manual build directly using npm or gulp.
  * Prevent npm_install.sh from failing silently.

* Improve wifi interface.

  * Autofill `dfautomation` in wifi password field when tapped 5 times.
  * Add toggle wifi password visibility.
  * Add dns-nameserver field in wifi static ip configuration.

* Modify hardware test to handle multi-port IO.
* Allow user to extend auto-poweroff countdown on user panel. Fix #30.
* Fix laser flicker on live view and add manual control rotation, fix #119.
* Implement amcl particle cloud visualizer (in debug mode).
* Contributors: Farhan Mustar, Nik, Patrick Chin

1.13.13 (2019-03-24)
--------------------
* Remove network settings from `Development Mode` app.
* Fix for debian packaging:

  * Add symlink to require'd files from agv05_webserver package.
    To ensure the files are copied properly during bloom-release.
  * Improve build to be independent of global npm version.
  * Migrate to package.xml version 3 and add copyright file.
  * Fix version of gulp-sass to stablise npm install process.

* Add cookie prefix.
* Contributors: Patrick Chin

1.13.12 (2019-01-21)
--------------------

1.13.11 (2019-01-12)
--------------------

1.13.10 (2018-11-30)
--------------------

1.13.9 (2018-11-29)
-------------------
* Add on screen manual control on map creator.
* Contributors: Farhan, Patrick Chin

1.13.8 (2018-11-27)
-------------------

1.13.7 (2018-10-20)
-------------------
* Fix mismatch LED blink control buttons in hardware test.
* Simplify html of hardware test app.
* Merge the bundle.js for tracked and trackless AGV.
* Display task-runner resuming state.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------
* Add RFID handler in hardware test.
* Contributors: nik3

1.13.5 (2018-09-07)
-------------------
* Fix bug line sensor tab in hardware test. Rear sensor display front data.
* Contributors: dramzpj

1.13.4 (2018-08-27)
-------------------
* Fix version display mismatch in help tab.
* Change hardware-test to use tab view.
* Display wifi error message and handle wifi card off condition.
* Contributors: Iwan, Patrick Chin <patrickcjh@gmail.com>, Xin Wei Lee, nikfaisal

1.13.3 (2018-07-16)
-------------------
* Scroll interface to top after exiting task-runner.
* Prevent starting default app when fms is out-of-sync.
* Add auto-starting default app.
* Fix app loading icon not cleared after failed start.
* Allow user elevation in UserPanel.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, Xin Wei Lee, Zhe Xian Low <zx_low@hotmail.com>

1.13.2 (2018-05-18)
-------------------
* Update hardware-test module for new inputs.
* Fix action-test app not handling param versioning properly.
* Add wifi-test module.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.1 (2018-05-08)
-------------------
* Update company address.
* Limit browser-sync version due to incompatibility with nodejs version.
* Add minimize button for safety-resuming popup.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.0 (2018-04-20)
-------------------
* Refactor code and move popup handling into agv05_executor node.
* Sanitize user message in popup.
* Add permission for pausing and resuming task runner.
* Refactor code for task runner auto-initialization. Fix #100.
* Refactor code. Fix #75.
* Modify task-runner template.
* Add play pause button for task runner.
* Auto-scroll to top after starting task-runner. Fix #91.
* Update webapp to display defaut init auto start countdown.
* Add agv version information in help tab.
* Change hardware test I/O to buttons and rearrange.
* Add pre init task template for webserver and webapp.
* Unhide stop task runner button.
* Change apps and task runner to start and stop using REST API instead of sockserver.
* Allow user to minimize user message.
* Mute certain safety types.
* Add alarm sound when safety popup is active.
* Add calibration app.
* Display template name on task history.
* Display template name on task runner if it differs from task name.
* Restrict npm package version incompatible with nodejs v0.10.25.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, ammar95, farhan, kxlee, nikfaisal

1.12.4 (2018-03-12)
-------------------

1.12.3 (2018-02-27)
-------------------

1.12.2 (2018-02-23)
-------------------
* Bug fix in wifi.
* Contributors: nikfaisal

1.12.1 (2018-01-03)
-------------------
* Disable hotspot toggle when the wifi card is off.
* Add WPA Enterprise encryption type for WiFi.
* Contributors: nikfaisal

1.12.0 (2017-12-03)
-------------------
* Animate agv motion on live map.
* Hide task call buttons when FMS sync pending.
* Allow tasks to be created in suspended state and resumed later.
* Enable `cancel_task` and `prioritize_task` operation from AGV.
* Allow filtering of tasks for other agvs.
* Add stub for fms permissions.
* Enable `create_task` operation from agv in FMS mode.
* Update task status enum.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.7 (2017-11-13)
-------------------

1.11.6 (2017-11-06)
-------------------
* Remove Resume button and fix bug motor wont run during hardware test.
* Add function to trigger IO 9-16 in hardware test.
* Update homing app UI to display FMS status.
* Add progress field and modify status enum in task.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.11.4 (2017-10-19)
-------------------
* Display parameters for running tasks.
* Fix issue with node-sass dependencies.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.3 (2017-10-02)
-------------------

1.11.2 (2017-09-26)
-------------------
* Fix Wifi UI.
* Fix proper WiFi message.
* Remove CsrfExemptSessionAuthentication.
* Improve boot loading UI.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.11.1 (2017-09-12)
-------------------
* Implement new wifi interface using wpa_supplicant.
* Contributors: nikfaisal

1.11.0 (2017-08-22)
-------------------
* Integrate fms status on task runner app.
* Disable task creation from AGV panel during FMS mode.
* Display fms icon and status on user panel.
* Add fms channel.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.6 (2017-08-08)
-------------------
* Add Customizable task template at task runner for RFID auto position reset.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>

1.10.5 (2017-07-21)
-------------------

1.10.4 (2017-07-12)
-------------------
* Hardware test: allow choosing to test front sensor or rear sensor.
* Allow reset to a station to start task runner. Fix #68.
* Update live map to support change in robot location format.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.10.3 (2017-05-22)
-------------------
* Add permission control for live map and status page. Fix #73.
* Fix virtual keyboard not shown for dev mode app. Fix #72.
* Implement live map for tracked agv. Close #54.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.2 (2017-05-08)
-------------------
* Add development mode module.
* Add missing files for installation.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.1 (2017-04-14)
-------------------
* Add response message for Wifi.
* Hardware test: add led blink control, expansion IO, and error LED control.
* Limit viz.js to version 1.5.1 as further version has issues.
* Speed up gulp by excluding heavy libraries from bundle. Fix #46.
* Better manage multiple streams in gulp task.
* Remove unnecessary gulp task.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.9.1 (2017-03-22)
------------------
* Hardware test: Add Fuse Status
* Hardware test: Show left and right speed.
* Hardware test: Correct Left and right motor.
* Contributors: Ikhwannudin503939

1.9.0 (2017-03-15)
------------------

1.8.0 (2017-03-04)
------------------
* Add button to open the network connection editor.
* Fix glitch in selection input popup.
* Display wifi ssid in webapp wifi popup.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.7.0 (2017-03-02)
------------------
* Allow user mentions using '@' to keep a popup message private.
* Turn selection input into popup not only for touchscreen but also for agv panel.
* Reduce throttling of battery message to use it as heartbeat for network disconnection.
* Add icon for manual charging and charging status.
* Add perms 'show popup warning'.
* Add resume button to resume motor by reseting the safety flag.
* Improve diagnostic status display on AGV panel.
* Display the motor relay status and charger status in hardware test.
* Remove start and stop button. Replace with exit button in hardware test.
* Add map layout overlay on live map view.
* Move map-creator to main start menu.
* Add virtual keyboard for string input in ActionTest app.
* Add micro grid. Make more "map" code sharable between webserver and webapp.
* Refactor code to make "map" code sharable between webserver and webapp.
* Fix bug in Firefox that prevents fill pattern with duplicated id from rendering.
* Move zoom controls from toolbar into map viz itself.
* Add live map stub.
* Move map broadcast from executor to here to improve performance and code reuse.
* Optimize rendering of laser scan in map creator.
* Add zoom buttons for map creator interface.
* Add saving indicator for MapCreator.
* Implement start, stop and saving of map.
* Change wireless connect message
* Fix map creator axis orientation.
* Remove ctrl key from being required for zooming in webapp.
* Implement map creator module (display only).
* Allow development on port 3000 using gulp and browser-sync.
* Remove viz.js from webapp to improve page load time.
* Add map creator stub.
* Enable manual-control module for trackless robot.
* Add process.env.TRACKLESS flag for javascript.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.6.0 (2016-12-01)
------------------
* Remove mailto link that could open external application.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* Allow laser sensor area selection in hardware test.
* Add relay control in hardware test.
* Add out of line and junction detected status in hardware test.
* Improve motor test: increase speed on press and hold, and decrease speed on release.
* Add motor, panel and obstacle sensor test
* Add Hardware Test app
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.4.0 (2016-11-02)
------------------
* Display virtual keyboard for task parameter of string type.
* Ensure app-based permission check always pass for superuser.
* Change 'SkillTest' to 'ActionTest'.
* Update branding.
* Modify boot screen
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.3.0 (2016-10-17)
------------------
* Add diagnostics viewer.
* Show task history in agv panel.
* Enforce task operation permissions.
* Add allowed_users field to TaskTemplate and filter call buttons accordingly.
* Implement authentication through websocket and enforce panel permisssions.
* Persist task to database.
* Add wifi configuration on front panel with on-screen keyboard.
* Replace select dropdown with popup dialog in touch panel.
* Check for websocket connection before publish.
* Display safety-triggered icon in notification bar.
* Disable manual-control module.
* Make modal popup take the full width and height of the panel.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.2.0 (2016-08-24)
------------------

1.1.0 (2016-07-28)
------------------
* Move version info display to backend instead of panel view.
* Contributors: nikfaisal

1.0.0 (2016-07-21)
------------------
* Enable wifi adhoc function in icon display.
* Use text file to store AGV05 hardware and sofware version.
* Remove the status menu at the moment.
* Update task_runner frontend due to executor trimming out taskTemplateMetas.
* Toggle extra help with 5 consecutive taps.
* Add remote assistance feature.
* Fix title not displayed and glitch in poweroff countdown.
* Change pin input to password type.
* Change default restApiUrl and sockjsUrl to favour port forwarding from production agv.
* Change error message when pin not entered.
* Show prompt when stopping current task.
* Add pin elevation in agv panel.
* Add agv_name field.
* Avoid glitch in displaying false countdown value.
* Add permissions for agv panel.
* Add countdown of two minutes before power off.
* Remove history tab.
* Add free motor function.
* Add keypad to speed controls in manual line follow and manual control module.
* Make virtual keypad trigger ng-change.
* Modify button size.
* Allow cancellation during task runner initialization.
* Add homing module.
* Show popup if task runner is unable to start.
* Enable location initialization.
* Add start and stop management for each module.
* Add hidden feature pull-to-refresh-browser at help page.
* Add 2-second delay and countdown timer for safety resume.
* Add popup for user input of task params.
* Fix home template to display more than 4 available task templates.
* Add safety triggered and safety resume popup.
* Add simple keypad directive.
* Add timeout to automatically close task status popup.
* Implement wifi manager.
* Add DF logo.
* Upgrade nw.js version to solve touch interface issue.
* Enable warning message display during robot start.
* Add multiple popup types and subscribe to topics that trigger its display.
* Automatic recheck whether task template data is received.
* Increase sizing of select input to improve touch, but to no avail.
* Add task_runner module.
* Add skill_test module.
* Reduce timeout for reload after soft reboot.
* Use scope.apply to update battery and volume value in notification, instead evalAsync. only show notification when robot running
* Add viz.js, which is usable in future.
* Add volume and battery percentage to webapp
* Add reload after timeout for soft reboot.
* Fix slow babelify transform.
* Allow ES6 for agv05_webapp.
* Add missing spinner.
* Implement rebootable robot controller.
* Upgrade ionic to fix bug with content resizing.
* Allow controls' orientation to be rotated.
* Add numeric indicator on speed bar.
* Remove roslibjs as it is no longer in use.
* Enable touch event for nw.js browser.
* Added manual control and manual line follow module stub.
* Implement channels on top of sockjs protocol.
* Update package metadata.
* Fix catkin_lint issues.
* Use factory as a more proper way to create angular service.
* Fix controller name decorator.
* Update roslint options.
* Update nwjs browser width.
* Add "system" icon link.
* Insert "ngInject" where implicit matching fails.
* Initial commit.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com, Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

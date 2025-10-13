^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_webserver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'nav-improve'

  * Add option to start Task Runner with previous map tracker location.

* Merge branch 'location-hint'

  * Add location_hint in agv serializer.

* Merge branch 'ui-improvement'

  * Add permission to view live monitor tools in task runner.

* Merge branch 'component-tracking'

  * Allow data upload and sync from DF Hub.
  * Add AMR part and service-log pages.

* Fix not found display for various page.
* Merge branch 'panel-dashboard'

  * Expose overlay path in cache and update dashboard to search package in overlay.

* Merge branch 'hw-app'

  * Serve hw-app asset using nginx.

* Merge branch 'panel-dashboard'

  * Add dashboard template and dev server.
  * Fix hwapp dev server.
  * Define dashboard assets using ros params.
  * Add log channel for app sockserver.

* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.

* Update log sorting using LooseVersion.
* Merge branch 'system-log-downloader'

  * Fix log file size information.
  * Add api to navigate logs directory.
  * Improve diagnostic log api and view.
  * Add progress update when bundling log.
  * Rename file.
  * Monitor file status using PUT request, and download when ready.
  * Fix only can serve from media root, increase download log cleanup frequency and remove immediately after nginx serving.
  * Use celery to bundle log, add log cleanup celerybeat schedule and update api view to use celery task.

* Merge branch 'camera'

  * Synchronize depth and color images.
  * Rotate image based on camera orientation.
  * Add multiple depth cloud coloring modes.

* Prefers hot reload.
* Limit queue size.
* Disable MySQL performance schema to reduce memory usage.
* Fix regression bug for agv activity ordering and task list download.
* Reply with error if executor not running.
* Merge branch 'add-missing-protected-parameter-option'

  * Add protected parameter option in restore feature.

* Merge branch 'fix-uncleared-background-interval'

  * Fix roslint error.

* Add load time warning to the hw_app_dev.
* Fix MySQL max_allowed_packet config for Ubuntu Bionic.
* Merge branch 'hw-app'

  * Add more api function.
  * Simplify dev server api response to make it easier to customize.
  * Allow loading plugin without asset entry for development.
  * Display hw app name similar to navwiz display.
  * Fix to reload entry dynamically.
  * Connect hw app and frontend.
  * Add sockjs to hw_app_dev.
  * Change to serve using tornado to also handle websocket.
  * Load hw app and serve the static files.
  * Add hw_app_dev node.
  * Resolve entry into url.
  * Remove hw-app endpoints and merge it with apps endpoints.
  * Add hw-app api endpoint.
  * Add hw-app endpoint to serve the assets.
  * Add cache field for models_app_descriptions.

* Upgrade python version.
* Optimize message conversions.
* Fix linter error.
* Fix machine id always changing.
* Merge branch 'fix-missing-variable-list' into 'master'

  * Add REST API endpoint for variables list.

* Fix advanced restore skip transition trigger and map param due to return.
* Amend agv activity export title text.

* Contributors: Farhan Mustar, Patrick Chin, Tan Kui An Andrew, Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Update to sveltekit webapp.

  * Add usb update ui form.
  * Add usb software update endpoint.
  * Update setup.py.
  * Remove unused files.
  * Fix to obtain validation message from older version.
  * Update to obtain csrf_cookie_name dynamically.
  * Add license edit page and update info for help page.
  * Add license endpoint.
  * Add view panel permission to agv_panel user.
  * Add new endpoint for agv panel login using token.
  * Update generate panel token to new path.
  * Convert agv panel token auth backend to session auth backend.
  * Update group permissions.
  * Add UserOwnProtectedMixin.
  * Add change my password.
  * Add trackless permission config viewset.
  * Add protected parameter feature to parameter editor and endpoint.
  * Add update permission for delete parameter.
  * Add warning for parameter endpoint issue.
  * Update permission for agv and parameter endpoint.
  * Update menu display based on permissions.
  * Add License Void Mixin
  * Add FmsVoidMixin and handling
  * Add Map Quality Score page
  * Handle duration display for cancelled task.
  * Add download diagnostics feature for task completed.
  * Add download feature for monthly task completed.
  * Add download feature for monthly agv activity.
  * Add Agv Activities download view.
  * Add Task Completed download view.
  * Add task completed list.
  * Add agv activity configuration.
  * Add agv and task report.
  * Update eap form to use ClearableFileInput and add toast when successfully submit.
  * Update frontend for new user endpoint.
  * Add password validation and also exclude guest as groups option.
  * Add toast info for dfleet active status.
  * Add dfleet mode response info.
  * Add agv config page
  * Add diagnostics page
  * Update validation_data to send only on changed.
  * Add available filter options in task templates endpoint and add filter widget to task templates page.
  * Add filter_class for task templates endpoint.
  * Add mass update page and fix enpoint to only update provided data.
  * Add api endpoint for task template mass operation.
  * Add date-time page
  * Add search for transition trigger.
  * Add description and url to search results
  * Add system monitor page
  * Add advanced restore feature
  * Add restore feature
  * Add backup and advanced backup feature
  * Add group config page
  * Add validation data channel and display validation msg.
  * Change validation_msg to validation_data.
  * Amend hardware plugin checks.
  * Update white_label to show download link with file name.
  * Add white-label form view.
  * Add white-label update endpoint.
  * Add software patch page.
  * Add hardware plugin config page.
  * Update api and link to ui for permission update and reset.
  * Add permissions view.
  * Add assembly info and preventive maintenance page.
  * Add backend API for io configuration.
  * Add laser-sensors editor page.
  * Add laser config endpoint.
  * Add parameter reset and remove option.
  * Add ClearableFileField on audio form.
  * Add parameters download view and make FileField produce download url.
  * Handle audio form when node offline.
  * Add audio api endpoint.
  * Add submit parameter form.
  * Add parameter list and edit form view.
  * Disable parameter endpoint pagination and add online and controller grouping.
  * Add protected info on detail layout.
  * Add parameter endpoint.
  * Prefer OrderedDict response in api endpoint.
  * Add user config page
  * Add camera channel to publish metadata.
  * Add variable editor and set prettier to use bracketSameLine rule.
  * Update /config/webhooks.
  * Response OK (200) instead of bad request (400) during exception.
  * Update /config/robot-config.
  * Add map ocg editor.
  * Add ocg list view and update ocg endpoint.
  * Add ocg selection in mapx layout editor.
  * Add ocg api endpoint.
  * Add env checking, add mapx layout editor and rearrange code.
  * Add mapx endpoint.
  * Add map param editor.
  * Add map transition trigger editor and add multiple option in Select component.
  * Add map teleport editor view.
  * Add active map editor.
  * Update map changeset-view and add backend api for it.
  * Update clone map api.
  * Add map annotation editor.
  * Add checking for updated while editing.
  * Add map layout editor.
  * Add map list view.
  * Allow both Django REST framework endpoints to work, with or without a trailing slash.
  * Add trackless env variable inside auth endpoint.
  * Update /config/network/eap.
  * Update /config/network/https.
  * Update /config/network/hostname.
  * Update /config/network/identify.
  * Update /config/network/configuration.
  * Update /config/network.
  * Add global param editor.
  * Add task template users edit page.
  * Add copy and delete task template ability.
  * Add last-cached and last-saved task template view.
  * Add force overwrite option for task template editor and rearrange code.
  * Add task template add capability.
  * Connect with backend, rearrange ui and add task template service.
  * Add task-templates metadata endpoint for task templates editor.
  * Fix config rest api to redirect from root rest api.
  * Add task templates list view and api.
  * Add search api, refactor fetch api and linkup search api to search modal.
  * Add agv activity stats builder, add billboard.js and plot pie chart for agv activity card.
  * Add task for mileage stats and plot the data on mileagecard.
  * Add battery stats task and update battery card to display the data.
  * Add celery task for dashboard stats.
  * Add today's task statistic.
  * Update data calc for pm card.
  * Add server-clock store and add pm card.
  * Add skillset image.
  * Add dashboard data.
  * Add white-label store and update page to use it.
  * Add Perm components to check for permissions.
  * Add login interface.
  * Configure to work with django backends.
  * Update webserver to be able to run rest api.
  * Update build directory and dev server port.
  * Remove webserver pages.

* Update for python 3 support.

  * Update postinst and postrm for python version compatibility.
  * Update python_2_unicode_compatible from six module.
  * Update yaml to use full load.
  * Add more decode based on python3 and fix basestring check with six.
  * Update package.xml
  * Fix shapely version.
  * Fix sort error.
  * Update string type.
  * Update StringIO and base64.b64encode.
  * Update dict keys, values and items view.
  * Update hashlib, license and ordereddict view chaining for python3.
  * Add universal_newlines to the subprocess call.
  * Update requirements.
  * Update python dependency.
  * Fix bytes handling in python3.
  * Upgrade python version.
  * Fix robot config bug in python3.
  * Fix postinst to determine install environment based on distribution.
  * Update CMakeLists to make python compatible.
  * Update wraptor for compatibility.
  * Fix noetic-roslint warning.

* Update License

  * Update license page.
  * Use machine ID as AGV UUID.
  * Read agv_uuid from agv05 database only.
  * Update license signature scheme to ed25519.
  * Update LicenseVoidMixin.
  * Amend migration.
  * Protect some variables and store them in agv05 database only.
  * Change AES operation mode to GCM.
  * Update license to new format.
  * Compute device fingerprint.

* Add sockserver debugpy config.
* Add navwiz sub folder in download log feature.
* Fix hostname setting in Ubuntu Focal.
* Accept bool for app endpoint.
* Optimize out N+1 queries.
* Merge branch 'hal' into svelte-webapp

  * Support 5 laser topics and 6 lidars for docking target display.

* Validate package.
* Merge branch 'nav-omni' into svelte-webapp

  * Add omni drive flag into environment variable and allowed motion.

* Merge branch 'secure-py' into svelte-webapp

  * Enforce using DF's Python release.

* Merge branch 'task-template-variables-api' into svelte-webapp

  * Add api for GetVariable and SetVariable

* Fix sockserver crash if no next_pm_due.
* Add navwiz package for patch log
* Merge branch 'fix-canvas2d-bug'

  * Fix canvas2D bug due to frequent read-back

* Merge branch 'nav-improve-p3-map-10mm' into 'master'

  * Fix junctions are hard to be selected.
  * Add positioning resolution layer of 10mm on map-editor.
  * Fix cursor coordinate is not snap to the grid.

* Merge branch 'fix-cached-server-time' into 'master'

  * Refactor code and use timezone-aware format.
  * Retrieve server time on_subscribe

* Merge branch 'fix-software-patch-process' into 'master'

  * Set noninteractive for dpkg install command

* Fix ui glitch when cloning map annotation
* Fix mutable method parameters pointing to shared variable.
* Remove unnecessary ordering in query.
* Support up to 5 lidars for docking target display.
* Contributors: Farhan Mustar, Patrick Chin, Tan Kui An Andrew, Tan Wai Liang, Wong Tze Lin

3.0.0 (2023-08-02)
------------------
* Fix npm install issue.
* Refactor form.
* Merge branch 'update-df-logo'

  * Update df logo

* Merge branch 'patch-plugin'

  * Add navwiz-core pkg for patch changelog

* Merge branch 'hot-reload'

  * Add permission.
  * Handle hot reload request when controller not running.
  * Add hot reload ui and api.
  * Add reloadable cache flag.

* Merge branch 'fix-advanced-import-behaviour'

  * Fix advanced import behaviour of TransitionTrigger.
  * Fix advanced import behaviour of teleport.

* Merge branch 'fix-parameter-update'

  * Update disabled multiple-choice to be able to view all options and refactor code.
  * Disable dropdown button for disabled parameter.
  * Fix obstacle sensor cannot be disabled.

* Replace Object fromEntries to support old browsers
* Fix form width.
* Fix map annotation invalid form.
* Merge branch 'map-annotation-feature'

  * Check for concurrent edit.
  * Modify live map for new cache format.
  * Modify map-annotation to merge models and scene.
  * Remove map annotation plumbing.
  * Modify map annotation models fields.
  * Sync changes with agvccs.
  * Allow multiline textAnnotation.
  * Modify style of polygon annotation.
  * Update selection behaviour and refactor code.
  * Add annotation edit button in map list view.
  * Remove unnecessary grid in map annotation scene.
  * Remove generateTextAnnotation function.
  * Add ruler to mapx annotation toolbar.
  * Sync map annotation with FMS.
  * Backup restore map annotation.
  * Clone map annotation when cloning map.
  * Remove unnused drawTextAnnotation.
  * Add hud to control visibility of mapx annotation.
  * Integrate mapx-annotation into webapp.
  * Add mapx-annotation toolbar and scene.
  * Add mapx annotation editor page & reuse mapx layer.
  * Add hud to control visibility of map annotation.
  * Trigger model cache dirty if map annotation change.
  * Integrate map-annotation into webapp.
  * Add shortcut key listener for toolbar.
  * Remove unused lift and ruler.
  * Add icon annotation.
  * Add text annotation content update feature.
  * Add font size adjustment feature.
  * Save and load annotation.
  * Add toolbar with text and shape annotation UI.
  * Add map annotation editor page & reuse map layer.
  * Add migration file for MapAnnotation model.
  * Add MapAnnotationAdmin.
  * Add MapAnnotation model.

* Fix linter error.
* Merge branch 'update-transitiontrigger-and-teleport-form'

  * Allow inherit from variables.
  * Fix variables display in teleport and transition-trigger matching task template.
  * Filter out inactive task templates in Teleport & TransitionTrigger form
  * Pass global params to teleport form.
  * Pass global params to transition trigger form.
  * Pass task template variables to teleport form.
  * Pass task template variables to transition trigger form.

* Merge branch 'protected-parameter'

  * Rename laser parameter.
  * Add url param check to enable protected parameter modification.
  * Rename audio parameter.

* Fix UI glitch.
* Read from NO_AUTO_POWER_OFF environment variable.
* Merge branch 'secs-gem'

  * Change from transaction_cfg to transaction_enabled.
  * Add resume transaction option on transaction panel.
  * Add permission for transaction abort and cancel.
  * Add transaction api endpoint.
  * Add transaction_cfg as fms downloadables.
  * Update transaction list display and add current action display.

* Merge branch 'safety-margin'

  * Swap form fields positions.
  * Add safety margin in AGV dimension form.

* Merge branch 'custom-init'

  * Allow user to limit the station choices of "AGV is Parked at a Station".
  * Indicate that initial dock to charger is for tracked mode.
  * Subsection custom initialization dropdown selection to separate the built-in from the task templates.
  * Fix the custom initialization choices in AGV form are not updated for the lifetime of the program.
  * Allow user to hide predefined initialization.
  * Remove top-level requirement for pre-init and custom-init task templates.

* Update log download page submit text.
* Merge branch 'manual-layout'

  * Add manual layout to task template editor.

* Add shapely to pip requirements file.
* Merge branch 'additional-soft-reboot-button'

  * Separate soft reboot btn from validation-message.
  * Refactor safe_soft_reboot.
  * Changed to safe_soft_reboot operation.
  * Ensure no running tasks when soft reboot.
  * Added soft reboot shortcut button.

* Merge branch 'diagnostic-view'

  * Handle diagnostic view if no diagnostic file.
  * Arrange diagnostics in descending modified time.
  * Add diagnostic-view to tasks-completed view.
  * Add download button to download csv.
  * Fix nav-action-hint error on null data.
  * Add navigate to column function.
  * Handle null value in filter selection.
  * Add filter plugin.
  * Update display style.
  * Add columns selection.
  * Add navigation hint.
  * Add progressbar.
  * Rearrange to separate toolbar from table and handle close function.
  * Fix resource not found when django collect.
  * Add diagnostic table view and link click event with loading the viewer.
  * Add diagnostic-view page.

* Merge branch 'live-task-template'

  * Disable note if not in edit mode and refactor.
  * Fix task-template editor event triggering.
  * Update live data on action update and add models api to render.
  * Add task template live view and obtain templates data from task api.

* Merge branch 'log-download-page'

  * Fix to trim active log to the TarInfo header size.
  * Change streaming tarfile to compressed mode.
  * Remove ros log due to unknown end of file error, handle if fail to get creation time and refactor.
  * Fix permission error, read using dd in subprocess.
  * Add log download page.

* Merge branch 'remote-io-ui'

  * Added RemoteIoChannel and remote IO live view UI.

* Merge branch 'task-template-action-search'

  * Add task template action search using selectpicker.

* Merge branch 'protected-param'

  * Default to untick protected parameter in backup restore.
  * Ensure parameter restore behave the same when client unavailable.
  * Add permission to permission view.
  * Rename permission to change_protected_parameter.
  * Add handling for protected parameters restore.
  * Disable update for protected_parameter and add protected_parameter option in backup_restore.

* Merge branch 'task-template-note'

  * Update task template note display.

* Merge branch 'mapx-layer-feature'

  * Prevent expand selection to include disabled layers.
  * Undo unnecessary changes.
  * Fix fullscreen popup overlap with toolbar if toolbar overflow and change panel to default theme.
  * Fix layer behaviour on revision history view.
  * Adjust display style.
  * Update active-object on layer visible change.
  * Change layer popup panel and add color hint.
  * Rearrange layer controller file.
  * Update construct color based on active layer and update active layer behaviour.
  * Add active object color for diff layer.
  * Fix index error when splitting muted render.
  * Update point and rect hit test to consider visible object.
  * Rearrange layout and split drawing for muted objects.
  * Move expand selection into active-object.
  * Added expand selection feature.
  * Fix multiple select auto apply previous layerNum.
  * Improve UX for selection of objects by hiding objs.
  * Fix for layer not updated if switch active obj.
  * Show layerControlBtn in fullscreen mode.
  * Allow user to update layer property.
  * Changed layer setting to side panel.
  * Added active layer & layerInView feature to mapx.
  * Add layer to mapx junctions, paths & stations.

* Remove unused codes.
* Merge branch 'transition-trigger'

  * Update backup restore handling.
  * Default cancelNonStopTransition set to true.
  * Sync changes from DFleet for transition_trigger.

* Clear preventive maintenace popup upon checking conditions cleared.
* Merge branch 'time-jump-safety-trigger'

  * Add time jump error code and hardware test display.
  * Add migration file.
  * Add TIME_JUMP to ErrorCode class.

* Remove unused file.
* Merge branch 'hssid_eap'

  * Allow download cert and p12 files.
  * Add support for p12 file with ca in it.
  * Update wifi crt to use p12 file and add cert details in network view.
  * Save eap certs & config to media folder.
  * Added scan_ssid.
  * UI improvement for hidden ssid & refactoring.
  * Support for hiddenSSID & WPA2-EAP.

* Merge branch 'no-rotate-zone'

  * Allow merge-and-remove of a zone's corner.
  * Group no-rotate and forbidden zones under the same toolbar menu.
  * Fix UI glitch.
  * Remove duplicate files.
  * Add no-rotate zone for tracked agv.
  * Add no-rotate zone to live map.
  * Add no-rotate zone in map editor.

* Merge branch 'titan-svg'

  * Add Titan SVG template.
  * Limit figures in robot SVG template to 4 decimals.

* Fix UI glitch. Multi mode active when ctrl key not pressed.
* Fix autofocus for station search.
* Improve path display.
* Refactor code.
* Remove warranty information.
* Improve backup restore UI and refactor code.
* Merge branch 'path-forward-reverse-motion'

  * Improve multi-path flow toggle to retain direction relative to each other.
  * Allow user to set path forward or reverse motion in map editor.

* Return map ocg in REST API for trackless mode.
* Fix UI bug and refactor code.
* Fix task template mass delete layout.
* Fix typo.
* Fix search page layout bug.
* Refactor task template overlay.
* Refactor script.
* Fix postinst script failure during fresh install.
* Merge branch 'mass-task-template-edit'

  * Change edit tags icon and fix table layout.
  * Add mass update category forms.
  * Rearrange task templates selection ui.
  * Add mass task template edit view.

* Merge branch 'task-template-drag-outcome'

  * Update activeObject after drag completed.
  * Add arrow indicator.
  * Add drag outcome behaviour.

* Disable auto open browser.
* Merge branch 'vscode-support'

  * Update launch file to prefix with debugpy if DEBUGPY env is true.

* Merge branch 'tracked-branching-for-trackless'

  * Make map editor junction focusable using search prefix _jid\_.
  * Add trackless map branching editor from dfleet.
  * Revert trackless map editor.
  * Bug fix for same simulated path being considered.
  * Bug fix for multiple select caused invalid tracked.
  * Allow path list to be overrided by simulated paths
  * Added excludeP feature for canBranch function.
  * Add dynamic path toggle into consideration.
  * Hittest check when clicked and improve UI UX.
  * Added additional hit test to handle more cases.
  * Refactor hitTestPathAndPathTracked.
  * Allow tilted bezier curve and minor rearrangement.
  * Additional checking for both bezier paths case.
  * Restrict control point placed on junction.
  * Update bj1 bj2 if junction is dragged.
  * Basics of tracked branching with hit test.
  * Added tracked attribute for hittest.

* Fix migration during testing.
* Merge branch 'search-page'

  * Add search page feature.

* Merge branch 'time-synchronization'

  * Remove unnecessary force-sync and reduce response delay.
  * Automatically set DFleet's IP as local NTP server.
  * Allow setting of local NTP server.
  * Display synchronization status.
  * Replace systemd-timesyncd with chrony.

* Fix bug clearing station exclusion list in agv task trigger.
* Fix sql query for compatibility with newer MySQL 8.0.
* fix Mapping & Navigate config hidden in trackless
* Merge branch 'fix-laser-sensor-form-view'

  * bug fix due to Parameter model change

* Merge branch 'tracked-and-trackless-share-parameters'

  * Update migration file to copy ROS variables.
  * Store ROS variables in agv05 database.
  * hide Mapping & NavigateTo config in tracked mode
  * added migration file to copy agv05x parameter
  * store parameters in agv05 database only

* Refactor code.
* Backward compatible with older cache.
* Merge branch 'task-templates-allowed-groups'

  * Refactor code.
  * consider allowed_groups in home controller
  * allow task-templates to be assigned to user groups

* Fix error.
* Fix watchify.
* Optimize by making use of prefetch results.
* Merge branch 'patch-plugin'

  * Rename to software_patch.
  * Remove auto softreboot when apply patch.
  * Use software update progress view to display patch progress.
  * Autoformat and remove unused imports.
  * Rename patch plugin to patch.
  * Add flag to dpkg to ignore dependency, rearrange javascript file and remove unused code.
  * Added patch-plugin feature.

* Rename "FMS" text to "DFleet".
* Merge branch 'line-sensor-error-msg'

  * Rename line sensor com error to line sensor error.

* Merge branch 'robot-svg'

  * Update labels.
  * Update robot SVG in live map.
  * Add config form for payload dimension and SVG.
  * Add config form for AGV dimension and SVG.

* Merge branch 'global-variable'

  * Run autoformat and cleanup.
  * Add backup restore feature.
  * Fix to allow for empty string default value.
  * Update variable edit permission.
  * Rename variable field name.
  * Allow task template paramter to accept variable type.
  * Update task template editor to handle variable type and allow inherit variable value.
  * Add variable editor form and add models field for variables.

* Merge branch 'teleport-reset-position-tracker'

  * Add option to reset position or reset position tracker after teleport.

* Merge branch 'teleport-align'

  * Add align_station_type option for teleport.

* Merge branch 'agv-task-action'

  * Add action field in AgvSerializer.

* Merge branch 'usb2can-support'

  * Blacklist rename interface from network configuration page.
  * Blacklist socketCAN interface from network configuration page.

* Prevent duplicate map timestamp after advanced restore.
* Merge branch 'map-parameter-backup'

  * Include map parameters in backup file.
  * Rename "Restore" to "Restore (clear local)".
  * Fix bug and rearrange code.

* Merge branch 'dont-clear-parameters-when-restore'

  * Update parameter restore behaviour to not reset all parameters.

* Merge branch 'unicode-fix'

  * Fix error downloading custom log containing unicode chars.

* Merge branch 'io-monitor-popup-dialog'

  * Reduce magic number.
  * Add longer delay to wait for initialization.
  * Removed unnecessary locks.
  * Fix linting and unittest.
  * Added rostest for IoChannel.
  * Added IoChannel & IO monitor popup.

* Merge branch 'notification-ui'

  * Remove volume icon from UserPanel.

* Fix missing dependency on capabilities package.
* Fix ntp sync status check in Ubuntu Bionic.
* Merge branch 'dynform-optimization'

  * Avoid repeated table numbering update.
  * Populate dropdown only when it is in focus.

* Merge branch 'realtime-pose-update-on-simulator-robot'

  * Fix test DB settings.
  * Display robot pose from topic for tracked_sim robot.

* Contributors: Andrew Tan, Chow Yeh Loh, Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Rui En Lee, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Fix advanced backup restore omitting inactive task templates.
* Fix backup restore task template conflict resolve bug.
* Fix postinst script to test for link existance.
* Merge branch 'network-branch'

  * Mute error output when generate certificate.

* Fix laser area reset to default not working.
* Fix permission for system-monitor page.
* Merge branch 'rest-api-pause-agv'

  * Handle permission in rest api.
  * Add rest api endpoint for pause agv.

* Merge branch 'mass-edit-task-template'

  * Add permission check for mass edit task template.
  * Rearrange layout and redirect to single form if only one item selected.
  * Add task templates mass user edit view.
  * Update mass delete layout.
  * Added mass delete feature.

* Merge branch 'set-pose'

  * Add controls to relocalize agv pose in manual control app.

* Optimize marker pose publishing.
* Merge branch 'web-video-server'

  * Add permission object.

* Fix laser view when laser component does not exist.
* Stop default link click behaviour.
* Merge branch 'map-live-edit'

  * Move default nav-to params to executor module.
  * Disable live construct when live state changed.
  * Handle live update for ocg.
  * Add reverse go to.
  * Add goto pose action in liveapp for ocg editor.
  * Add live app to ocg editor.
  * Display reflectors in live app panel and editor.
  * Add reverse navigate to and go to action.
  * Prevent sending command when live robot working.
  * Update live app channel to only allow single connection.
  * Rename live program to live app.
  * Ensure can navigate-to from headingless station.
  * Handle fail to navigate to when stations name are invalid.
  * Add reset tracker capability and rearrange.
  * Add goto capability in live app.
  * Fix to disable nav to button when current location is unknown by tracker or no station selected.
  * Rearrange so that live app is part of controller start option.
  * Send current map metadata everytime.
  * Add lidar display.
  * Add navigate to interface.
  * Update live app to send ocg id insead of fetching and send from frontend.
  * Set disconnect timeout to 10 sec and fix raw map format when serialize.
  * Add live-map on live program app and add ocg interface to set map from client.
  * Add reset agv pose and refactor.
  * Add live-program channel module and add live app connector on layout editor.

* Fix laser scan display.
* Rearrange menu.
* Limit task queue to 100 active tasks.
* Merge branch 'darken-raw-map'

  * Add darken raw map feature.

* Merge branch 'rotateMapFeature'

  * Change rotation method to use canvas on canvas and rearrange.
  * Added rotate map feature to raw map editor.

* Merge branch 'fix-chown-overlay'

  * Fix changing owner of overlay folders.

* Silent get pluging version on error.
* Freeze bootstrap-sass version.
* Merge branch 'map-quality-score'

  * Handle directory missing exception.
  * Add map quality report.
  * Add view_map_quality permission object.

* Merge branch 'validation-msg-link'

  * Allow validation message to use django template.

* Merge branch 'io_rename_fix'

  * Update io form, update io hardware display and add data to backup restore.
  * Add a function to rename the IO name used in hardware_test.

* Merge branch 'map-clone-path'

  * Handle station clone.
  * Modify branching update for clone.
  * Add clone capability for tracked map.
  * Add clone ability for selected objects and replace multi selection shortcut with clone mode.

* Merge branch 'map-multi-select'

  * Add tracked multi select feature.
  * Indicate placeholder when path does not have common speed.
  * Modify drag behaviour to match previous version.
  * Update drag for multiple objects.
  * Fix hitTestBezierAndLine out of bound checking.
  * Change toolbar to use the same properties field in multi selection mode.
  * Add station rotation on multi station.
  * Add toggle path flow for multi selection.
  * Add new selection mode and refactor.
  * Add path speed handler for multi path selection.
  * Update webapp activeObject to match webserver.
  * Update multiple obj is remove allowed.
  * Update undo tracking for remove item and update toggle tracked for multiple path.
  * Add multi properties for multiple object selection.
  * Add discardObject to handle remove selection mode and remove discardMode flag.
  * Add bazier bound check before find intersection.
  * Move hit detection to hit test and improve path detection.
  * Add object select and unselect handler.
  * Update active-object to handle multiple objects.
  * Add area selection indicator.

* Customize settings for running tests.
* Add visualization for laser intensity threshold.
* Merge branch 'network-preset'

  * Update network config page to show preset information.
  * Update form factory and validation for network preset.
  * Update robot config view to handle network preset.
  * Fix upload robot config does not toggle mobile robot provider.
  * Change robot config network table to handle network preset.

* Merge branch 'sockjs-fix'

  * Drop support for other SockJS transports except Websocket for performance and reliability reasons.

* Merge branch 'backup-restore-fix'

  * Fix typo bug in advanced inactive map restore.
  * Fix suffix to limit length to 127.

* Contributors: Farhan Mustar, Lee Rui En, Ng Yi Liang, Ooi Zhen Zhi, Patrick Chin, YC Lee

2.1.0 (2021-11-24)
------------------
* Merge branch 'ujson-memory-leak'

  * Eliminate the use of ujson library.

* Merge branch 'simulator-expansion'

  * Add trackless_simulator robot and SimulatorMobileRobot interface.

* Merge branch 'preventive-maintenance'

  * Fix bug resetting mileage.
  * Increase limit of p.m. due date to 15 months from now.
  * Limit next p.m. mileage to be within 20000km of current mileage.

* Update the installer disk identifier.
* Add a button on map to trigger auto focus on the agv.
* Show AMCL particle cloud only when feature flag (SHOW_AMCL_PARTICLES) is set.
* Fix concurrent build failing due to node_modules symlink.
* Force output of celerybeat to screen.
* Amend envvar validation in robot config, allow adding '/' symbol.
* Add missing imports.
* Fix bare except statements.
* Add ros distro checking in hardware plugin validation.
* Merge branch 'path-planner'

  * Fix context menu not visible in editor fullscreen mode.

* Contributors: Farhan Mustar, Ng Yi Liang, Patrick Chin

2.0.0 (2021-05-25)
------------------
* Refactor code.
* Handle unicode BOM in task diagnostic download.
* Prevent map jump when resetting AGV position.
* Enable AMCL particle cloud display in production mode.
* Run autopep8.
* Fix watchdog hit test in laser-sensor editor to match agv05_lidar.
* Merge branch 'mobile-robot-version'

  * Update plugin version display to display version name.
  * Add plugin version info to version endpoint in rest api.
  * Add version validation for the uploaded plugin file.
  * Add mobile robot version display in hardware plugin upload page title.

* Merge branch 'lidar-config-min-activation'

  * Update max limit to 100 for min activation.
  * Update laser area editor to add min activation parameter.

* Merge branch 'obstacle-hint'

  * Update laser view and channel for new topics parameter format.

* Add next_motion as a provided parameter in teleport form.
* Unhide agv05_lidar node in parameter view.
* Separate direct and transitive pip dependencies into two requirements files.
* Refactor code to match DFleet's.
* Fix json serialization.
* Update laser scan topics when laser editor page reloads.
* Merge branch 'path-planner'

  * Disable dynamic path planning unless feature flag is set.
  * Add path_blocked error code.
  * Add dynamic path.
  * Add forbidden zone in trackless map.
  * Refactor laser editor code.
  * Subscribe to full costmap only.

* Merge branch 'ujson-update' into 'master'

  * Upgrade ujson to v2.0.3 and fix serialization issue.

* Merge branch 'network-fix'

  * Fix network adapter IP address not being applied in Ubuntu Bionic.

* Skip data broadcast if nothing changes.
* Merge branch 'action-tooltip'

  * Add tooltip editor for task template.

* Fix flip detection to have more tolerant.
* Merge branch 'ui-refactor'

  * Exclude hidden files in npm build stamp.
  * Highlight junction after merging.
  * Handle RFID data when merging junction.
  * Refactor junction dragging in trackless map editor.
  * Refactor hit test.
  * Refactor junction dragging in tracked map editor.
  * Adjust AGV task-trigger form alignment to provide context.
  * Fix map editor glitch.
  * Ensure param and path speed range is validated on UI side.

* Update backup restore error messages.
* Merge branch 'soft-reboot-after-hostname-change'

  * Force soft-reboot after hostname change to avoid issue with ROS master.

* Fix gulp error does not return errorcode.
* Merge branch 'remote-assistance-fix'

  * Fix invalid directive in SSH config for Ubuntu Trusty.

* Run autopep8.
* Merge branch 'software-update-improvement'

  * Upgrade kiosk separately if it was not installed.
  * Make apt try harder to resolve unmet dependencies.

* Move nodejs into buildtool_depend.
* Remove stage_node from the list of mobile robot providers.
* Prevent status channel crashing due to non-utf8 bytes in diagnostic msg.
* Fix task diagnostics download bug.
* Filter out removed stations in AGV idle trigger form.
* Fix style class.
* Merge branch 'merge-agv05x-to-agv05-repo'

  * Rename and move agv05x_nav to agv05_navx.

* Allow laser editor for tracked mode.
* Merge branch 'upgrade-backup-restore' into 'master'

  * Support advance backup
  * Remove info showing for old file and support advance download.
  * Add robohash.
  * Add new meta data into backup file.

* Merge branch 'fix-map-editor-branch-drag' into 'master'

  * Fix branching test bug for s-curve and bend paths.
  * Fix dragging junction bugs.
  * Fix junction dragging does not update branching.
  * Refactor branch hit test.
  * Refactor branching to fix hit test side effect and add path id and branch id in scene.

* Fix UI bug in robot config page.
* Merge branch 'ocg-optimization'

  * Subscribe ocg in png format to reduce size. Fix #78.
  * Reduce zlib compression level in ocg editor.

* Defer reading laser scan topics from param.
* Fix error in date-time settings for Ubuntu Bionic.
* Merge branch 'docking-visualization'

  * Add display of marker tf in live map.
  * Add display of docking cloud in live map.
  * Add display for docking reflectors in live map.

* Fix missing mobile_robot_provider in robot config file.
* Fix the '@' character in postinst.em file removed by templating.
* Contributors: Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin

1.14.2 (2020-10-23)
-------------------
* Merge branch 'hardware-plugin'

  * Validate plugin file extension.
  * Add page to upload hardware plugin.

* Add usbmount script to download log if usb contain magic file.
* Add audio download view, close `#52 <https://gitlab.com/dfautomation/product/navwiz/agv05_ui/-/issues/52>`_.
* Display docking plan on live map.
* Merge branch 'single-lidar-node' into 'master'

  * Add error message if laser sensor not available.
  * Add live obstacle detection in laser sensor editor.
  * Add copy and paste region.
  * Update laser channel, update laser area editor for new laser channel and refactor.
  * Enable panning in laser area editor.
  * Update laser view for single lidar node.

* Add dev_mode_server as controller in parameter list.
* Split parameter list by controller and hardware.
* Add option to overwrite mileage in AssemblyInfo form.
* Freeze version of browserify. Newer version breaks shims in dependent libraries.
* Prohibit redirection for POST requests to FMS.
* Freeze version of pyotp.
* Merge branch 'backup-import-mode' into 'master'

  * Allow unmatched param for tt import.
  * Update ui to handle for diffrent import mode.
  * Update form and view for import mode.

* Increase path speed limits.
* Merge branch 'forward-fix-bionic'

  * Vary the installer disk identifier for Trusty and Bionic.
  * Upgrade to node v10 and remove npm dependency which is included by default.
  * Relocate enum module to prevent module name collision.
  * Add "ifup lo" command required in Ubuntu Bionic.
  * Specify HostKeyAlgorithm in ssh config. Ubuntu Bionic disables ssh-dss by default.

* Merge branch 'path-planner'

  * Add costmap display on live map.
  * Add animation for planned path.
  * Display planned path on live map.

* Merge branch 'preventive-maintenance'

  * Add maintenance notification.
  * Add preventive maintenance form.
  * Add one-time password algorithm.

* Merge branch 'node-consolidation'

  * Update shutdown method to call executor's service.
  * Add access to the alternate tracked or trackless database.
  * Update to use nav2d_scan_topics param.
  * Adapt robot config form to choose the mobile robot provider.
  * Adapt the sytem info display based on the new HardwareInfo msg.
  * Move python-imaging dependency from agv05x_bringup to here.
  * Move topics and msgs into agv05 namespace.

* Improve parameter enum form to display the description.
* Add custom log download on api and remove custom log download view.
* Merge branch 'laser-area-robot-origin' into 'master'

  * Add insert coordinate table row.
  * Add drag region option.
  * Allow simple polygon and remove fixed point to origin.
  * Remove scan angle.
  * Add migration from local to global coordinate.
  * Remove current pose from metadata and update laser frame to become list of laser frame id.
  * Rename template and static file, remove unused file and fix bootstrap message not shown on page.
  * Merge laser sensor pages to single editor.

* Merge branch 'relocate-debian-patch'

    Relocate most of the debian patches (except rules.em) from gbp repo.
  * Create specific MySQL user account for better compatibility.
  * Fix possible error in package removal script.
  * Fix mysql configuration not being applied.
  * Use single quote for Django's secret key.
  * Allow usbmount to mount NTFS filesystems.
  * Add group-write permission for media upload folder.
  * Add package maintainer scripts.

* Merge branch 'forward-fix-bionic'

  * Prevent npm modifying npm entry in package.json.
  * Apply some forward fix for ubuntu bionic.
  * Update requirements.txt.

* Merge branch 'cloud-vm-fix'

  * Allow setting FORCE_SCRIPT_NAME in the environment.

* Override media_root parameter when restoring audio from backup file. Fix `#120 <https://gitlab.com/dfautomation/product/navwiz/agv05_ui/-/issues/120>`_.
* Create specific MySQL user account for better compatibility.
* Freeze version of django-cleanup.
* Contributors: Farhan Mustar, Patrick Chin, Tang Swee Ho

1.14.1 (2020-06-03)
-------------------
* Add server clock in UserPanel.
* Speed up task template rendering using webassembly GraphViz.
* Remove django cors header and add '--disable-web-security' flag on kiosk instead.
* Fix missing unregistration of reflector subscriber.
* Contributors: Patrick Chin

1.14.0 (2020-05-15)
-------------------
* Merge branch 'advanced-backup-restore' into 'master' (`!34 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/34>`_)

  * Fix can backup empty data, fix display empty data and sort for variable and parameter.
  * Add advanced backup restore mode.

* Merge branch 'task-triggers' into 'master' (`!33 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/33>`_)

  * Fix cannot submit form FMS mode if task triggers form have error.
  * Add task trigger field in agv and remove auto homing field in agv.
  * Add extra AGV executor settings (min. battery level to trigger auto-homing).

* Merge branch 'alt-param-display' into 'master' (`!32 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/32>`_)

  * Add alternative display for tasks-completed with only boolean parameters.
  * Add alternative display for completed tasks with only boolean parameters.

* Merge branch 'mileage-report'

  * Include mileage and task counter in agv activity report.
  * Handle requests exception.
  * Improve tick interval on duration axis in report view.

* Merge branch 'min-battery'
  * Refactor to read manual charging state from battery_state topic.
  * Add min_battery_level setting.

* Add wheel-slippage error code.
* Merge branch 'oee-sync'

  * Update style.
  * Add task to sync agv statistic to FMS.
  * Rename lock id.
  * Split the agv activity archive to separate folder for trackless mode.

* Merge branch 'generate-self-signed-certificate'

  * Allow generating private key.
  * Fix cert file permissions.

* Add "submit and continue editing" button for parameters.
* Merge branch 'task-runner-improvement'

  * Allow starting task runner in paused state.

* Merge branch 'oee' into 'master'
  Overall Equipment Effectiveness (OEE) (`!30 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/30>`_)

  * Add downtime activity tracker.
  * Add FMS status text in AGV API.
  * Allow editing activity and auto-update stats.
  * Minor fix.
  * Add configuration for activity labelling.
  * Add AGV activity report.
  * Add DB models for AGV activity.
  * Add error code definitions.

* Merge branch 'optimization'

  * Reduce CELERYD_MAX_TASKS_PER_CHILD.
  * Move some transient cache from DB to Redis to improve performance.

* Fix timezone bug for task statistics.
* Ensure task statistic update before archive. Refactor code.
* Keep at least 10 completed tasks unarchived and refactor code.
* Freeze version of ujson as newer ujson removes serialization of iterables.
  See https://github.com/ultrajson/ultrajson/commit/53f85b1bd6e4f27a3e4cdc605518c48a6c7e7e9e
* Merge branch 'white-label' into 'master' (`!29 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/29>`_)

  * Add favicon and logo api view and update html for new image url.
  * Update webserver and webapp to display address and copyright label from database.
  * Add white-label config view, REST API and service on angularjs.

* Merge branch 'generate-self-signed-certificate'

  * Add option to generate self signed certificate for https network settings.

* Merge branch 'reflectors' into 'master' (`!28 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/28>`_)

  * Add landmark dragging capability and change landmark hover behaviour.
  * Add landmark input to map layout editor
  * Display reflectors data and landmarks in live map.

* Merge branch 'charging-status-api'

  * Add charging status in AGV API.

* Merge branch 'https-network' into 'master' (`!27 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/27>`_)

  * Add HTTPS setting to agv.
  * Supress warnings due to unverified HTTPS request.
  * Allow request to fms without verification.

* Merge branch 'mileage-api'

  * Add mileage in AGV API.

* Merge branch 'validate-rfid-after-teleport'

  * Add option to validate rfid after teleport.

* Merge branch 'forward-flag-input-for-teleport'

  * Add forward flag input as teleport parameter.

* Merge branch 'config-hostname' into 'master' (`!26 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/26>`_)

  * Add initial hostname form data and add back button.
  * Add hostname configuration page.

* Merge branch 'backup-timestamp-fix'

  * Fix timestamp in backup content wrongly shifted by timezone.

* Merge branch 'task-template-api'

  * Fix MapSerializer.
  * Add REST API for active task templates.

* Merge branch 'error-code'

  * Add error code in AGV API.

* Merge branch 'map-params'

  * Prevent concurrent editing of map parameters.
  * Add map parameter editor.
  * Update map layout editor to support map parameters.
  * Change speed unit in map editor to meter/sec.
  * Add customizable path speed levels for map editor. Fix #102.

* Fix regression bug, where the title info is missing.
* Merge branch 'allowed-motion'

  * Use a newer version of networkx library from pip.
  * Update description in allowed motion choices.
  * Add extra AGV executor settings (allowed_motions).

* Contributors: Farhan Mustar, Patrick Chin, Tang Swee Ho, Xin Wei Lee

1.13.16 (2020-01-13)
--------------------
* Add low disk space alert. (`!24 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/24>`_)
* Add webhook for agv and task events.
* Prevent concurrent edits for task template, global parameters, raw map, map layout and teleport. (`!23 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/23>`_)
* Improve npm_install.sh to include symlinked files and dirs.
* Add indent to nested task column options for clarity.
* Contributors: Amirul Nazmi, Farhan Mustar, Patrick Chin

1.13.15 (2019-11-22)
--------------------
* Detect as source version if there is workspace overlay.
* Merge branch 'per-task-diagnostic' into 'master' (`!22 <http://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/22>`_)

  * Add per-task diagnostic download.

* Merge branch 'custom-log'

  * Change custom log download to csv format.
  * Fix time display in wrong timezone in custom log download.
  * Allow custom log download.

* Merge branch 'task-columns'.

  * Add all columns to exported task csv.
  * Ensure cancelled task has end time and no start time.
  * Allow configuration of task columns display in completed tasks page.

* Merge branch 'simplify-robot-config'.

  * Move robotcfg from database to environment default file.
    Remove set_robot service.
  * Move robotcfg form fields to Django from JS.
  * Handle robot config import and export in frontend.

* Increase database commit interval of celery beat scheduler.
* Merge branch 'tf'

  * Throttle laser scan in tandem with pose update.
  * Strip leading slash from tf's frame_id.
  * Optimize the buffer period of tf listener.

* Fix form with hidden error will never submit.
* Update scan topic in mapx channel to follow env_var settings.
* Use range_max from laser as limit in display.
* Merge branch 'license-agv' into 'master' (`!16 <https://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/16>`_)

  * Add license validation for NavWiz software

* Merge branch 'wifi-status-topic'

  * Update notification channel and wifi REST API for wifi status topic.

* Freeze version of django rest framework.
* Merge branch 'wifi-strength-indicator', close #123.

  * Update notifcation to display wifi icon based on signal level.

* Merge branch 'laser-scan-dynamic-frame-id' into 'master' (`!15 <https://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/15>`_)

  * Update mapx channel to support multiple frameId.
  * Update laser channel to support multiple frameId.
  * Allow up to 3 laser sources in laser area editor.
  * Display only scan topic which is related to localization in map.

* Merge branch 'network-configuration-update'

  * Update network view to not configure preset_network and remove extra_options.

* Merge branch 'software-update-interface'

  * Redirect stderr to log file.
  * Suppress prompt for overwriting config file.

* Merge branch '121-teleport-ui-versioned-param' into 'master' (`!14 <https://gitlab.com/dfautomation/product/navwiz/agv05_ui/merge_requests/14>`_)

  * Fix teleport form to handle versioned action param correctly.

* Contributors: Amirul Nazmi, Farhan, Farhan Mustar, Muhammad Farhan Mustar, Patrick Chin

1.13.14 (2019-05-27)
--------------------
* Implement software update through USB pendrive.
* Enable configurable robot type.

  * Add debug option.
  * Fix autoneg off will never up since it is not configured and ifplugd cannot listen to link-beat.
  * Allow manual input for bringup provider if list not available and fix error not display properly.
  * Add download and upload option for robot config.
  * Remove preset config.
  * Add network extra option and handle in network view.
  * Show confirmation before set robot.
  * Add robot configuration page.
  * Update network config using robot configurator data.
  * Store robot_cfg in cache.

* Set database CONN_MAX_AGE to zero (default). Fix #118.
* Improve npm_install script.

  * Make npm_install intelligently choose to run npm install or gulp build.
  * Fix npm_install not running although the build is outdated.
    This happens after manual build directly using npm or gulp.
  * Prevent npm_install.sh from failing silently.

* Add `About` section within `Help` page, and remove help content upload form.
* Enable dynamic discovery of obstacle sensor providers.

  * Add service source for dyncfg parameters, and show missing options for multi-choice dr.
  * Add topic method on supplying enum option for dyncfg parameters.

* Allow user to extend auto-poweroff countdown on user panel. Fix #30.
* Amend css because autoprefixer no longer process `pixelated` value.
* Fix svg rendering bug in Firefox 66 and above.
* Add dns-nameserver field in wifi static ip configuration.
* Add missing build dependencies.
* Fix laser flicker on live view and add manual control rotation, fix #119.
* Implement amcl particle cloud visualizer (in debug mode).
* Contributors: Farhan Mustar, Patrick Chin

1.13.13 (2019-03-24)
--------------------
* Fix for debian packaging:
  * Add missing dep on `mysql-server` package.
  * Prepend sudo to supervisorctl command.
  * Add related debian configuration files.
  * Add mysql config file for installation.

* Fix for demo vm hosting:
  * Fix log url when django_script_name is not at root path.
  * Ensure correct sockjsUrl when django_script_name is not at root path.
  * Make timezone settings more foolproof.

* Update and display change in laser pose immediately.
* Add network configuration page for LAN interfaces.

* Fix for debian packaging:
  * Fix broken package from pip install.
  * Add script to setup database.
  * Improve setup.py.
  * Remove collectstatic and database setup operation from build.
  * Improve build to be independent of global npm version.
  * Migrate to package.xml version 3 and add copyright file.
  * Install pip dependencies automatically.
  * Fix version of gulp-sass to stablise npm install process.
  * Add empty media folder to repo.
  * Accept secret_key and debug flag from environment variables.
  * Remove unused librabbitmq library.
  * Add gunicorn as production dep.
  * Update robohash library to a strip-down release.
  * Move debinterface dependencies to the linux_wifi package itself.
  * Remove unnecessary --editable flag in requirements.txt.
  * Fix version conflict for pip nested dependency.
  * Limit version of mysqlclient library.

* Add cookie prefix.
* Fix bare except statement which suppress KeyboardInterrupt.
* Indicate continuity of selected path or station in trackless map editor. Fix #64.
* Keep selection when clicking on blank area in map editor.
* Contributors: Farhan, Patrick Chin

1.13.12 (2019-01-21)
--------------------
* Add system-monitor view.
* Fix teleport not cloned properly. Fix #115.
* Implement raw map editor. Fix #57.
* Make the gray shades distinctive from black and white in the raw map's color palette.
* Fix tooltip for toolbar hidden when editor is in fullscreen mode.
* Fix bug raw map selection not updated during map changeset view.
* Fix map changeset display height.
* Fix shortcuts and ruler button not disabled when changeset view is active.
* Fix station search tool not appearing on first shortcut trigger.
* Contributors: Patrick Chin

1.13.11 (2019-01-12)
--------------------
* Fix time in success message of map layout not using local timezone.
* Limit the maximum undo levels.
* Improve title display for raw map pages.
* Improve title for map and task template cloning page.
* Allow `stopasgroup` flag on gunicorn.
* Contributors: Patrick Chin

1.13.10 (2018-11-30)
--------------------

1.13.9 (2018-11-29)
-------------------
* Change laser2 color and rearrange laser layer in live map.
* Fix shared throttler for map channel laser data.
* Add laser2 in live map.
* Make robot orientation follow the laser frame in laser area editor.
* Add configurable scan angle and laser frame in laser area editor.
* Contributors: Farhan, Patrick Chin, nik3

1.13.8 (2018-11-27)
-------------------
* Fix live laser data not being displayed in laser area editor.
* Add creation, start and completion time in task object returned by REST API.
* Contributors: Patrick Chin

1.13.7 (2018-10-20)
-------------------
* Add custom log viewer. Fix #114.
* Merge the bundle.js for tracked and trackless AGV.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------
* Allow draggable junctions for map editors. Fix #13.
* Implement measurement ruler in map editors.
* Add button to rotate station by 90 degrees for trackless map editor.
* Always show tooltip on toolbar by setting disabled attribute instead of disabled class.
* Remove underline when hovering over action block in Chrome.
* Fix zoom control hidden in Firefox.
* Update zoom buttons to zoom from center point of task template.
* Zoom buttons in map editor zoom at current center point of map. Fix #65.
* Maintain previous link in action's first outcome after inserting action.
* Add proper tooltip for action blocks in task template editor.
* Add shortcut key Tab and Shift+Tab to switch between params in task template. Fix #107.
* Add copy and paste feature for task template editor.
* Add link to view sub tasks. Fix #32.
* Enable add action after certain outcome when selected. Fix #105.
* Move undo and redo buttons into sub-toolbar.
* Prevent triggering shortcut key when typing.
* Add shortcut key for lines and station in map editor. Fix #12.
* Add shortcut key for undo and redo.
* Add undo and redo feature for map editor. Fix #19.
* Add undo and redo feature for task template editor.
* Contributors: Patrick Chin, Xin Wei Lee

1.13.5 (2018-09-07)
-------------------
* Fix bug invalid path speed being accepted in map editor.
* Change title-info to skyblue background.
* Remove Zalpha text from AGV on live-map.
* Contributors: Patrick Chin

1.13.4 (2018-08-27)
-------------------
* Update HTTP error code in REST API.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------
* Allow user elevation in UserPanel.
* Fix build depends.
* Update dynamic_reconfigure node name to agv05x_lidar.
* Add dynamic reconfigure multiple choice app.
* Update laser area editor for multiple laser node.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, Xin Wei Lee, Zhe Xian Low <zx_low@hotmail.com>, farhan

1.13.2 (2018-05-18)
-------------------
* Improve map changeset and toolbar display in fullscreen mode.
* Make global param default to int type when added.
* Update power topics and messages.
* Display title info in fullscreen mode for map and task template editor. Fix #92
* Contributors: Patrick Chin <patrickcjh@gmail.com>, Xin Wei Lee

1.13.1 (2018-05-08)
-------------------
* Update the agv core node used for shutdown.
* Limit browser-sync version due to incompatibility with nodejs version.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.0 (2018-04-20)
-------------------
* Add default app option and consolidate options into executor_cfg variable.
* Add timeout configuration for task runner default-init.
* Refactor code for show-modal.
* Refactor code and move popup handling into agv05_executor node.
* Add permission for pausing and resuming task runner.
* Suppress form validation error from toolbar numeric input. Fix #101.
* Change step increment to support two decimal places param.
* Add built-in options to default initialization choices.
* Include agv_uuid in the result of AGV status API.
* Add safety popup message for sensor_com_error.
* Include pre_init and default_init data in backup file.
* Add viewer for original (unsanitized) task template structure. Fix #95.
* Add versioning for skill param.
* Sanitize outdated action params.
* Add agv_mctrl run to do shutdown sequence for Suki.
* Add AGV version info in help tab. Fix #97.
* Add default-init task template for task runner.
* Refactor code for starting and stopping app throught REST API.
* Add pre init task template for webserver and webapp.
* Added new permission to start app, stop app, start task runner and stop task runner.
* Added api to start and stop app.
* Mute certain safety types.
* Fix custom init task template does not show initial data.
* Update custom init task template ui using bootstrap-select for multiple selection.
* Add calibration app.
* Group backup-restore of global_param with task template.
* Prevent teleport from using global params.
* Refactor code and improve station search. Fix #79.
* Add global param to backup restore variable.
* Add ctrl + f to find station in map and mapx editor.
* Add reserved global params and refactor.
* Add global parameter model and editor.
* Enforce maximum value for teleport distance cost.
* Add map clone feature.
* Add cache entry for fms skillset.
* Refactor to use skillset in cache instead of ros topic to match FMS implementation.
* Fix empty task template outcomes not validated.
* Add Restore from previous changeset for map editor, close #16.
* Add junction RFID feature in map editor.
* Rearrange pathbranch enum.
* Add PathBranch enum for plan skill to nav.
* Add branching feature in map editor.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, ammar95, farhan, kxlee, nikfaisal

1.12.4 (2018-03-12)
-------------------
* Restrict tornado version due to incompatibility with current Python version.
* Fix some attributes of task template not being copied.
* Fix wifi api error.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan

1.12.3 (2018-02-27)
-------------------
* Increase pagination limit for all list view to 100.
* Display all previous safety triggers during safety resume popup. Fix #80.
* Fix task template editor for input param number with undefined min max.
* Add safety message for wait_traffic.
* Update message enum for navigation failure.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan

1.12.2 (2018-02-23)
-------------------
* Fix package dependencies.
* Bug fix in wifi.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.12.1 (2018-01-03)
-------------------
* Fix typo in `Register` API response.
* Add active map api.
* Add WPA Enterprise encryption type for WiFi.
* Fix UI bug.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.12.0 (2017-12-03)
-------------------
* Animate agv motion on live map.
* Allow tasks to be created in suspended state and resumed later.
* Group various task triggers into change_task service.
* Allow aborting task independently on agv in case it is not found on FMS.
* Enable `cancel_task` and `prioritize_task` operation from AGV.
* Implement task abort with fms permission checking.
* Add `aborting` task status.
* Enable `create_task` operation from agv in FMS mode.
* Update task status enum.
* Limit task API to provide only create, list and retrieve operations.
* Force empty result when given invalid `status` filter in task API.
* Add `non-stop transition` field to teleport.
* Fix version of requests library.
* Move safety and user message retrieval for agv status API into executor.
* Add up to 6 fallback remote assistance server hosts.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.7 (2017-11-13)
-------------------
* Fix task creation by `agv_panel_pin_protected` user.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.6 (2017-11-06)
-------------------
* Fix django-filters version.
* Update api to use the get_agv_status service.
* Add junction id in map svg to ease identification.
* Add progress field and modify status enum in task.
* Display last safety trigger in the popup for safety resume. Fix #80.
* Freeze version of crispy-forms.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.11.4 (2017-10-19)
-------------------
* Add fallback remote assistance server hosts.
* Change static file storage backend.
* Fix CMakeLists.txt to allow one-step catkin build across whole workspace.
* Make optional for some field in agv form. Fix #84.
* Remove old audio files when restoring audio from backup file.
* Migrate task statistic from mongodb to mysql.
* Remove obsolete diagnostic mongodb-based download.
* Fix issue with node-sass dependencies.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.3 (2017-10-02)
-------------------
* Fix svg cascading rules issue after Firefox 56.0 fix.
  See https://developer.mozilla.org/en-US/docs/Web/SVG/Element/use#Notes and
  https://bugzilla.mozilla.org/show_bug.cgi?id=265894
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.2 (2017-09-26)
-------------------
* Remove old wifi page.
* Remove CsrfExemptSessionAuthentication.
* Improve boot loading UI.
* Add django-enumfields library.
* Fix Wifi Mac not showing.
* Include custom initialization settings in backup file.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.11.1 (2017-09-12)
-------------------
* Implement new wifi interface using wpa_supplicant.
* Set a limit on the number of users based on licensing requirement. Fix #42.
* Allow user to define custom permission groups. Fix #63.
* Add interface for generating auth token.
* Add API endpoint for retrieving AGV status and messages.
* Update API endpoint permissions.
* Add REST API endpoint for reading and updating registers.
* Allow task creation by specifying task template name in REST API.
* Append tracked or trackless type into version display.
* Update fms pairing task to enable map uploading from AGV to FMS.
* Keep only 100 most recent map changesets. See #16.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.11.0 (2017-08-22)
-------------------
* Add version retrieval from agv core node.
* Fix ambiguity in server-clock display format.
* Add parameter variable_storage_url.
* Add auto sync time for date time configuration.
* Workaround for Chrome overwriting the default value in datetime picker.
* Update clock to reflect server timezone.
* Fix bug in date-time settings.
* Disable task creation from AGV panel during FMS mode.
* Convert cached task templates from OrderedDict to list.
* Auto reload validation message when validation completes.
* Add live-endpoint for executor sync.
* Add periodic task for maintenance of fms pairing.
* Retrieve current executor info through Django cache instead of ROS topic.
* Add fms channel.
* Add AGV skillset display.
* Add hashicon library.
* Implement skillset identifier which encompass all the capabilities of the agv.
* Drop pika in favor of kombu.
* Add authorization header and update the data included in agv sync.
* Allow uuid regeneration for admin only.
* Block map and task template in FMS mode.
* Add page for regenerating UUID.
* Contributors: Nik Faisal, Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>

1.10.6 (2017-08-08)
-------------------
* Fix datetime to use datetimetcl.
* Add Customizable task template at task runner for RFID auto position reset.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>

1.10.5 (2017-07-21)
-------------------
* Add permission for date and time configuration.
* Add perform soft reboot after date and time change.
* Add date time menu for trackless.
* Update diagnostic menu link to serve from nginx static directory.
* Add new permission `change_agv`.
* Rename view and fix import error in trackless backup view.
* Add date and time setting page.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>

1.10.4 (2017-07-12)
-------------------
* Update Django timezone to follow system timezone.
* Improve sequence of generated filter choices.
* Add filters for task template listing.
* Include `category` field of task template in backup.
* Backup Restore update on trackless system.
* Move audio parameter together with audio backup.
* Add parameter display exclusion for audio player.
* Add backuprestore content analyzer before restoring backup file.
* Add numbering in teleport table.
* Make `autoResetAgvPosition` enabled by default in new teleport.
* Fix bug - teleport serialized without skillId when action is empty.
* Fix compatibility with previous backup files.
* Change default distance_cost for teleport.
* Backup teleport data.
* Add label for category field.
* Add option to backup and restore settings.
* Add user interface for map teleportation.
* Add MODELS_VERSION to cache.
* Fix glitch when pressing save icon on toolbar.
* Add `category` field to TaskTemplate. See #62.
* Add db_index for MapOcg `created` field.
* Temporarily remove backup form for single task template.
* Fix multi-map backup.
* Switch to ujson for performance.
* Remove bottleneck in backup download. Fix #71.
* Add chart to completed tasks report.
* Upgrade Django to version 1.11 (LTS).
* Add missing migration.
* Fix bug due to race condition when restoring parameter.
* Increase dynamic reconfigure client timeout.
* Remove the tab layout in task template list.
* Allow adding new corner by clicking the midpoint of the edge of laser area. Fix #51.
* Improve laser area validation.
* Fix cursor while dragging in laser area editor.
* Move submit button onto toolbar in task template editor.
* Fix station choices to populate from multi maps.
* Update signals to support multi map.
* Add missing cache invalidation.
* Add caching key 'teleports'.
* Partially add multimap support.
* Relocate map model to system app.
* Save the correct map in backup file. Fix #76.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.10.3 (2017-05-22)
-------------------
* Fix ineffective throttle.
* Add permission control for live map and status page. Fix #73.
* Ensure values saved by laser profile editor are integers. Fix #74.
* Implement live map for tracked agv. Close #54.
* Fix 'on_unsubscribe' function called on non-subscribed channels when closing a connection.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.10.2 (2017-05-08)
-------------------
* Temporarily disable audio backup in preparation for release.
* Add development mode module.
* Backup audio to use relative path.
* Add missing files for installation.
* Add popup message for action pause. #2
* Backup to include audio files. #44
* Disable auto poweroff when running in debug mode.
* Replace rospy logger with python's logger.
* Throttle pose error message.
* Fix throttling issues.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.10.1 (2017-04-14)
-------------------
* Trigger dyncfg server to update parameters during settings restore. Fix #59.
* Add response message for Wifi.
* Add wait for ROS_LOG_DIR.
* Fix ConditionMixin supressing other 'as_view' decorators.
  Specifically, the AjaxTemplateMixin on the 'Vary' header, and the skipping of
  permission check in PermissionRequiredMixin.
* Add total duration statistic and csv download for task report.
* Update STATICFILES_STORAGE to CachedStaticFilesStorage.
* Improve diagnostic download.
* Use default compress level of 9 for backup file.
* Increase wait_for_message timeout to 0.5s.
* Handle exception when syncing to FMS and skillset not available.
* Refactor code to change reference of "ccs" to "fms".
* Add agv_panel to allowed users by default for new task templates. Fix #21.
* Limit viz.js to version 1.5.1 as further version has issues.
* Speed up gulp by excluding heavy libraries from bundle. Fix #46.
* Better manage multiple streams in gulp task.
* Add clock in config panel.
* Put task template's allowed users in backup-restore, match by username. Fix #45.
* Fix to remove success message when settings-restore has errors.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.9.1 (2017-03-22)
------------------
* Complete html special chars escaping in task template editor. Fix #15.
* Fix display label of params and outcomes of user-defined task templates. See #15.
* Fix bug: No error message when fail to save task template due to missing skillsets.
* Move webserver log files into the '~/.ros/log/{run_id}' directory. Fix #28.
* Make webserver and sockserver have distinct node names.
* Fix bug: supervisor locked out of permissions page once after saving permissions.
* Contributors: Ikhwannudin503939, Patrick Chin <patrickcjh@gmail.com>

1.9.0 (2017-03-15)
------------------
* Diagnostics download page fix for agv05.
* Contributors: nikfaisal

1.8.0 (2017-03-04)
------------------
* Add button to open the network connection editor.
* Display wifi ssid in webapp wifi popup.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.7.0 (2017-03-02)
------------------
* Allow user mentions using '@' to keep a popup message private.
* Escape html character to prevent task template viz rendering crash.
* Reduce throttling of battery message to use it as heartbeat for network disconnection.
* Simplify template.
* Fix missing success message in backup & restore view.
* Add icon for manual charging and charging status
* Add perms 'show popup warning'.
* Fix missing navigator button for previous month in task report.
* Simplify css using sass.
* Apply ES6 arrow functions.
* Fix error on django-admin commands when current workspace is not sourced or initially built.
* Add diagnostics download page.
* Allow rotation of station heading in both direction in map editor.
* Upgrade laser profile editor.
* Requery laser pose whenever robot controller is up.
* Fix missing unregistration of subscribers.
* Implement backup & restore for trackless.
* Various fixes for laser profile editor.
* Fix bezier limit checking when dragging control points.
* Add safety popup for navX.
* Stream map layout into MapX channel.
* Enable map display in amcl mode.
* Auto stop mapping when robot stops running.
* Add start menu with alternative start options.
* Support hybrid tracked path in map editor.
* Add cache entry for OCG.
* Fix edge case in hit test.
* Add fullscreen mode for map and task template editor. Fix #1.
* Add trackless map editor.
* Fix bug where hover is not reenabled immediately after an edit.
* Fix Chrome bug in which svg station fill color is not rendered properly.
* Add link on active map to its layout editor.
* Separate out d3 custom behaviors for code reuse.
* Update zoom functions affected by axis orientation change.
* Add waitForTransform for laser frame when starting up map channel.
* Remove duplicated and premature action sanitization in task template editor.
* Add micro grid. Make more "map" code sharable between webserver and webapp.
* Refactor code to make "map" code sharable between webserver and webapp.
* Fix bug in audio view.
* Add API view for map changesets (trackless).
* Move map broadcast from executor to here to improve performance and code reuse.
* Rotate ocg map preview to the correct orientation.
* Save map png in a flat folder structure to avoid issue removing folders.
* Modify icon and tab arrangment.
* Change icons and fix text size.
* Make map changeset immutable in database.
* Update permission form.
* Add interface for displaying trackless map in backend.
* Trigger client-side validation on saving map editor form.
* Add package django-cleanup.
* Initial commit of laser profile editor.
* Improve tiny imperfection in the visuals of map grid.
* Allow development on port 3000 using gulp and browser-sync.
* Add wraptor library to provide throttle decorator.
* Remove viz.js from webapp to improve page load time.
* Throttle battery update through websocket.
* Bring sockjs channels to systemx app.
* Add process.env.TRACKLESS flag for javascript.
* Add trackless app "systemx".
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.6.0 (2016-12-01)
------------------
* Add uploader for audio player files.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* Rename MySQL database from 'test' into 'agv05'.
* Fix bug with duplicate statistics written into MongoDB.
* Implement sockjs stub on system side.
* Rename permissions and change default permissions. Fix #11.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.4.0 (2016-11-02)
------------------
* Fix agvccs sync url.
* Ensure app-based permission check always pass for superuser.
* Add more columns in task export file.
* Hide restore task template from non-admin user.
* Add reporting for completed tasks, i.e task statistics.
* Add django_mongokit integration.
* Add download page for archived tasks.
* Use isoformat for time directly for compatibility with Excel.
* Archive tasks into monthly files instead of daily files to reduce complexity.
* Add django-debug-toolbar package.
* Update branding.
* Fix sendfile version for compatibility with nginx<1.5.9.
  See http://nginx.org/en/CHANGES (now nginx expects escaped URIs in "X-Accel-Redirect" headers).
* Fix file upload permissions.
* Use django-sendfile for help file download.
* Add django-sendfile package.
* Fix continuous firing of validate_models service when syncing with ccs.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.3.0 (2016-10-17)
------------------
* Add production mode settings.
* Add TaskCompletedListView.
* Modify layout.
* Add automatic task archiving.
* Enforce missing permissions.
* Update branding.
* Allow overlapped stations on single junction.
* Enable more delicate snapping to junction and station.
* Prevent user creating path that is too short in map editor.
* Handle diagnostics_agg message which comes unordered.
* Prevent user from messing up with permission to change permission.
* Modify menu.
* Add diagnostics viewer.
* Add celery integration.
* Make agv05_webserver and agv05_sockserver a required process.
* Add db_auto_reconnect to the background app. Fix #6.
* Remove the requirement of running roscore during catkin_make.
* Enable groupings for dyncfg parameter form.
* Show task history in agv panel.
* Enforce task operation permissions.
* Add allowed_users field to TaskTemplate and filter call buttons accordingly.
* Implement authentication through websocket and enforce panel permisssions.
* Use ServiceProxy instead of ActionClient or Publisher within Django webserver.
* Fix dyncfg_client not cleaned up properly.
* Use wait_for_message instead of Subscriber within Django webserver.
* Add new permissions for panel and task operations.
* Implement permission reset to factory default.
* Change permissions view to tabbed layout.
* Persist task to database.
* Upgrade to Django 1.10.
* Add wifi configuration on front panel with on-screen keyboard.
* Fix map changeset time not displayed as localtime.
* Handle uncompressed backup file during restore.
* Remove str() because it is unnecessary and it breaks unicode.
* Invalidate models cache on django post_migrate signal.
* Refactor code to use the default PermissionRequiredMixin (since Django 1.9).
* Monkey patch GroupManager to return queryset of our desired ordering.
* Fix Chrome mixing cached content for ajax and non-ajax request.
* Bring in the improvement from CCS on AJAX form.
* Change task template ordering.
* Use datetime of 'aware' type.
* Allow setting a user as inactive.
* Update anon backend to inject a dummy anonymous User entry.
* Fix auto-homing settings can't validate when auto-homing task template is empty.
* Implement task template individual file backup restore.
* Move version control from text file to database.
* Avoid sending popup or safety trigger message when robot controller is not running.
* Change session cookie age.
* Allow user to upload help file.
* Fix PermissionRequiredMixin always fails for guest user.
* Override cookie and media settings of Django.
* Display error message when sync with AGVCCS fails.
* Add new variables for backup and restore.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.2.0 (2016-08-24)
------------------
* Add auto-homing configuration.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.1.0 (2016-07-28)
------------------
* Fix wifi connect manual button.
* Move version info display to backend instead of panel view.
* Contributors: nikfaisal

1.0.0 (2016-07-21)
------------------
* Enable wifi adhoc function in icon display.
* Use text file to store AGV05 hardware and sofware version.
* Fix server IP in agv sync.
* Add sync agv with server.
* Add server ip field for AGV.
* Add global param agv_home for task template.
* Remove task_template_metas cache.
* Add downloadables_md5 cache.
* Add agv uuid.
* Add remote assistance feature.
* Fix title not displayed and glitch in poweroff countdown.
* Ensure compatibility between py2 and py3.
* Improve catkin build by automating certain Django setup tasks.
* Fix bug with mark_dirty being fired multiple times.
* Add pin elevation in agv panel.
* Rearrange menu.
* Allow Del shortcut to take effect only if the current focus is not an input field.
* Handle popup for status_charger_connected.
* Include agv_name in backup and restore.
* Add agv_name field.
* Attempt fix for changes not applied after soft reboot.
* Add permission editor page.
* Fix changes not saved in panel pin change form.
* Change the concept of user level to permission group.
* Add form for changing agv panel pin.
* Add permissions for agv panel.
* More proper permission handling in APIView.
* Fix csrf error while starting or stopping robot controller.
* Add countdown of two minutes before power off.
* Add description in backup page.
* Prevent multiple validations fired during restore from file.
* Add required_permissions.
* Add backup & restore feature.
* Allow path removal if station is also attached to another path.
* Fix bugs: Changes not validated and applied after a delete operation.
* Add 2-second delay and countdown timer for safety resume.
* Add feature to clone existing task template.
* Change safety flag to use enum format.
* Add cache field for task_template_metas.
* Allow nesting task template as action.
* Add keyboard shortcut F2 for action linking in task template editor.
* Round battery percentage to every 5% in display.
* Add safety triggered and safety resume popup.
* Fix issue with setting boolean param's checkbox value.
* Fix issue with boolean param's checkbox.
* Fix issue with action removal.
* Add "clear all" button in map and task template editor.
* Fix issue selecting outcome of first action in task templated editor.
* Fix issue loading and saving first action in task template editor.
* Use tornado.ioloop.PeriodicCallback for firing at fixed intervals.
* Add 'Del' keyboard shortcut for map and task template editor.
* Fix bug: Outcome of start terminal not removed properly.
* Draw edge only if start terminal has valid outcome.
* Fix typo that prevents removal of action outcome.
* Extends power off function to shutdown embedded controller as well.
* Implement wifi manager.
* Modify path shape attribute enums.
* Add skill_descriptions_md5 and map attribute enums.
* Enable warning message display during robot start.
* Add multiple popup types and subscribe to topics that trigger its display.
* Limit presence of viz.js to speed up page load.
* Add gunicorn worker
* Sort station names prior to saving.
* Add atomic set_flag_if.
* Add AGV settings page. Set cache flag appropriately when saving.
* Change task template editor from blockly to viz.js.
* Update dyn server based on config_desc instead of the existing cfg because the config_desc can change.
* Add placeholder for Wifi settings page.
* Use database caching for persistence of cached data.
* Fix timestamp printing to localtime instead of UTC.
* Add viz.js, which is usable in future.
* Add volume and battery percentage to webapp
* Make roscore a requirement for running webserver.
* Fix slow babelify transform.
* Implement rebootable robot controller.
* Cleanup dynamic configure client when done. Catch exception when creating the client.
* Switch to simplejson for performance.
* Update pip requirements file.
* Change to mysql database even in development mode for database lock issue and performance reasons.
* Implement channels on top of sockjs protocol.
* Auto set dynamic_reconfigure_storage_backend based on Django's settings.
* Add sockjs server on django management commands.
* Add page for dynamic reconfigure parameters.
* Update SuccessMessageMixin to cater for DeleteView.
* Add logging configuration.
* Update package metadata.
* Update arguments to init_node.
* Fix catkin_lint issues.
* Ensure unicode compatibility with Python 2 & 3.
* Add Django CORS Headers middleware library.
* Fix bug: Station displayed incorrectly on Chrome due to css not recognized within <use> tag.
* Modify ui control for changing station direction.
* Fix layout issue on Chrome browser.
* Fix bug: A mouse click and release at the same point constructs a path that links to the previous.
* Add task template editor component.
* Moved paginator into separate html template.
* Add PageTitleMixin and SuccessMessageMixin.
* Add map editor component.
* Touch up UserViews' permissions and urls.
* Add Django REST Framework library.
* Fix glyphicons font path.
* Update npm packages and allow ES6.
* Update roslint options.
* Startup django system app.
* Integrate browser-sync with django.
* Freeze django version for current development cycle.
* Add "system" icon link.
* Update npm build files for agv05_webserver package.
* Add agv05_webserver package.
* Contributors: Nik Faisal <nikfaisal05@gmail.com>, Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

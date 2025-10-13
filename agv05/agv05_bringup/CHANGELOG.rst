^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Move human_follower behind feature flag.
* Merge branch 'nav-omni' into 'master'

  * Add camera5.

* Merge branch 'safety-renesas' into 'master'

  * Add safety heartbeat publishing.

* Launch human_follower node.
* Contributors: Patrick Chin, Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Add diagnostic entry for agv05_modbus_client.
* Add agv05_modbus_client as dependency and launch agv05_modbus_client node.
* Merge branch 'hal' into 'master'

  * Add laser6.
  * Add quaternary and quinary lidar.

* Merge branch 'nav-improve-p3' into 'master'

  * Increase the possible update rate of costmap.

* Optimize planner by not publishing potential map.
* Contributors: Patrick Chin, Tan Kui An Andrew, Tan Wai Liang, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'nav-improve-p2-costmap'

  * Add support to laser5 in costmap.
  * Add selection of obstacle sensors which are used during Dynamic Path Planning.
  * Add option to change path border cost.

* Merge branch 'tracked-and-trackless-share-parameters'

  * Merge storage config of tracked and trackless mode.

* Merge branch 'improved-map-resolution-to-20mm'

  * Updated default map resolution of gmapping to 20mm.
  * Changed loop closure resolution according to map.
  * Changed cartographer resolution to 20mm.
  * Changed slam toolbox resolution to 20mm.

* Contributors: Andrew Tan, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Improve amcl default parameters.
* Contributors: Patrick Chin

2.1.0 (2021-11-24)
------------------
* Merge branch 'simulator-expansion'

  * Add simulator_diagnosics provider.
  * Rename stage_bringup provider to simulator_bringup.

* Merge branch 'path-planner' into 'master'

  * Add costmap parameters of obstacle_layer.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------
* Merge branch 'msb_embedded_process'

  * Disable agv05_line_sensor through parameter instead of envvar
    to prevent diagnostic stale.
  * Use junction_count for junction detection

* Merge branch 'path-planner'

  * Add handling to NavxAction "goal_tolerance".
  * Add new costmap plugin layer of static pre-planned path and forbidden zone.
  * Publish only full costmap.
  * Add planner.
  * Add costmap.

* Merge branch 'merge-agv05x-repo' into 'master'

  * Import agv05_stage and merge launch files from agv05x repo.
  * Import agv05_navx package from agv05x repo.
  * Import agv05_description package from agv05x repo.
  * Import agv05_lidar package from agv05x repo.

* Contributors: Farhan Mustar, Muhammad Syafiq Ramli, Patrick Chin, Wong Tze Lin

1.14.2 (2020-10-19)
-------------------
* Merge branch 'node-consolidation'

  * Reorganize launch files.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Add diagnostic entry for executor node.
* Contributors: Patrick Chin

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Merge branch 'cpu_status_update' into 'master'. (`!10 <https://gitlab.com/dfautomation/product/navwiz/agv05/merge_requests/10>`_)

  * Add CPU status logging in diagnostic.

* Contributors: Khairul Dhinie Kamaruzaman, Patrick Chin

1.13.14 (2019-05-21)
--------------------
* Add dead zone for zalpha joy node.
* Contributors: Farhan Mustar

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
* Remove launch option to disable wifi manager diagnostic.
* Update launcher to enable wifi manager diagnostic by default.
* Contributors: Farhan, Patrick Chin

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------
* Fix message type mismatch in led_low_batt topic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------
* Exclude wifi manager from diagnostic unless enabled explicitly.
* Update package meta and launch files.
* Add agv_r2100_obstacle_sensor for PGV AGV
* Update launch file for agv_obstacle_sensor changes.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan, ziyang

1.13.2 (2018-05-18)
-------------------
* Move agv05_description package into agv05x repo.
* Merge with Node Consolidation branch.
* Update motor topics.
* Merge panel topics. Remove the now-obsolete agv05_translator.
* Remove deprecated agv05_core and agv05_driver packages.
* Rename agv05_nav into agv_nav_track.
* Add wifi_manager into diagnostics.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-06)
-------------------

1.12.4 (2018-03-27)
-------------------
* Make trackless laser to be on top by default.
* Make agv_rfid node optional in launch file.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.12.3 (2018-02-27)
-------------------
* Update modbus server to try to restart if port in use.
* Contributors: farhan

1.12.2 (2018-02-23)
-------------------

1.12.1 (2018-01-03)
-------------------
* Reorder diagnostic entry.
* Add configuration for laser positioned on top.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.6 (2017-11-06)
-------------------

1.11.5 (2017-10-24)
-------------------

1.11.4 (2017-10-19)
-------------------

1.11.3 (2017-09-29)
-------------------

1.11.0 (2017-08-22)
-------------------
* agv05_bringup: update diagnostic with new core
* agv05_bringup: add new node agv_core_zalpha to replace agv_core, add new agv_rfid node
* Add agv05_variable_storage package.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.10.6 (2017-07-27)
-------------------
* agv05_remote_io: add in diagnostic information for request and reply
* Contributors: phtan

1.10.4 (2017-07-10)
-------------------

1.10.3 (2017-05-26)
-------------------
* agv05_bringup: add agv05_remote_io node into launch file
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal05 <nik@dfautomation.com>, phtan

1.10.2 (2017-05-06)
-------------------
* agv05_bringup: add remote io into package run depend
* Contributors: phtan

1.10.1 (2017-04-13)
-------------------

1.10.0 (2017-03-30)
-------------------

1.7.0 (2017-03-01)
------------------
* Add libsensors_monitor package to log CPU temperature data.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.6.0 (2016-11-30)
------------------
* Improve audio player to play from different playlist.
* Sort diagnostic params alphabetically.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* Discard joystick diagnostics in aggregator.
* Remap odom topic.
* agv05_bringup: add in all remaining nodes 's diagnostic updater
* agv05_bringup: add in led diagnostic updater
* agv05_bringup: add in diagnostic updater for line sensor
* agv05_bringup: add expansion IO and laser sensor to the diagnostic publisher
* Update package meta.
* Add parameter to choose whether to run I/O-based laser sensor.
* Fix problem fix velocity not moving
* Add agv05_joy node, dependencies
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.4.0 (2016-10-31)
------------------
* Add diagnostic_recorder.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.3.0 (2016-10-16)
------------------
* agv05_bringup: add change node launching to the agv05_power_management node.
* agv05_bringup: add in agv05_pgv_line_sensor
* Add example for diagnostic logging.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.2.0 (2016-08-25)
------------------
* Update package metadata.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.1.0 (2016-08-04)
------------------

1.0.0 (2016-07-21)
------------------
* agv05_bringup: add new node for the launch file
* Update dynamic_reconfigure_storage plugin notation to all dots.
* Update launch files.
* agv05_bringup, add in agv05_laser_sensor and agv05_nav for auto launching
* added panel button & led control (agv05/peripharel/panel/led_error => 0 always off, >200 always on, >100 delay after end pulse, tenth led on time, digit number of pulse)
* rewrite audio player (play all mp3 files in music playlist; set play, pause, stop, volume, filename)
* Make agv05_bringup use dynamic_reconfigure_file_storage backend.
* Fix catkin_lint issues.
* Add agv05_bringup, agv05_capabilities, and agv05_description packages.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

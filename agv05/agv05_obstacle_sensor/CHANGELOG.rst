^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_obstacle_sensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Add heartbeat monitoring on obstacle sensors.
  * Add navigation lasers into lidar inspector watchdog list.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Merge branch 'obstacle-popup-spam' into 'master'

  * Fix Obstacle Blocked Pop-Up Spamming, retain error message for near block 2s timeout.

* Remove header files from debian package.
* Merge branch 'nav-improve-p3-safety-area' into 'master'

  * Different safety area for different safety block speed range.

* Merge branch 'nav-improve-p3-omni-dock' into 'master'

  * Add safety area for omni motion.

* Merge branch 'obstacle-sensor-group-lidar-name' into 'master'

  * List the obstacle sensors of each group in diagnostic status.

* Fix roslint warning.
* Merge branch 'nav-improve-p3-dim' into 'master'

  * Move robot radius and margin to AGV dimension profiles.

* Contributors: Patrick Chin, Tan Wai Liang, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'update-obstacle-sensor-default'

  * Update obstacle sensor manual control default.

* Fix "Set Parameters" skill failed to set parameters of agv05_obstacle_sensor.
* Merge branch 'obstacle-sensor-group-c'

  * Default Group A for lasers, Group B for embedded 1 and Group C for embedded 2.
  * Add obstacle sensor group C.

* Merge branch 'fix-parameter-update'

  * Nested tabs for obstacle sensors group.
  * Fix parameter reset failed with agv05_navx and agv05_obstacle_sensor.

* Merge branch 'rename-obstacle-sensor-parameter'

  * Rename front and rear sensors to A and B.

* Filter front and rear obstacle sensor list for duplicates.
* Contributors: Andrew Tan, Farhan Mustar, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Merge branch 'update-manual-control-obstacle-block-detection'

  * update manual control obstacle block detection default area

* Contributors: Muhammad Syafiq Ramli

2.1.0 (2021-11-24)
------------------
* Merge branch 'path-planner' into 'master'

  * Add parameter inflation margin for additional clearance in between obstacles and dynamic path if there is enough space.

* Merge branch 'path-planner' into 'master'

  * Add robot radius configuration.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------
* Refactor code.
* Reduce frequency diagnostic averaging window to 1s.
* Merge branch 'obstacle-hint'

  * Add obstacle hint to diagnostic.
  * Update agv05_obstacle_sensor to provide hint when near block or malfunction.

* Merge branch 'path-planner'

  * Update obstacle sensor profile handling for dynamic forward and reverse. Refactor code.

* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin

1.14.2 (2020-10-19)
-------------------
* Merge branch 'node-consolidation'

  * Move agv_obstacle_sensor into agv05 namespace.

* Sort obstacle sensor list.
* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Add laser profile area for manual control.
* Contributors: Patrick Chin

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Increase number of laser profiles to 10.
* Contributors: Patrick Chin

1.13.14 (2019-05-21)
--------------------
* Enable dynamic discovery of obstacle sensor providers.

  * Update obstacle provider topic name.
  * Add service to enumerate provider sources.
  * Update agv_obstacle_sensor to scan sensor list using topics.
  * Update agv_obstacle_sensor obstacle list param to use topic enum_src method.

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
* Refactor code. Use ObstacleSensor message.
* add in agv05_variable_storage dependency into agv_obstacle_sensor package to fix build error.
* Update package meta and launch files.
* Add agv_r2100_obstacle_sensor for PGV AGV
* Add 2 second timeout for near_activation and malfunction of agv_obstacle_sensor.
* Rename obstacle topic name.
* Update agv_obstacle_sensor to build sensor options from pkg manifest.
* Fix memory error due to vector resize.
* Fix agv_obstacle_sensor function prototype parameter type.
* Fix agv_obstacle_sensor subs and pubs.
* Update agv_obstacle_sensor for multiple laser and update ll300 laser topic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan, nikfaisal, phtan, ziyang

1.13.2 (2018-05-18)
-------------------
* Merge obstacle sensor topics and messages.
* agv_obstacle_sensor: Update msgs.
* Update obstacle sensor config to allow profile and area selection.
* Update node frequency.
* agv_obstacle_sensor: add more action feedback.
* agv_safety: Remove sensor area control.
* agv_obstacle_sensor: Split front and rear area control for profile.
* agv_obstacle_sensor : Add laser control based on profile.
* agv_obstacle_sensor : Add config for laser profile.
* agv_obstacle_sensor: agv_obstacle_sensor node file commit.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

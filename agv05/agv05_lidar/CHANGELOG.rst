^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_lidar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Add heartbeat monitoring on obstacle sensors.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Remove header files from debian package.
* Merge branch 'hal' into 'master'

  * Add quaternary and quinary lidar.

* Merge branch 'const-msg-callback' into 'master'

  * Use const type in callback to avoid copy overhead.

* Contributors: Patrick Chin, Tan Wai Liang, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'protected-parameter'

  * Suffix lidar config with underscore.

* Merge branch 'nav-improve-p2-costmap'

  * Add selection of obstacle sensors which are used during Dynamic Path Planning.

* Contributors: Farhan Mustar, Wong Tze Lin

2.2.0 (2022-06-30)
------------------
* Merge branch 'update-manual-control-obstacle-block-detection'

  * update manual control obstacle block detection default area

* Contributors: Muhammad Syafiq Ramli

2.1.0 (2021-11-24)
------------------
* Merge branch 'stage-fix'

  * Fix tf not received in Stage simulation mode.

* Contributors: Patrick Chin

2.0.0 (2021-05-24)
------------------
* Merge branch 'lidar-config-min-activation'

  * Update max limit to 100 for min activation.
  * Add min_activation to agv05_lidar dynamic config.

* Merge branch 'obstacle-hint'

  * Add activation hint in diagnostic.
  * Add hint parameter for obstacle sensor msg and update agv05_lidar to publish topic hint if lidar near block or mulfunction.
  * Fix not using private nodehandle.
  * Update agv05_lidar to accept lidar name parameter.

* Merge branch 'lidar-debug'

  * Add more lidar hit test.
  * Add more computeHit test and fix parallel border hit.
  * Fix computeHit for failing test.
  * Add computeHit test.
  * Refactor code for unit test and add test lidar transformation.

* Merge branch 'merge-agv05x-repo' into 'master'

  * Import agv05_stage and merge launch files from agv05x repo.
  * Import agv05_lidar package from agv05x repo.

* Merge branch 'lidar-bug-fix'

  * Fix compute hit param pass by reference and fix area valid must be 3 point or more.
  * Change lidar msg to use LaseScanPtr type.
  * Fix bug agv05x_lidar trigger transform error.

* Contributors: Farhan Mustar, Patrick Chin

1.14.2 (2020-10-20)
-------------------
* Merge branch 'single-lidar-node' into 'master'

  * Add 3 lidar inspector, update config parameter and add diagnostic message.
  * Add param input for inspector configuration and add init method.
  * Refactor agv05x_lidar.

* Merge branch 'node-consolidation'

  * Move topics and msgs from agv to agv05 namespace.

* Merge branch 'laser-area-robot-origin' into 'master'

  * Add secondary lidar option.
  * Update compute hit to check is inside polygon.
  * Update default area parameter for area in robot origin.

* Contributors: Farhan Mustar, Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-15)
-------------------

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-22)
--------------------

1.13.14 (2019-05-15)
--------------------
* Update agv05x_lidar topic name and launcher to enable dynamic discovery of obstacle sensor providers.
* Contributors: Farhan Mustar

1.13.13 (2019-04-05)
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
* Remove agv05x_lidar frame_id filter.
* Update lidar for new obstacle entry and add second lidar on qube launcher.
* Update agv05x_lidar for configurable frame_id.
* Contributors: Farhan, Patrick Chin

1.13.8 (2018-11-27)
-------------------

1.13.7 (2018-10-19)
-------------------

1.13.6 (2018-09-29)
-------------------

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------
* Update agv05x_lidar to become an obstacle input to agv_obstacle_sensor node.
* Replace agv05x_laser_sensor with agv05x_lidar.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-23)
-------------------

1.12.4 (2018-03-12)
-------------------

1.12.3 (2018-02-28)
-------------------

1.12.2 (2017-12-30)
-------------------
* Update CMakeLists.txt.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.11.4 (2017-10-09)
-------------------

1.11.3 (2017-09-30)
-------------------

1.11.0 (2017-08-14)
-------------------
* Apply 2-second timeout prior to clearing the malfunction and near-blocked status.
* Publish laser malfunction status when profile parameter is incorrect.
* Switch laser profile stored coordinates to follow forward(X)-left(Y)-up(Z) convention.
* Implement laser sensor node.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

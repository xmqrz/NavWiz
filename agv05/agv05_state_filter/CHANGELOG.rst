^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_state_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Fix missing heartbeat update during ICP matching error.
  * Add safety heartbeat publishing.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Merge branch 'nav-improve-p3' into 'master'

  * Refactor code.

* Fix return-type warnings.
* Separate state filter base parameters from CSM group.
* Contributors: Farhan Mustar, Wong Tze Lin

3.0.0 (2023-08-01)
------------------

2.2.0 (2022-06-30)
------------------
* Merge branch 'odom-imu' into 'master'

  * When base class's constructor calls a virtual function on its this object, derived class's override of that virtual function will not get invoked.
  * Original odom TF topic is renamed into odom/wheel TF topic while new odom TF topic will be product of IMU data fusion and is compared with odom/wheel.

* Contributors: Patrick Chin, Wong Tze Lin

2.1.0 (2021-11-24)
------------------
* Merge branch 'state-filter-improvement' into 'master'

  * Cache odom and laser scan data even if "SCAN SEEMS TOO OLD" happens.
  * Add publishing of odometry from laser scan data.
  * Error rate is calculated by using individual period of time to avoid huge error if there is a delay in calling callback function.
  * Add changeable parameter of output inlier correspondence threshold percentage as LEAN only has more than 30% of scan data being blocked by robot body.
  * Split class StateFilter into separated classes to handle multiple scan topics.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------
* Merge branch 'merge-agv05x-repo' into 'master'

  * Import agv05_state_filter package from agv05x repo.

* Contributors: Patrick Chin

1.14.2 (2020-10-20)
-------------------
* Merge branch 'node-consolidation'

  * Move topics and msgs from agv to agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-15)
-------------------
* Add agv05x_state_filter package to detect wheel slippage.
* Contributors: Patrick Chin

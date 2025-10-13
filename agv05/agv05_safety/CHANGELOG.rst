^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_safety
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.
  * Add system error message into diagnostic log.
  * Add safety heartbeat monitoring.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Merge branch 'system-overload'

  * Reduce time jump tolerance.
  * Publish safety trigger for system overload conditions.

* Fix noetic-roslint warning.
* Fix roslint warning.
* Contributors: Patrick Chin, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'time-jump-safety-trigger'

  * Move time jump detection to safety node.
  * Trigger safety when time jump happens.

* Contributors: Andrew Tan, Patrick Chin

2.2.0 (2022-06-30)
------------------
* Merge branch 'external-safety-message'

  * Add external safety message to diagnostic.
  * Modify external safety parameters to accept error messages.

* Contributors: Farhan Mustar

2.1.0 (2021-11-24)
------------------

2.0.0 (2021-05-24)
------------------
* Reduce frequency diagnostic averaging window to 1s.
* Hold AGV safety triggered status for diagnostic update.
* Contributors: Patrick Chin, Wong Tze Lin

1.14.2 (2020-10-19)
-------------------
* Remove safety_internal_control topic.
* Merge branch 'node-consolidation'

  * Merge agv_safety and agv05_safety into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Add wheel-slippage related safety flag and nav action feedback.
* Contributors: Patrick Chin

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------

1.13.14 (2019-05-21)
--------------------
* Remove redundant safety braking feature.
* Change motor overload to motor fault.
* Handle motor overload safety trigger.
* Contributors: Patrick Chin

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
* Fix enable parameter for safety V IO.
* batt1_swap relay act as voltage IO safety cut off.
* Contributors: nik3

1.13.9 (2018-11-29)
-------------------

1.13.8 (2018-11-27)
-------------------

1.13.7 (2018-10-19)
-------------------

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------
* agv05_safety: handling for  additional safety_io_1 and safety_io_2
* agv05_safety: improve message for configuration.
* agv05_safety: add subscribe for safety_io_1 and safety_io_2, add publisher to safety_brake.
* agv05_safety: add in additional configuration to allow enable/disable nav safety and io safety to trigger safety trigger
* Publish safety_trigger flag as warning in diagnostic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.13.2 (2018-05-18)
-------------------
* Merge power topics and messages. Remove the now-obsolete agv_translator.
* Merge safety topics and messages. Flip definition of safety flags.
* Add dummy external safety 2.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.1 (2018-05-08)
-------------------

1.13.0 (2018-04-06)
-------------------

1.12.4 (2018-03-27)
-------------------

1.12.3 (2018-02-27)
-------------------

1.12.2 (2018-02-23)
-------------------

1.12.1 (2018-01-03)
-------------------

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

1.10.6 (2017-07-27)
-------------------

1.10.4 (2017-07-10)
-------------------

1.10.3 (2017-05-26)
-------------------

1.10.2 (2017-05-06)
-------------------

1.10.1 (2017-04-13)
-------------------

1.10.0 (2017-03-30)
-------------------

1.7.0 (2017-03-01)
------------------

1.6.0 (2016-11-30)
------------------

1.5.0 (2016-11-25)
------------------
* agv05_safety: add diagnostic updater
* Contributors: phtan

1.4.0 (2016-10-31)
------------------

1.3.0 (2016-10-16)
------------------

1.2.0 (2016-08-25)
------------------

1.1.0 (2016-08-04)
------------------

1.0.0 (2016-07-21)
------------------
* agv05_safety: set nav_trigger to be true by default. to allow hardware control after restart.
* agv05_msgs SafetyTrigger & agv05_safety: change safety trigger from charging_state_trigger to charger_connected_trigger,
  rename topic of subscribe from charging_status to charger_connected
* agv05_safety: subscribe to power_management charging_status, and will trigger safety if the agv is currently charging.
* agv05_safety:
  safety will not receive trigger from laser sensor,
  nav will instead trigger the "nav_trigger" to safety for the "software" safety control
* agv05_safety: add in comment to describe the functionality for each topic
* agv05_safety: laser_sensor true/false error fix
* agv05_safety: file commit for agv05_safety node, with new SafetyTrigger new message.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

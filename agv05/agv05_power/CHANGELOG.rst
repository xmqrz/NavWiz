^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_power
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------

3.1.0 (2024-10-16)
------------------
* Merge branch 'hal' into 'master'

  * Fix diagnostic battery low level is not synchronized with AGV setting.
  * HAL battery state diagram.

* Fix noetic-roslint warning.
* Fix roslint warning.
* Contributors: Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'zalpha-3.3.0'

  * Increase maximum value of charger_min_voltage for Titan with 48V battery system.

* Merge branch 'remove-unused-agv05power-parameters'

  * Remove unused agv05 power variables.
  * Remove unused agv05 power parameters.

* Merge branch 'state-update-while-charging-cable-connected'

  * Fix states not updated when manual charging cable connected without current.

* Contributors: Andrew Tan, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------
* Fix battery_percentage initial value from source.
* Contributors: Farhan Mustar

2.0.0 (2021-05-24)
------------------
* Fix typo and fix auto charger due to battery low will always stop immidiately.
* Contributors: Farhan Mustar

1.14.2 (2020-10-19)
-------------------
* Remove brake_relay_control topic and flag.
* Allow battery percentage overwrite from hardware source.
* Re-add function to disable charging.
* Merge branch 'node-consolidation'

  * Improve diagnostic display.
  * Merge agv05_power_management and agv_power into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Publish charging LED only when voltage detected.
* Fix bug agv_power turning off LED.
* Contributors: Patrick Chin

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------

1.13.14 (2019-05-21)
--------------------

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
* Add missing `install` directive in CMakeLists.txt files.
* Fix manual cable connected without power that cause false charging reading.Temporary Fix
* Fix manual charger indicator light still on when unplug.
* Add LED indicator for low power and charging.
* Publish low power indicator to led strip.
* Contributors: Patrick Chin, nik3

1.13.8 (2018-11-27)
-------------------

1.13.7 (2018-10-19)
-------------------
* Remove unnecessary CompositeDiagnosticTask and HeaderlessTopicDiagnostic.
  Use FrequencyStatus directly.
* Contributors: Patrick Chin

1.13.6 (2018-09-28)
-------------------
* Send shutdown signal to PC besides embedded board, when battery is low.
* Contributors: Patrick Chin

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------
* agv_power: add 1s delay for before enabling unbrake button after safety triggered.
* agv_power: add in control to disable brake release under agv_power node, this control state will be same as actuator power state.
* agv_power : Fix led low power to use LedControl.
* Fix message type mismatch in led_low_batt topic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal, phtan

1.13.3 (2018-07-16)
-------------------
* Fix wrong topic led low battery.
* Prevent shutdown when battery not detected, i.e running in VM.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.13.2 (2018-05-18)
-------------------
* Update node frequency.
* agv_power: Increase time for current checking.
* agv_power: use BatteryState topic from agv_msgs instead of agv05_msgs
* agv_power: enable automatically select the side of the auto charger. Allow user to edit the period for charger current detection.
* agv_core_zalpha & agv_power: added shutdown function for core, added better diagnostic for power
* agv_power: add shutdown feature when battery is <3% for more than 10min, add new publish topic to initiate shutdown at mainboard
* agv_power: improve processPercentage call method
* agv_power: add low power publish to agv_power
* agv_power: add diagnostic status for voltage and percentage
* agv_power: completed process state, completed draft agv_power without diagnostic
* agv_power: added manual charging state
* agv_power: update power node, added processes for percentage and state, incomplete
* agv_power: initial node file commit
* Contributors: nikfaisal, phtan

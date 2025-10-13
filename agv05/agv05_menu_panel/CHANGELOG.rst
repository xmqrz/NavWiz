^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_menu_panel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------

3.0.0 (2023-08-01)
------------------
* Merge branch 'time-jump-safety-trigger'

  * Trigger safety when time jump happens.

* Merge branch 'nav-improve-p2-turn'

  * Rename line sensor com error to line sensor error.

* Contributors: Andrew Tan, Farhan Mustar, Patrick Chin, Wong Tze Lin

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------
* Merge branch 'path-planner' into 'master'

  * Fix missing status string.

* Contributors: Patrick Chin, Wong Tze Lin

2.0.0 (2021-05-24)
------------------

1.14.2 (2020-10-19)
-------------------
* Merge branch 'node-consolidation'

  * Move agv_menu_panel into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------
* Add hold mode button to exit task runner.
* Add menu diagnostics.
* Contributors: Farhan Mustar, Nik

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Update agv_menu_panel to use wifi status topic.
* Contributors: Farhan Mustar

1.13.14 (2019-05-21)
--------------------

1.13.13 (2019-03-22)
--------------------
* Migrate to package.xml v3 and add copyright file.
* Contributors: Patrick Chin

1.13.12 (2019-01-21)
--------------------
* Reduce frequency for agv_menu_panel to 5.
* Contributors: nik3

1.13.11 (2019-01-12)
--------------------
* agv_menu_panel: Fix auto start task runner bug.
* add timeout for agv_io_adam and agv_mctrl
  standardize all timeout to 25ms, since MCTRL will have 20ms timeout
* Remove dependency on agv05x_msgs package.
* Contributors: Patrick Chin, nik3, phtan

1.13.10 (2018-11-30)
--------------------

1.13.9 (2018-11-29)
-------------------
* Add missing `install` directive in CMakeLists.txt files.
* Fix trackless lcd status not showing.
* Remove unnecessary topic subscriber.
* Fix missing space in lcd message.
* Fix map creator blocked by menu_panel.
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
* agv_menu_panel: led menu fix.
* agv_menu_panel: Fix feedback status timeout and stop other apps when run task runner.
* agv_menu_panel : Add charging state & fix IP always showing.
* Contributors: nik3

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------
* agv_menu_panel: Fix capital case config name.
* agv_menu_panel: Fix battery percentage and dynamic reconfigure.
* Contributors: nikfaisal

1.13.3 (2018-07-16)
-------------------
* Update cancel_default_init command text.
* agv_menu_panel: add shutdown detection message.
* agv_panel:Add shutdown hold power button.
* Fix package and build depends.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nikfaisal

1.13.2 (2018-05-18)
-------------------
* agv_menu_panel: Separate menu LCD node.
* Contributors: nikfaisal

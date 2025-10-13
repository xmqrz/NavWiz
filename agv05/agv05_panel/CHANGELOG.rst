^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_panel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------

3.1.0 (2024-10-16)
------------------

3.0.0 (2023-08-01)
------------------

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------

2.0.0 (2021-05-24)
------------------

1.14.2 (2020-10-19)
-------------------
* Always turn on power LED.
* Merge branch 'node-consolidation'

  * Merge agv05_panel and agv_panel into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------

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
* Blink low-batt led for Zalpha. Fix #28.
* Contributors: Patrick Chin

1.13.4 (2018-08-18)
-------------------
* Fix message type mismatch in led_low_batt topic.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------

1.13.2 (2018-05-18)
-------------------
* Merge panel topics. Remove the now-obsolete agv05_translator.
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
* Fix topic name mismatch.
* Improve audio player to play from different playlist.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* agv05_panel: fix typo in subscriber topics
* agv05_panel: add diagnostic updater
* Update package meta files.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

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
* remove audio volume in panel
* modified led control for panel (negative on, zero off, > 100 forever blink, between 0 - 100 blink)
* added panel button & led control (agv05/peripharel/panel/led_error => 0 always off, >200 always on, >100 delay after end pulse, tenth led on time, digit number of pulse)
* Contributors: nuxail

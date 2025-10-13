^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_remote_io
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Fix agv05_remote_io regression bug.
* Contributors: Patrick Chin

3.1.0 (2024-10-16)
------------------
* Fix noetic-roslint warning.
* Refactor code.
* Fix remote_io reconnection bug.
* Fix python shebangs.
* Update dependencies to support Python 3.
* Contributors: Patrick Chin, Quan Bong, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'remote-io-ui'

  * Stream remote IO inputs and outputs when requested.

* Merge branch 'rio'

  * Add port config for remote IO.

* Contributors: Andrew Tan, Farhan Mustar, Patrick Chin

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------
* Add WISE-4051 remote IO device.
* Contributors: Patrick Chin

2.0.0 (2021-05-24)
------------------

1.14.2 (2020-10-19)
-------------------

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
* Add support for remote IO ADAM-6050 device.
* Contributors: Patrick Chin

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

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------
* Sort remoteIo model choices alphabetically.
* Add remote_io enum model selection.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan

1.13.2 (2018-05-18)
-------------------
* Update remoteIo device id to start from 1, but config for device 0 is lost.
* agv05_remote_io: added new model of remote io WISE-4060 and ADAM-6052
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

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
* agv05_remote_io: add in simulator to simulate input and output for remote io
* Contributors: phtan

1.11.3 (2017-09-29)
-------------------

1.11.0 (2017-08-22)
-------------------

1.10.6 (2017-07-27)
-------------------
* agv05_remote_io: add in diagnostic information for request and reply
* agv05_remote_io: update node to support up to 16 remote io, identification of remote io start from 0-15
* agv05_msgs: update messages for remote io, agv05_remote_io: update new type of publish subscribe to have request and reply
* agv05_remote_io: update config, add publisher for timeout
* Contributors: phtan

1.10.4 (2017-07-10)
-------------------
* agv05_remote_io: change output write method to only write when changes is detected
* agv05_remote_io: add threading for remote IO read write and reduce error message
* agv05_remote_io: update remoteIo.py library to support WISE-4050 model
* Contributors: phtan

1.10.3 (2017-05-26)
-------------------
* agv05_remote_io: fix typo
* Contributors: phtan

1.10.2 (2017-05-06)
-------------------
* agv05_remote_io: fix publish error
* agv05_remote_io: update remoteIo library to support ADAM 6250
* agv05_remote_io: complete working agv05_remote_io package, tested only on ADAM-6256
* agv05_remote_io: rename script folder under package
* agv05_remote_io: add library file to remote io
* Contributors: phtan

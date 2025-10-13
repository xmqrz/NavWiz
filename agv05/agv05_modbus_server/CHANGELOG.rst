^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_modbus_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Merge branch 'safety-renesas' into 'master'

  * Rename time jump to system error.

* Contributors: Wong Tze Lin

3.1.0 (2024-10-16)
------------------
* Fix noetic-roslint warning.
* Refactor code.
* Fix python shebangs.
* Update dependencies to support Python 3.
* Contributors: Patrick Chin, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'time-jump-safety-trigger'

  * Trigger safety when time jump happens.

* Contributors: Andrew Tan, Patrick Chin

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------

2.0.0 (2021-05-24)
------------------

1.14.2 (2020-10-19)
-------------------
* Merge branch 'node-consolidation'

  * Move topics into agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Merge branch '39-update-modbus-server-for-multi-port-io', Fix #39.

  * Fix modbus server for io port update.

* Contributors: Farhan Mustar

1.13.14 (2019-05-21)
--------------------
* Set iptables within the agv05_modbus_server script itself. Fix #32.
* Contributors: HazimGharib, Patrick Chin

1.13.13 (2019-03-22)
--------------------
* Fix bare except statement which suppress KeyboardInterrupt.
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

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------

1.13.3 (2018-07-16)
-------------------
* Fix IO topic names in modbus server.
* update modbus server with read multiple DI CO
* Update modbus with Discrete Input and Coils Output for GPIO purpose
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan, khoo.jack

1.13.2 (2018-05-18)
-------------------
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
* Update modbus server to try to restart if port in use.
* Add input register for modbus server.
* Contributors: farhan

1.12.2 (2018-02-23)
-------------------
* Fix modbus to list from ip.
* Update package metadata and setup.py
* Add modbus server node.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, farhan

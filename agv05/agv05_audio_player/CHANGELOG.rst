^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agv05_audio_player
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-06-12)
------------------
* Check for invalid media file for alarm and beep media.
* Contributors: Farhan Mustar

3.1.0 (2024-10-16)
------------------
* Merge branch 'python3'

  * Fix audio player server.
  * Update dependency.

* Fix noetic-roslint warning.
* Fix python shebangs.
* Contributors: Farhan Mustar, Patrick Chin, Wong Tze Lin

3.0.0 (2023-08-01)
------------------
* Merge branch 'protected-parameter'

  * Suffix audio config with underscore.

* Contributors: Farhan Mustar

2.2.0 (2022-06-30)
------------------

2.1.0 (2021-11-24)
------------------

2.0.0 (2021-05-24)
------------------

1.14.2 (2020-10-19)
-------------------
* Merge branch 'node-consolidation'

  * Move audio topics and msgs to agv05 namespace.

* Contributors: Patrick Chin

1.14.1 (2020-06-03)
-------------------

1.14.0 (2020-05-14)
-------------------

1.13.16 (2020-01-12)
--------------------

1.13.15 (2019-11-21)
--------------------
* Merge branch 'sop39' into 'master'. (`!13 <https://gitlab.com/dfautomation/product/navwiz/agv05/merge_requests/13>`_)

  * Add option to pause music on alarm.

* Contributors: Patrick Chin

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

1.13.6 (2018-09-28)
-------------------

1.13.5 (2018-09-07)
-------------------

1.13.4 (2018-08-18)
-------------------
* Enable shuffle and loop options for audio playlist. Fix #3.
* Added shuffle option for music playback
* Contributors: LowZheXian, Patrick Chin <patrickcjh@gmail.com>

1.13.3 (2018-07-16)
-------------------

1.13.2 (2018-05-18)
-------------------
* Merge audio node and audio messages.
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
* Improve audio player to play from different playlist.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.5.0 (2016-11-25)
------------------
* agv05_audio_player: change config default filename for alarm and beep,
  add in diagnostc updater for audio player
* Fix CMakeLists.txt for catkin_make install.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, phtan

1.4.0 (2016-10-31)
------------------

1.3.0 (2016-10-16)
------------------

1.2.0 (2016-08-25)
------------------
* Update package metadata.
* Contributors: Patrick Chin <patrickcjh@gmail.com>

1.1.0 (2016-08-04)
------------------
* audio_player: fix the index of the music list to start from 1st song after stop is executed
* agv05_audio_player: enable node to auto scan for folder and play the file according to all files inside the folder,
  music files doesn't need to be name 0.mp3, 1.mp3 anymore
* Contributors: phtan

1.0.0 (2016-07-21)
------------------
* create new beep, alarm and music folder when not exists
* Update volume inside dynamic reconfigure callback
* Use dynamic reconfigure and set volume and file by topic in audio player
* Update launch files.
* added panel button & led control (agv05/peripharel/panel/led_error => 0 always off, >200 always on, >100 delay after end pulse, tenth led on time, digit number of pulse)
* rewrite audio player (play all mp3 files in music playlist; set play, pause, stop, volume, filename)
* Add audio player for emergency and running music.
* Contributors: Patrick Chin <patrickcjh@gmail.com>, nuxail, phtan

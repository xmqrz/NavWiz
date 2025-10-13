^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package amcl
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.17.7 (2025-06-12)
-------------------
* Merge branch 'safety-renesas' into 'master'

  * Fix missing timestamp in global to odom transform.

* Contributors: Wong Tze Lin

1.17.6 (2024-10-16)
-------------------
* Merge branch 'map-quality-score' into 'master'

  * Save map quality score only on demand.

* Contributors: Wong Tze Lin

1.17.5 (2024-10-14)
-------------------
* Default profiles to force nomotion update after initialpose.
* Merge branch 'nav-improve-p3' into 'master'

  * Add reflector radius for cylinder reflector.

* Fix return-type warning.
* Contributors: Farhan Mustar, Wong Tze Lin

1.17.4 (2023-09-25)
-------------------
* Update dependency for Python2 backward compatibility.
* Merge remote-tracking branch 'ros/noetic-devel'
* Contributors: Patrick Chin

1.17.3 (2023-01-10)
-------------------
* [AMCL] Add option to force nomotion update after initialpose (`#1226 <https://github.com/ros-planning/navigation/issues/1226>`_)
  * Adds a new boolean parameter force_update_after_initialpose. When set to true, an update is forced on the next laser scan callback, such as when the /request_nomotion_update service is called. This often results in an improved robot pose after a manual (not very precise) re-localization - without a need for the robot to move.
  * Fixes a bunch of compiler warnings (unused variable now, catching exceptions by value), normalizes how tf exceptions are caught
* [ROS-O] various patches (`#1225 <https://github.com/ros-planning/navigation/issues/1225>`_)
  * do not specify obsolete c++11 standard
  this breaks with current versions of log4cxx.
  * update pluginlib include paths
  the non-hpp headers have been deprecated since kinetic
  * use lambdas in favor of boost::bind
  Using boost's _1 as a global system is deprecated since C++11.
  The ROS packages in Debian removed the implicit support for the global symbols,
  so this code fails to compile there without the patch.
* Contributors: Michael Görner, Stephan

1.17.2 (2022-06-20)
-------------------
* Update pf.c (`#1161 <https://github.com/ros-planning/navigation/issues/1161>`_)
  `#1160 <https://github.com/ros-planning/navigation/issues/1160>`_ AMCL miscalculates orientation covariance for clusters
* Improved Overall readablity (`#1177 <https://github.com/ros-planning/navigation/issues/1177>`_)
* fix crashes in AMCL (`#1152 <https://github.com/ros-planning/navigation/issues/1152>`_)
  * fix: catch runtime_error from roscore
  * ignore malformed message from laser, otherwise it will crash
* Fixes `#1117 <https://github.com/ros-planning/navigation/issues/1117>`_ (`#1118 <https://github.com/ros-planning/navigation/issues/1118>`_)
* Fixed the risk of divide by zero. (`#1099 <https://github.com/ros-planning/navigation/issues/1099>`_)
* Contributors: David V. Lu!!, Matthijs van der Burgh, Noriaki Ando, Supernovae, christofschroeter, easylyou

1.17.1 (2020-08-27)
-------------------

1.17.0 (2020-04-02)
-------------------
* Merge pull request `#982 <https://github.com/ros-planning/navigation/issues/982>`_ from ros-planning/noetic_prep
  Noetic Migration
* map is not subscriptable in python3
* fix python3 errors in basic_localization.py
* use upstream pykdl
* Contributors: Michael Ferguson

1.16.16 (2023-07-31)
--------------------
* Merge branch 'protected-parameter'

  * Suffix load_profile with underscore.

* Merge branch 'map-quality-score'

  * Rename topic.
  * Add threshold value of map quality score for warning display.
  * Separate landmark score from being averaged among LiDARs.
  * Update Map Quality Score Formula
  * Update built-in parameter profiles for AMCL.
  * Update inlier percentage calculation for reflector.
  * Diagnostic displays runtime map quality scores of each laser sensor.
  * Fix splitting map quality score calculation of same laser sensor if offset value in agv05_description is changed.
  * Combined all laser sensors in calculating overall map quality score.
  * Update inlier percentage calculation.
  * Add option to create new map quality score.
  * Enable map quality score calculation only if Task Runner is initialized.

* Contributors: Patrick Chin, Wong Tze Lin

1.16.15 (2022-06-29)
--------------------
* Add 3 built-in parameter profiles for AMCL.
* Merge branch 'map-quality-score'

  * Improve map quality by 20% for each matching reflector.
  * Record map quality scores to PNG files.
  * Disable C++11 due to incompatibility with PCL library on ROS Indigo.
  * Refactor and create MapScore class.

* Merge branch 'reflector-matching'

  * Update default parameters.
  * Check inlier percentage threshold before applying reflector matching correction.
  * Refactor ICP to use InlierPercentage class.
  * Compute inlier percentage and display in diagnostic.
  * Add correspondence weight between reflector and landmark.
  * Fix wrong transform threshold calculation in ICP.
  * Optimize calculations.
  * Minimize the squared error instead.
    For correctness sake and to avoid multiple minimas causing jumps in the result.
  * Make reflector matcher run continuously in the background.
  * Refactor code and pre-rotate reflectors to speed up matching.

* Separate intensity threshold for 2 clusters of laser ranges.
* Merge branch 'refactor'

  * Limit laser subscriber queue size to 1.
  * Throttle ICP process rate independently for each laser.
  * Remove time-relative pose correction in ICP as it is duplicated in handleInitialPoseMessage.
  * Refactor to ensure landmarks are always updated in AMCLLaser.

* Allow disabling ICP completedly by setting scan_rate to zero.
* Merge branch 'reflector-matching' into 'master'

  * Add reflector matching service in AMCL.

* Contributors: Patrick Chin, Wong Tze Lin

1.16.14 (2021-11-24)
--------------------
* Increase maximum possible reflector weight.
* Fix tests.
* Fix some params not being applied.
* Refactor code.
* Contributors: Patrick Chin

1.16.13 (2021-05-25)
--------------------
* (AMCL) Skip processing if map unchanged.
* Fix ICP max iterations param not being applied.
* Contributors: Patrick Chin

1.16.12 (2021-02-25)
--------------------
* (AMCL) add missing test dep on tf2_py (`#1092 <https://github.com/ros-planning/navigation/issues/1092>`_)
* (AMCL)(Melodic) use robot pose in tests (`#1088 <https://github.com/ros-planning/navigation/issues/1088>`_)
* (amcl) fix missing '#if NEW_UNIFORM_SAMPLING' (`#1080 <https://github.com/ros-planning/navigation/issues/1080>`_)
* Contributors: Matthijs van der Burgh

1.16.11 (2020-12-22)
--------------------
* Subscribe ocg in png format to reduce size.
* Publish reflectors only if it is used by the active laser_model.
* Remove upper limit for reflector_intensity_threshold config.
* Contributors: Patrick Chin

1.16.10 (2020-10-11)
--------------------
* Use print() function in both Python 2 and Python 3 in basic_localization.py  (`#1011 <https://github.com/ros-planning/navigation/issues/1011>`_)
* (AMCL) add resample limit cache [Melodic] (`#1012 <https://github.com/ros-planning/navigation/issues/1012>`_)
* Contributors: Christian Clauss, Matthijs van der Burgh

1.16.9 (2020-07-22)
-------------------
* Make amcl subscribe to multiple scan topics based on scan_topics parameter.
* Publish amcl particle cloud only when there is at least one subscriber.
* Remove duplicate initialization in constructor and reconfigureCB function.
* Eliminate some dynamic memory allocations.
* Fix memory leak in AMCL.
* Contributors: Patrick Chin

1.16.8 (2020-06-03)
-------------------
* Apply pimpl idiom to SnapMapICP class to fix lz4.h header conflict in ROS melodic PCL library.
* Contributors: Patrick Chin

1.16.7 (2020-05-06)
-------------------
* Add selective_resampling option in AMCL dyncfg parameters.
* [melodic] updated install for better portability. (`#973 <https://github.com/ros-planning/navigation/issues/973>`_)
* Implement selective resampling (`#921 <https://github.com/ros-planning/navigation/issues/921>`_) (`#971 <https://github.com/ros-planning/navigation/issues/971>`_)
  Co-authored-by: Adi Vardi <adidasv111@gmail.com>
* Add CLI option to trigger global localization before processing a bagfile (`#816 <https://github.com/ros-planning/navigation/issues/816>`_) (`#970 <https://github.com/ros-planning/navigation/issues/970>`_)
  Co-authored-by: alain-m <alain@savioke.com>
* amcl: include missing CMake functions to fix build (`#946 <https://github.com/ros-planning/navigation/issues/946>`_)
* Contributors: Ben Wolsieffer, Michael Ferguson, Nicolas Limpert, Patrick Chin, Sean Yen

1.16.6 (2020-05-06)
-------------------
* Merge branch 'reflectors' into 'master' (`!2 <https://gitlab.com/dfautomation/product/navwiz/navigation/merge_requests/2>`_)
  Reflectors
  * Subscribe to map landmarks.
  * Utilize reflector data in AMCL.
* Fix some reconfigure parameters not being applied [amcl]. (`#952 <https://github.com/ros-planning/navigation/issues/952>`_)
* Set proper limits for the z-weights [amcl]. (`#953 <https://github.com/ros-planning/navigation/issues/953>`_)
* Contributors: Patrick Chin, Tang Swee Ho

1.16.5 (2019-11-20)
-------------------
* Merge branch 'static-tf'.
  * Publish AMCL correction as static transform.
    Fix issue on transformation delay.
* Fix typo in amcl_laser model header (`#918 <https://github.com/ros-planning/navigation/issues/918>`_)
* Contributors: Hadi Tabatabaee, Patrick Chin

1.16.4 (2019-09-02)
-------------------
* Log the number of ICP corrections to diagnostic.
* Fix amcl_snap_map_icp library not being installed.
* Contributors: Patrick Chin

1.16.3 (2019-08-13)
-------------------
* Merge branch 'icp' into 'master' (`!1 <https://gitlab.com/dfautomation/product/navwiz/navigation/merge_requests/1>`_)
  Apply ICP pose correction to AMCL
* Remove unused `gui_publish_rate` parameter in AMCL.
* Remove duplicate topic subscriptions.
* Remove unused requestMap functionality in AMCL.
* Load amcl parameters from dynamic_reconfigure_storage.
* Allow backward compatibility with CMake 2.8.3 in ROS indigo.
* Merge pull request `#849 <https://github.com/ros-planning/navigation/issues/849>`_ from seanyen/amcl_windows_fix
  [Windows][melodic] AMCL Windows build bring up.
  * Add HAVE_UNISTD and HAVE_DRAND48 and portable_utils.hpp for better cross compiling.
  * Variable length array is not supported in MSVC, conditionally disable it.
  * Fix install location for shared lib and executables on Windows.
  * Use isfinite for better cross compiling.
* feat: AMCL Diagnostics (`#807 <https://github.com/ros-planning/navigation/issues/807>`_)
  Diagnostic task that monitors the estimated standard deviation of the filter.
  By: reinzor <reinzor@gmail.com>
* fix typo for parameter beam_skip_error_threshold but bandaged for other users in AMCL (`#790 <https://github.com/ros-planning/navigation/issues/790>`_)
  * fix typo but bandage for other users
* Merge pull request `#785 <https://github.com/ros-planning/navigation/issues/785>`_ from mintar/amcl_c++11
  amcl: Add compile option C++11
* Contributors: Martin Günther, Michael Ferguson, Patrick Chin, Rein Appeldoorn, Sean Yen, Steven Macenski

1.16.2 (2018-07-31)
-------------------
* Merge pull request `#773 <https://github.com/ros-planning/navigation/issues/773>`_ from ros-planning/packaging_fixes
  packaging fixes
* update amcl to have proper depends
  * add geometry_msgs
  * add tf2_msgs
  * fix alphabetical order
* Contributors: Michael Ferguson

1.16.1 (2018-07-28)
-------------------
* Merge pull request `#770 <https://github.com/ros-planning/navigation/issues/770>`_ from ros-planning/fix_debians
  Fix debian builds (closes `#769 <https://github.com/ros-planning/navigation/issues/769>`_)
* make AMCL depend on sensor_msgs
  previously, amcl depended on TF, which depended on
  sensor_msgs.
* Contributors: Michael Ferguson

1.16.0 (2018-07-25)
-------------------
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Merge pull request `#734 <https://github.com/ros-planning/navigation/issues/734>`_ from ros-planning/melodic_731
  AMCL dynamic reconfigure: Extend parameter range (Forward port `#731 <https://github.com/ros-planning/navigation/issues/731>`_)
* Merge pull request `#728 <https://github.com/ros-planning/navigation/issues/728>`_ from ros-planning/melodic_tf2_conversion
  switch AMCL to use TF2
* fix swapped odom1/4 in omni model, fixes `#499 <https://github.com/ros-planning/navigation/issues/499>`_
* Merge pull request `#730 <https://github.com/ros-planning/navigation/issues/730>`_ from Glowcloud/melodic-devel
  Fix for Potential Memory Leak  in AmclNode::reconfigureCB `#729 <https://github.com/ros-planning/navigation/issues/729>`_
* Fix for Potential Memory Leak  in AmclNode::reconfigureCB
* switch AMCL to use TF2
* Merge pull request `#727 <https://github.com/ros-planning/navigation/issues/727>`_ from ros-planning/melodic_668
  Update laser_model_type enum on AMCL.cfg (Melodic port of `#668 <https://github.com/ros-planning/navigation/issues/668>`_)
* Update laser_model_type enum on AMCL.cfg
  Adding likelihood_field_prob laser model option on AMCL.cfg to be able to control dynamic parameters with this laser sensor model.
* Merge pull request `#723 <https://github.com/ros-planning/navigation/issues/723>`_ from moriarty/melodic-buildfarm-errors
  Melodic buildfarm errors
* include <memory> for std::shared_ptr
* Merge pull request `#718 <https://github.com/ros-planning/navigation/issues/718>`_ from moriarty/tf2-buffer-ptr
  [melodic] tf2_buffer\_ -> tf2_buffer_ptr\_
* [melodic] tf2_buffer\_ -> tf2_buffer_ptr\_
  Change required due to changes in upstream dependencies:
  `ros/geometry#163 <https://github.com/ros/geometry/issues/163>`_: "Maintain & expose tf2 Buffer in shared_ptr for tf"
  fixes `ros-planning/navigation#717 <https://github.com/ros-planning/navigation/issues/717>`_ (for compile errors at least.)
* Contributors: Alexander Moriarty, Glowcloud, Martin Ganeff, Michael Ferguson, Miguel Cordero, Vincent Rabaud, maracuya-robotics

1.15.2 (2018-03-22)
-------------------
* Fix minor typo (`#682 <https://github.com/ros-planning/navigation/issues/682>`_)
  This typo caused some confusion because we were searching for a semicolon in our configuration.
* Merge pull request `#677 <https://github.com/ros-planning/navigation/issues/677>`_ from ros-planning/lunar_634
  removing recomputation of cluster stats causing assertion error (`#634 <https://github.com/ros-planning/navigation/issues/634>`_)
* Merge pull request `#673 <https://github.com/ros-planning/navigation/issues/673>`_ from ros-planning/email_update_lunar
  update maintainer email (lunar)
* Remove Dead Code [Lunar] (`#646 <https://github.com/ros-planning/navigation/issues/646>`_)
  * Clean up navfn
  * Cleanup amcl
* Merge pull request `#649 <https://github.com/ros-planning/navigation/issues/649>`_ from aaronhoy/lunar_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, David V. Lu!!, Michael Ferguson, stevemacenski

1.15.1 (2017-08-14)
-------------------

1.15.0 (2017-08-07)
-------------------
* Reference Issue `#592 <https://github.com/ros-planning/navigation/issues/592>`_ Added warning to AMCL when map is published on ... (`#604 <https://github.com/ros-planning/navigation/issues/604>`_)
* rebase fixups
* convert packages to format2
* recompute cluster stat when force_publication
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* amcl: fix compilation with gcc v7
* Added deps to amcl costmap_2d move_base (`#512 <https://github.com/ros-planning/navigation/issues/512>`_)
* fix order of parameters (closes `#553 <https://github.com/ros-planning/navigation/issues/553>`_)
* Fix potential string overflow and resource leak
* Contributors: Dmitry Rozhkov, Laurent GEORGE, Martin Günther, Michael Ferguson, Mikael Arguedas, Peter Harliman Liem, mryellow, vik748

1.14.0 (2016-05-20)
-------------------
* Allow AMCL to run from bag file to allow very fast testing.
* Fixes interpretation of a delayed initialpose message (see `#424 <https://github.com/ros-planning/navigation/issues/424>`_).
  The tf lookup as it was before this change was very likely to fail as
  ros::Time::now() was used to look up a tf without waiting on the tf's
  availability. Additionally, the computation of the "new pose" by
  multiplying the delta that the robot moved from the initialpose's
  timestamp to ros::Time::now() was wrong. That delta has to by multiplied
  from the right to the "old pose".
  This commit also changes the reference frame to look up this delta to be
  the odom frame as this one is supposed to be smooth and therefore the
  best reference to get relative robot motion in the robot (base link) frame.
* New unit test for proper interpretation of a delayed initialpose message.
  Modifies the set_pose.py script to be able to send an initial pose with
  a user defined time stamp at a user defined time. Adds a rostest to
  exercise this new option.
  This reveals the issues mentioned in `#424 <https://github.com/ros-planning/navigation/issues/424>`_ (the new test fails).
* Contributors: Derek King, Stephan Wirth

1.13.1 (2015-10-29)
-------------------
* adds the set_map service to amcl
* fix pthread_mutex_lock on shutdown
* Contributors: Michael Ferguson, Stephan Wirth

1.13.0 (2015-03-17)
-------------------
* amcl_node will now save latest pose on shutdown
* Contributors: Ian Danforth

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------

1.11.14 (2014-12-05)
--------------------

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* Bug fix to remove particle weights being reset when motion model is updated
* Integrated new sensor model which calculates the observation likelihood in a probabilistic manner
  Also includes the option to do beam-skipping (to better handle observations from dynamic obstacles)
* Pose pulled from parameter server when new map received
* Contributors: Steven Kordell, hes3pal

1.11.11 (2014-07-23)
--------------------

1.11.10 (2014-06-25)
--------------------

1.11.9 (2014-06-10)
-------------------

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* removes useless this->z_max = z_max assignment
* Fix warning string.
* Contributors: Jeremiah Via, enriquefernandez

1.11.5 (2014-01-30)
-------------------
* Fix for `#160 <https://github.com/ros-planning/navigation/issues/160>`_
* Download test data from download.ros.org instead of willow
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* amcl_pose and particle cloud are now published latched
* Fixed or commented out failing amcl tests.


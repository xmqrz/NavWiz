^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package marker_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.3 (2025-06-12)
------------------
* Fix single pocket minimum block-pair checking.
* Merge branch 'nav-improve' into 'master'

  * Navigation Improvement Phase 4: Bar-AR, Pallet
  * Add camera5.
  * Patch for melodic roslint.
  * Merge vertical strips which are discontinued by noise.
  * Retry with split cluster at minima point.
  * Add support to single pocket pallet detection.
  * Fix marker is not on center block of pallet.
  * Detect pallet deck.
  * Fix https://github.com/kam3k/laser_line_extraction/issues/19.
  * Add bar-ar marker detection.
  * Refactor code.

* Merge branch 'nav-improve' into 'master'

  * Navigation Improvement Phase 4
  * AR marker default to disable image sharpening.
  * Add external heading offset parameter.
  * Add option to use external heading in sensor frame for single ID AR marker.
  * Add image sharpening pre-process.
  * Add bar-reflector marker detection.

* Fix visualization.
* Use compressed transport for depth image subscriber.
* Update image transport publisher params.
* Increase maximum cap of pallet block height and width.
* Contributors: Patrick Chin, Wong Tze Lin

1.4.2 (2024-10-28)
------------------
* Merge branch 'drift-filter' into 'master'

  * Add maximum 2D pose deviation from the 1st found marker filtering.

* Merge branch 'ar-improve' into 'master'

  * Fix heading jump if single ID AR marker 45 degree tilted.
  * Add region of interest in detecting AR marker.
  * Fix wrong heading if AR marker found in between camera and center of wheels.

* Contributors: Wong Tze Lin

1.4.1 (2024-10-09)
------------------
* Merge branch 'hal' into 'master'

  * NavWiz 3.1 HAL
  * See merge request dfautomation/product/navwiz/marker_localization!6

* Remove profile number for custom marker detection.
* Add laser6.
* Add custom marker detection.
* Fix build in ROS Indigo.
* Fix compile error.
* Merge branch 'nav-improve-p3' into 'master'

  * Navigation Improvement Phase 3
  * See merge request dfautomation/product/navwiz/marker_localization!5

* Add mean filter on marker position.
* Add reflector radius for cylinder reflector.
* Merge branch 'ar-improvement' into 'master'

  * AR Improvement
  * See merge request dfautomation/product/navwiz/marker_localization!4

* Add filtered scan data view.
* Extend AR marker_type syntax to support ID and custom trim ranges.
* Extend AR marker_type syntax to support multiple profiles.
* Add parameters of AR min/max separation and max position deviation. Refactor code.
* Contributors: Farhan Mustar, Wong Tze Lin

1.4.0 (2023-08-01)
------------------
* Refactor code.
* Merge branch 'sensor-topic'

  * Support scan4 and scan5 topics.

* Merge branch 'reflector-improvement'

  * Extend reflector marker_type syntax to support custom trim ranges.

* Merge branch 'bar-improvement'

  * Separate marker type syntax for bar marker detection at center of heads or tails of bars-pair.

* Restrict aruco version to ensure compatibility.
* Contributors: Patrick Chin, Wong Tze Lin

1.3.0 (2022-06-29)
------------------
* Merge branch 'pallet-improvement' into 'master'

  * Add support to marker_type syntax of pallet{profile}_\_{sensor}__r{rangeMax}.
  * Extend pallet marker_type syntax to support multiple profiles.

* Contributors: Patrick Chin, Wong Tze Lin

1.2.0 (2021-11-24)
------------------
* Merge branch 'vmarker-improvement' into 'master'

  * Nested tab display for Reflector and Line Marker parameter settings.
  * Extend reflector marker_type syntax to support multiple profiles.
  * Extend vmarker marker_type syntax to support multiple profiles and custom trim ranges.
  * Extend vmarker to support lmarker, vlmarker, v2marker, l2marker and barmarker.

* Split up the depth callback function.
* Remove unused code.
* Merge branch 'laser-360' into 'master'

  * Fix duplicated checking on same reflector pair if there are only 2 reflectors found. Refactor code.
  * Add parameter reflector_max_position_deviation. Refactor code.
  * Pick the nearest if multiple candidates are available.
  * Add support to both angle ranges of [-180°, 180°] and [0°, 360°] for 360° laser.

* Contributors: Patrick Chin, Wong Tze Lin

1.1.0 (2021-05-25)
------------------
* Merge branch 'vmarker_improvement'

  * Changed abs to fabs, data will cache only when angle or sensor change.
  * Added trim angle for vmarker.
  * Add max_range in config.
  * Filter outward v.

* Merge branch 'reflector-bugfix'

  * Remove roll and pitch components of reflector's marker tf when laser is inverted.
  * Remove upper limit for reflector_intensity_threshold config.

* Use sensor timestamp to transform marker into odom frame.
* Add marker position in diagnostic.
* Merge branch 'pallet-improvement'

  * Allow specifying trim ranges in the marker_type string for pallet detector.
  * Add check for center block line fitting error.
  * Add option to set max threshold of position and orientation for first detection.
  * Add option to lock pallet orientation after first detection.
  * Add 'palletR' as alias to 'pallet' detector type.
  * Improve vstrip clustering.
  * Improve normals calculation for vertical surface filter.
  * Improve vstrip detection.
  * Publish depth image after vertical surface filter for debugging.
  * Use minima instead of median point of pallet block to increase stability.
  * Add option to lock pallet position within certain deviation from first detected position.
  * Allow specifying trim angles in the marker_type string for pallet detector.
  * Publish empty cloud when processing stop to hide them in UI.
  * Use depth image instead of pointcloud input for faster computation.

* Allow specifying trim angles in the marker_type string for reflector.
* Add separation criteria for reflectors.
* Publish empty reflector list when processing stop to hide them in UI.
* Avoid shutting down publishers as peer's resubscription takes time.
* Rearrange dyncfg parameters.
* Add visualization marker for pallet.
* Add support for third and fourth cameras.
* Contributors: Koh Wan Hui, Patrick Chin, Wong Tze Lin

1.0.0 (2020-10-21)
------------------
* Refactor code into detector classes. Improve pallet detection.
* Add ar markers images.
* Localization using ar markers.
* Cast PointXYZ to PointXYZRGB.
* Use pointcloud to detect pallet height.
* Using realsense for docking to pallet.
* Find the nearest and second nearest reflectors wrt amr.
* Remove scan_front\_ and rename string of marker_type.
* Use index instead of frame_id.
* Add marker_type and remove service.
* Add reflector localization.
* Migrate tf to tf2.
* Update config description for line extraction.
* Add marker result visualization.
* Add start and stop service and update diagnostic display.
* Add scan range configuration and publish selected v marker coordinate to laser.
* Add diagnostic publisher and refactor node header.
* Add line viz publisher and add dynamic reconfigure for v extraction parameters.
* Add filter to always track the same marker.
* Add extract v marker sequence.
* Add dynamic reconfigure for line_extraction component.
* Contributors: Farhan Mustar, Patrick Chin, Tang Swee Ho

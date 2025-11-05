/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_BAR_REFLECTOR_H
#define MARKER_LOCALIZATION_DETECTORS_BAR_REFLECTOR_H

#include <marker_localization/detectors/reflector.h>
#include <marker_localization/detectors/vmarker/bar_mixin.h>
#include <marker_localization/detectors/vmarker/line_extraction.h>


namespace marker_localization
{

class BarReflectorDetector: public ReflectorDetector, public BarMixin
{
public:
  BarReflectorDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile) :
    ReflectorDetector(ml, sensor, profile),
    BarMixin(config_, ReflectorDetector::trim_index_min_, ReflectorDetector::trim_index_max_)
  {
    switch (profile)
    {
#define _LOAD_PROFILE(p) \
  config_reflector_.intensity_threshold    = &config_.bar_reflector_intensity_threshold##p;         \
  config_reflector_.min_width              = &config_.bar_reflector_min_width##p;                   \
  config_reflector_.max_width              = &config_.bar_reflector_max_width##p;                   \
  config_reflector_.radius                 = &config_.bar_reflector_radius##p;                      \
  config_reflector_.max_position_deviation = &config_.bar_reflector_max_translational_deviation##p; \
  config_bar_.min_angle                    = &config_.bar_reflector_min_angle##p;                   \
  config_bar_.max_angle                    = &config_.bar_reflector_max_angle##p;                   \
  config_bar_.min_length                   = &config_.bar_reflector_min_length##p;                  \
  config_bar_.max_length                   = &config_.bar_reflector_max_length##p;                  \
  config_bar_.max_gap                      = &config_.bar_reflector_max_width##p

    default:
    case 1: _LOAD_PROFILE(1); break;
    case 2: _LOAD_PROFILE(2); break;
    case 3: _LOAD_PROFILE(3); break;
    case 4: _LOAD_PROFILE(4); break;
    case 5: _LOAD_PROFILE(5); break;

#undef _LOAD_PROFILE
    }
  }

  void start();
  void stop();

protected:
  void cacheData(const sensor_msgs::LaserScan& scan);
  void processData(const sensor_msgs::LaserScan& scan);
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_BAR_REFLECTOR_H

/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_BAR_AR_H
#define MARKER_LOCALIZATION_DETECTORS_BAR_AR_H

#include <marker_localization/detectors/ar.h>
#include <marker_localization/detectors/vmarker/bar_mixin.h>
#include <marker_localization/detectors/vmarker/line_extraction.h>

#include <std_msgs/Float32.h>


namespace marker_localization
{

class BarARDetector: public ARDetector, public BarMixin
{
public:
  BarARDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile) :
    ARDetector(ml, sensor, profile),
    BarMixin(config_, trim_index_min_, trim_index_max_),
    data_cached_(false), trim_index_min_(0), trim_index_max_(0), marker_heading_(-M_PI)
  {
    switch (profile)
    {
#define _LOAD_PROFILE(p) \
  config_ar_.image_is_rectified     = &config_.bar_ar_image_is_rectified##p;          \
  config_ar_.sharp_strength         = &config_.bar_ar_sharp_strength##p;              \
  config_ar_.sharp_sigma            = &config_.bar_ar_sharp_sigma##p;                 \
  config_ar_.height_start           = &config_.bar_ar_height_start##p;                \
  config_ar_.height_end             = &config_.bar_ar_height_end##p;                  \
  config_ar_.width_start            = &config_.bar_ar_width_start##p;                 \
  config_ar_.width_end              = &config_.bar_ar_width_end##p;                   \
  config_ar_.marker_size            = &config_.bar_ar_marker_size##p;                 \
  config_ar_.max_position_deviation = &config_.bar_ar_max_translational_deviation##p; \
  config_bar_.min_angle             = &config_.bar_ar_min_angle##p;                   \
  config_bar_.max_angle             = &config_.bar_ar_max_angle##p;                   \
  config_bar_.min_length            = &config_.bar_ar_min_length##p;                  \
  config_bar_.max_length            = &config_.bar_ar_max_length##p;                  \
  config_bar_.max_gap               = &config_.bar_ar_max_gap##p;                     \
  config_bar_ar_.ext_heading        = &config_.bar_ar_ext_heading##p

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
  void processData(const sensor_msgs::ImageConstPtr& msg);
  void callbackLaserScan(const sensor_msgs::LaserScanConstPtr& msg);
  void callbackMarkerHeading(const std_msgs::Float32& msg);

protected:
  ros::Subscriber laser_sub_;
  ros::Subscriber marker_heading_sub_;

  sensor_msgs::LaserScanConstPtr scan_;
  bool data_cached_;
  int trim_index_min_;
  int trim_index_max_;
  float marker_heading_;

  // bar AR config
  struct
  {
    const double *ext_heading;
  } config_bar_ar_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_BAR_AR_H

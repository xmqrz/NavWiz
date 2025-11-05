/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_REFLECTOR_H
#define MARKER_LOCALIZATION_DETECTORS_REFLECTOR_H

#include <marker_localization/detector.h>

#include <sensor_msgs/LaserScan.h>


namespace marker_localization
{

class ReflectorDetector: public Detector
{
public:
  ReflectorDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile) :
    Detector(ml, sensor, profile),
    data_cached_(false)
  {
    switch (profile)
    {
#define _LOAD_PROFILE(p) \
  config_reflector_.intensity_threshold    = &config_.reflector_intensity_threshold##p; \
  config_reflector_.min_width              = &config_.reflector_min_width##p;           \
  config_reflector_.max_width              = &config_.reflector_max_width##p;           \
  config_reflector_.radius                 = &config_.reflector_radius##p;              \
  config_reflector_.min_separation         = &config_.reflector_min_separation##p;      \
  config_reflector_.max_separation         = &config_.reflector_max_separation##p;      \
  config_reflector_.max_position_deviation = &config_.reflector_max_position_deviation##p

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
  virtual void cacheData(const sensor_msgs::LaserScan& scan);
  virtual void processData(const sensor_msgs::LaserScan& scan);
  void callbackLaserScan(const sensor_msgs::LaserScan& scan)
  {
    if (!data_cached_) cacheData(scan);
    processData(scan);
  }
  void getReflectorIndices(const sensor_msgs::LaserScan& scan,
                           geometry_msgs::PoseArray& markers, std::set<size_t>& indices);
  int getReflectorIndex(const sensor_msgs::LaserScan& scan, int start, int end);

protected:
  ros::Subscriber laser_sub_;

  bool data_cached_;
  float trim_angle_min_;
  float trim_angle_max_;
  float trim_range_max_;
  int trim_index_min_;
  int trim_index_max_;

  // reflector config
  struct
  {
    const double *intensity_threshold;
    const double *min_width;
    const double *max_width;
    const double *radius;
    const double *min_separation;
    const double *max_separation;
    const double *max_position_deviation;
  } config_reflector_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_REFLECTOR_H

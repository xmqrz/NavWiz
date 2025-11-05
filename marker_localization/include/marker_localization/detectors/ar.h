/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tang Swee Ho
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_AR_H
#define MARKER_LOCALIZATION_DETECTORS_AR_H

#include <aruco/aruco.h>
#include <image_transport/image_transport.h>
#include <marker_localization/detector.h>

#include <aruco_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32MultiArray.h>


namespace marker_localization
{

class ARDetector: public Detector
{
public:
  ARDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile) :
    Detector(ml, sensor, profile)
  {
    switch (profile)
    {
#define _LOAD_PROFILE(p) \
  config_ar_.image_is_rectified     = &config_.ar_image_is_rectified##p;  \
  config_ar_.sharp_strength         = &config_.ar_sharp_strength##p;      \
  config_ar_.sharp_sigma            = &config_.ar_sharp_sigma##p;         \
  config_ar_.height_start           = &config_.ar_height_start##p;        \
  config_ar_.height_end             = &config_.ar_height_end##p;          \
  config_ar_.width_start            = &config_.ar_width_start##p;         \
  config_ar_.width_end              = &config_.ar_width_end##p;           \
  config_ar_.marker_size            = &config_.ar_marker_size##p;         \
  config_ar_.min_separation         = &config_.ar_min_separation##p;      \
  config_ar_.max_separation         = &config_.ar_max_separation##p;      \
  config_ar_.max_position_deviation = &config_.ar_max_position_deviation##p

    default:
    case 1: _LOAD_PROFILE(1); break;
    case 2: _LOAD_PROFILE(2); break;
    case 3: _LOAD_PROFILE(3); break;
    case 4: _LOAD_PROFILE(4); break;
    case 5: _LOAD_PROFILE(5); break;

#undef _LOAD_PROFILE
    }
  }

  bool start(bool cloud_flatten);
  void start()
  {
    start(false);
  }
  void stop();

protected:
  void getMarkers(const sensor_msgs::ImageConstPtr& msg, geometry_msgs::PoseArray& marker_msg);
  virtual void processData(const sensor_msgs::ImageConstPtr& msg);
  void callbackImage(const sensor_msgs::ImageConstPtr& msg)
  {
    processData(msg);
  }

protected:
  static ros::Publisher marker_list_pub_;
  static image_transport::Publisher image_pub_;
  static image_transport::Publisher debug_pub_;

  image_transport::Subscriber image_sub_;

  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;

  int16_t id_[2];
  float trim_range_max_;

  // AR config
  struct
  {
    const bool *image_is_rectified;
    const double *sharp_strength;
    const double *sharp_sigma;
    const double *height_start;
    const double *height_end;
    const double *width_start;
    const double *width_end;
    const double *marker_size;
    const double *min_separation;
    const double *max_separation;
    const double *max_position_deviation;
  } config_ar_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_AR_H

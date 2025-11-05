/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_CUSTOM_H
#define MARKER_LOCALIZATION_DETECTORS_CUSTOM_H

#include <marker_localization/detector.h>

#include <agv05_msgs/Marker2D.h>


namespace marker_localization
{

class CustomDetector: public Detector
{
public:
  CustomDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile) :
    Detector(ml, sensor, profile)
  {}

  void start();
  void stop();

private:
  void callbackMarker(const agv05_msgs::Marker2D& msg);

private:
  ros::Subscriber marker_sub_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_CUSTOM_H

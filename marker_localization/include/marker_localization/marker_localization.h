/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef MARKER_LOCALIZATION_MARKER_LOCALIZATION_H
#define MARKER_LOCALIZATION_MARKER_LOCALIZATION_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <marker_localization/MarkerLocalizationConfig.h>
#include <std_msgs/String.h>


namespace marker_localization
{

class Detector;

class MarkerLocalization
{
  friend class Detector;

public:
  MarkerLocalization();

private:
  void handleMarkerType(const std_msgs::String& msg);

  void timerCallback(const ros::TimerEvent& event)
  {
    diagnostic_updater_.update();
  }

  // diagnostic
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // dynamic reconfigure
  void callbackConfig(marker_localization::MarkerLocalizationConfig& config, uint32_t level)
  {
    ROS_INFO("marker_localization: config received");
    config_ = config;
  }

private:
  /* ROS tf2 listener and broadcaster */
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tfl_;
  tf2_ros::TransformBroadcaster tfb_;

  /* ROS publishers */
  ros::Publisher cloud_flatten_pub_;
  ros::Publisher landmarks_pub_;

  /* ROS subscribers */
  ros::Subscriber marker_type_sub_;

  /* Dynamic reconfigure server */
  dynamic_reconfigure::Server<marker_localization::MarkerLocalizationConfig> ds_;

  /* ROS diagnostic */
  diagnostic_updater::Updater diagnostic_updater_;

  /* Timer */
  ros::Timer timer_;

  /* Data inputs */
  marker_localization::MarkerLocalizationConfig config_;
  std::string marker_type_;

  /* Detector */
  boost::shared_ptr<Detector> detector_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_MARKER_LOCALIZATION_H

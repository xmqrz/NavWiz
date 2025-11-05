/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <marker_localization/detectors/custom.h>


namespace marker_localization
{

void CustomDetector::start()
{
  ros::NodeHandle nh;

  Detector::start(nh, false, true);
  marker_sub_ = nh.subscribe("marker_custom", 1, &CustomDetector::callbackMarker, this);
}

void CustomDetector::stop()
{
  marker_sub_.shutdown();
  Detector::stop();
}

void CustomDetector::callbackMarker(const agv05_msgs::Marker2D& msg)
{
  if (sensor_ != msg.sensor) return;

  std::string docking_frame_id = "laser";  // TODO(someday): hack-ish as UI only display raw markers in laser frame
  geometry_msgs::PoseArray marker_msg;
  marker_msg.header = msg.header;
  marker_msg.poses.clear();

  geometry_msgs::Pose2D marker_new;
  tf2::Transform transform;
  if (frameTransform(msg.header, msg.marker, marker_new, docking_frame_id))
  {
    marker_msg.header.frame_id = docking_frame_id;
    tf2::fromMsg(marker_new, transform);
  }
  else
  {
    tf2::fromMsg(msg.marker, transform);
  }

  marker_msg.poses.resize(1);
  tf2::toMsg(transform, marker_msg.poses[0]);

  handleMarkers(msg.header, {msg.marker}, 0.0);

  // publish marker array
  landmarks_pub_.publish(marker_msg);
}

}  // namespace marker_localization

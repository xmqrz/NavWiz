/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <marker_localization/detectors/bar_reflector.h>


namespace marker_localization
{

void BarReflectorDetector::start()
{
  ReflectorDetector::start();
  BarMixin::start(trim_range_max_);
}

void BarReflectorDetector::stop()
{
  ReflectorDetector::stop();
}

void BarReflectorDetector::cacheData(const sensor_msgs::LaserScan& scan)
{
  ReflectorDetector::cacheData(scan);
  BarMixin::cacheData(scan);
}

void BarReflectorDetector::processData(const sensor_msgs::LaserScan& scan)
{
  // reflector extraction
  geometry_msgs::PoseArray markers;
  std::set<size_t> indices;
  getReflectorIndices(scan, markers, indices);

  // marker extraction
  std::vector<geometry_msgs::Pose2D> markers_2d;
  if (markers.poses.size())
  {
    BarMixin::processData(scan, markers, markers_2d, indices);
  }

  // Publish all the reflectors detected by laser
  landmarks_pub_.publish(markers);

  // Publish all bar and reflective points detected by laser
  if (cloud_flatten_pub_.getNumSubscribers() > 0)
  {
    publishCloudFlatten(scan, indices);
  }

  handleMarkers(scan.header, markers_2d, *config_reflector_.max_position_deviation);
}

}  // namespace marker_localization

/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <angles/angles.h>
#include <marker_localization/detectors/bar_ar.h>


namespace marker_localization
{

void BarARDetector::start()
{
  if (!ARDetector::start(true)) return;
  BarMixin::start(trim_range_max_);

  ros::NodeHandle nh;
  std::string topic;
  int index = sensor_.find('_');
  if (index != std::string::npos)
  {
    topic = getLaserTopic(sensor_.substr(index + 1));
  }
  if (!topic.length())
  {
    topic = getLaserTopic(sensor_.substr(0, index));
    marker_heading_sub_ = nh.subscribe("/marker_heading", 1, &BarARDetector::callbackMarkerHeading, this);
  }
  laser_sub_ = nh.subscribe(topic, 1, &BarARDetector::callbackLaserScan, this);
}

void BarARDetector::stop()
{
  laser_sub_.shutdown();
  marker_heading_sub_.shutdown();
  ARDetector::stop();
}

void BarARDetector::processData(const sensor_msgs::ImageConstPtr& msg)
{
  // aruco extraction
  geometry_msgs::PoseArray markers;
  getMarkers(msg, markers);
  if (markers.poses.empty()) return;

  // marker extraction
  std::vector<geometry_msgs::Pose2D> markers_2d;
  if (marker_heading_ >= 0)
  {
    if (frameTransform(markers, &markers_2d))
    {
      double theta = marker_heading_ > M_PI ? marker_heading_ - M_PI * 2 : marker_heading_;
      for (auto& marker : markers_2d)
      {
        marker.theta = theta;
      }
    }
  }
  else if (data_cached_)
  {
    if (frameTransform(markers, NULL, scan_->header.frame_id))
    {
      const sensor_msgs::LaserScan& scan = *scan_;
      std::set<size_t> indices;
      BarMixin::processData(scan, markers, markers_2d, indices);

      // Publish all bar and reflective points detected by laser
      if (cloud_flatten_pub_.getNumSubscribers() > 0)
      {
        publishCloudFlatten(scan, indices);
      }
    }
  }

  handleMarkers(markers.header, markers_2d, *config_ar_.max_position_deviation);
}

void BarARDetector::callbackLaserScan(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan_ = msg;
  if (!data_cached_)
  {
    trim_index_max_ = scan_->ranges.size();
    cacheData(*scan_);
    data_cached_ = true;
  }
}

void BarARDetector::callbackMarkerHeading(const std_msgs::Float32& msg)
{
  marker_heading_ = *config_bar_ar_.ext_heading < 0 ? *config_bar_ar_.ext_heading :
                    angles::normalize_angle_positive(msg.data + *config_bar_ar_.ext_heading);
}

}  // namespace marker_localization

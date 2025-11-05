/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include <marker_localization/detectors/vmarker.h>
#include <sstream>


namespace marker_localization
{

void VMarkerDetector::start()
{
  ros::NodeHandle nh;

  // ROS Publishers
  Detector::start(nh, true, false);
  if (!marker_viz_pub_)
  {
    // publishers are static variables and only registered once
    // publisher shutdown is avoided as it takes time for its peers to resubscribe
    marker_viz_pub_ = nh.advertise<visualization_msgs::Marker>("/marker_viz", 1);
  }

  // determine trim angles and range
  // possible string formats:
  //  {sensor}__{totalAngle} or {sensor}__{angleMin}_{angleMax} or {sensor}__r{rangeMax} or
  //  {sensor}__{totalAngle}__r{rangeMax} or {sensor}__{angleMin}_{angleMax}__r{rangeMax}
  // input units: angle -> degrees, range -> meters
  trim_angle_min_ = -M_PI;
  trim_angle_max_ = M_PI;
  double trim_range_max = config_.max_range;

  std::string angle_str;
  int index = sensor_.find("__");
  if (index != std::string::npos)
  {
    angle_str = sensor_.substr(index + 2);
    sensor_ = sensor_.substr(0, index);
  }
  std::string range_str;
  index = angle_str.find("__r");
  if (index != std::string::npos)
  {
    range_str = angle_str.substr(index + 3);
    angle_str = angle_str.substr(0, index);
  }
  else if (angle_str.front() == 'r')
  {
    range_str = angle_str.substr(1);
    angle_str.clear();
  }

  if (!angle_str.empty())
  {
    index = angle_str.find("_");
    if (index == std::string::npos)
    {
      std::istringstream iss(angle_str);
      float angle;

      if (iss >> angle)
      {
        // negative totalAngle = angle to be excluded
        if (fabs(angle) < 360)
        {
          angle *= M_PI / 180 / 2;
          trim_angle_min_ = -angle;
          trim_angle_max_ = angle;
        }
      }
    }
    else
    {
      std::istringstream iss(angle_str.substr(0, index));
      std::istringstream iss2(angle_str.substr(index + 1));
      float angle_min;
      float angle_max;

      if ((iss >> angle_min) && (iss2 >> angle_max))
      {
        if (fabs(angle_max - angle_min) <= 360)
        {
          trim_angle_min_ = angle_min * M_PI / 180;
          trim_angle_max_ = angle_max * M_PI / 180;
        }
      }
    }
  }

  if (!range_str.empty())
  {
    std::istringstream iss(range_str);
    float range;

    if (iss >> range)
    {
      trim_range_max = fabs(range);
    }
  }

  // ROS Subscribers
  std::string topic = getLaserTopic(sensor_);
  if (topic.length())
  {
    laser_sub_ = nh.subscribe(topic, 1, &VMarkerDetector::callbackLaserScan, this);
  }
  else
  {
    ROS_ERROR_STREAM("No sensor found: " << sensor_);
  }

  // Line Extraction config
  line_extraction_.setBearingVariance(config_.bearing_std_dev * config_.bearing_std_dev);
  line_extraction_.setRangeVariance(config_.range_std_dev * config_.range_std_dev);
  line_extraction_.setLeastSqAngleThresh(config_.least_sq_angle_thresh);
  line_extraction_.setLeastSqRadiusThresh(config_.least_sq_radius_thresh);
  line_extraction_.setMaxLineGap(config_.max_line_gap);
  line_extraction_.setMinLineLength(config_.min_line_length);
  line_extraction_.setMinRange(config_.min_range);
  line_extraction_.setMaxRange(trim_range_max);
  line_extraction_.setMinSplitDist(config_.min_split_dist);
  line_extraction_.setOutlierDist(config_.outlier_dist);
  line_extraction_.setMinLinePoints(config_.min_line_points);

  // Misc config
  max_trans_dev_ = marker_extraction_->setConfig(config_, profile_);
  pub_viz_ = config_.publish_visualization;
}

void VMarkerDetector::stop()
{
  laser_sub_.shutdown();
}

void VMarkerDetector::callbackLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  // Line Extraction
  if (!data_cached_)
  {
    data_cached_ = true;
    getTrimIndex(static_cast<int>(laser_scan->ranges.size()),
                 laser_scan->angle_min, laser_scan->angle_increment,
                 trim_angle_min_, trim_angle_max_, trim_index_min_, trim_index_max_);
    ROS_DEBUG_STREAM(sensor_ << " trim angle: " << trim_angle_min_ << " -> " << trim_angle_max_
                             << " trim index: " << trim_index_min_ << " -> " << trim_index_max_);

    cacheData(laser_scan);
  }

  int i = trim_index_min_;
  int n = (trim_index_min_ && (trim_index_min_ >= trim_index_max_)) ?
          static_cast<int>(laser_scan->ranges.size()) : trim_index_max_;

  if (i >= n)
  {
    return;
  }

  std::vector<double> scan_ranges_doubles(laser_scan->ranges.begin() + i, laser_scan->ranges.begin() + n);
  if (trim_index_min_ && (trim_index_min_ >= trim_index_max_))
  {
    scan_ranges_doubles.insert(scan_ranges_doubles.end(), laser_scan->ranges.begin(), laser_scan->ranges.begin() + trim_index_max_);
  }
  line_extraction_.setRangeData(scan_ranges_doubles);

  // line extraction
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // marker extraction
  Pose2DList markers;
  std::set<size_t> line_indices;
  marker_extraction_->getMarkers(lines, markers, line_indices);
  ROS_DEBUG_STREAM("markers size: " << markers.size());

  geometry_msgs::Pose2D result = handleMarkers(laser_scan->header, markers, max_trans_dev_);

  if (cloud_flatten_pub_.getNumSubscribers() > 0)
  {
    populateLineViz(laser_scan, lines, line_indices);
  }
  if (pub_viz_)
  {
    ROS_DEBUG_STREAM("publish marker");
    visualization_msgs::Marker marker_msg, markers_msg;
    marker_msg.header = laser_scan->header;
    markers_msg.header = laser_scan->header;
    populateLineViz(lines, marker_msg);
    populateVViz(markers, markers_msg);
    marker_viz_pub_.publish(marker_msg);
    marker_viz_pub_.publish(markers_msg);
  }

  // diagnostic
  diagnostic_line_detected_ = lines.size();
  diagnostic_v_detected_ = markers.size();
  if (!std::isnan(result.x))
  {
    diagnostic_v_x_ = boost::to_string(static_cast<double>(result.x));
    diagnostic_v_y_ = boost::to_string(static_cast<double>(result.y));
  }
}

void VMarkerDetector::populateLineViz(const sensor_msgs::LaserScanConstPtr& laser_scan,
                                      const std::vector<Line>& lines, const std::set<size_t>& line_indices)
{
  std::set<size_t> indices;

  for (auto idx : line_indices)
  {
    for (auto i : lines[idx].getIndices())
    {
      size_t n = i + trim_index_min_;
      indices.insert(n > laser_scan->ranges.size() ? n - laser_scan->ranges.size() : n);
    }
  }

  publishCloudFlatten(*laser_scan, indices);
}

void VMarkerDetector::populateLineViz(const std::vector<Line>& lines, visualization_msgs::Marker& marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator it = lines.begin(); it != lines.end(); ++it)
  {
    geometry_msgs::Point p_start;
    p_start.x = it->getStart()[0];
    p_start.y = it->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = it->getEnd()[0];
    p_end.y = it->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
}

void VMarkerDetector::populateVViz(const Pose2DList& vlist, visualization_msgs::Marker& marker_msg)
{
  marker_msg.ns = "v_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::POINTS;
  marker_msg.scale.x = 0.1;
  marker_msg.scale.y = 0.1;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (Pose2DList::const_iterator it = vlist.begin(); it != vlist.end(); ++it)
  {
    geometry_msgs::Point p;
    p.x = it->x;
    p.y = it->y;
    p.z = 0;
    marker_msg.points.push_back(p);
  }
}

void VMarkerDetector::cacheData(const sensor_msgs::LaserScanConstPtr& scan)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;

  int i = trim_index_min_;
  int n = (trim_index_min_ && (trim_index_min_ >= trim_index_max_)) ?
          static_cast<int>(scan->ranges.size()) : trim_index_max_;

  for (int count = 0; i < n; ++count)
  {
    const double b = scan->angle_min + i * scan->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(count);

    if (++i == static_cast<int>(scan->ranges.size()))
    {
      if (trim_index_min_ && (trim_index_min_ >= trim_index_max_))
      {
        i = 0;
        n = trim_index_max_;
      }
    }
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}


/* static variables */
ros::Publisher VMarkerDetector::marker_viz_pub_;

}  // namespace marker_localization

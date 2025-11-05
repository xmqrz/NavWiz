/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <marker_localization/detectors/reflector.h>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace marker_localization
{

void ReflectorDetector::start()
{
  ros::NodeHandle nh;

  Detector::start(nh, true, true);

  // determine trim angles and range
  // possible string formats:
  //  {sensor}__{totalAngle} or {sensor}__{angleMin}_{angleMax} or {sensor}__r{rangeMax} or
  //  {sensor}__{totalAngle}__r{rangeMax} or {sensor}__{angleMin}_{angleMax}__r{rangeMax}
  // input units: angle -> degrees, range -> meters
  trim_angle_min_ = -M_PI;
  trim_angle_max_ = M_PI;
  trim_range_max_ = std::numeric_limits<float>::max();

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
      trim_range_max_ = fabs(range);
    }
  }

  std::string topic = getLaserTopic(sensor_);
  if (topic.length())
  {
    laser_sub_ = nh.subscribe(topic, 1, &ReflectorDetector::callbackLaserScan, this);
  }
  else
  {
    ROS_ERROR_STREAM("No sensor found: " << sensor_);
  }
}

void ReflectorDetector::stop()
{
  laser_sub_.shutdown();
  Detector::stop();
}

void ReflectorDetector::cacheData(const sensor_msgs::LaserScan& scan)
{
  data_cached_ = true;
  getTrimIndex(static_cast<int>(scan.intensities.size()),
               scan.angle_min, scan.angle_increment,
               trim_angle_min_, trim_angle_max_, trim_index_min_, trim_index_max_);
  ROS_DEBUG_STREAM(sensor_ << " trim angle: " << trim_angle_min_ << " -> " << trim_angle_max_
                           << " trim index: " << trim_index_min_ << " -> " << trim_index_max_);
}

void ReflectorDetector::processData(const sensor_msgs::LaserScan& scan)
{
  geometry_msgs::PoseArray markers;
  std::set<size_t> indices;
  getReflectorIndices(scan, markers, indices);

  // Publish all the reflectors detected by laser
  landmarks_pub_.publish(markers);

  // Publish all the reflective points detected by laser
  if (cloud_flatten_pub_.getNumSubscribers() > 0)
  {
    publishCloudFlatten(scan, indices);
  }

  handleMarkerPairs(markers, *config_reflector_.min_separation,
                    *config_reflector_.max_separation, *config_reflector_.max_position_deviation);
}

void ReflectorDetector::getReflectorIndices(const sensor_msgs::LaserScan& scan,
                                            geometry_msgs::PoseArray& markers, std::set<size_t>& indices)
{
  std::vector<int> reflectors;
  int start = -1, end;

  int i = trim_index_min_;
  int n = (trim_index_min_ && (trim_index_min_ >= trim_index_max_)) ?
          static_cast<int>(scan.intensities.size()) : trim_index_max_;

  while (i < n)
  {
    if (scan.intensities[i] >= *config_reflector_.intensity_threshold)
    {
      if (start < 0)
      {
        start = i;
      }
      end = i;
      indices.insert(i);
    }
    else
    {
      if (start >= 0)
      {
        int idx = getReflectorIndex(scan, start, end);
        if (idx >= 0)
        {
          reflectors.push_back(idx);
        }
        start = -1;
      }
    }

    if (++i == static_cast<int>(scan.intensities.size()))
    {
      if (trim_index_min_ && (trim_index_min_ >= trim_index_max_))
      {
        i = 0;
        n = trim_index_max_;
      }
    }
  }

  if (start >= 0)
  {
    int idx = getReflectorIndex(scan, start, end);
    if (idx >= 0)
    {
      reflectors.push_back(idx);
    }
    start = -1;
  }

  markers.header = scan.header;
  for (auto idx : reflectors)
  {
    double range = scan.ranges[idx] + *config_reflector_.radius;
    double angle = scan.angle_min + scan.angle_increment * idx;

    geometry_msgs::Pose p;
    p.position.x = range * std::cos(angle);
    p.position.y = range * std::sin(angle);
    p.position.z = 0;
    p.orientation.w = 1.0;
    markers.poses.push_back(p);
  }
}

int ReflectorDetector::getReflectorIndex(const sensor_msgs::LaserScan& scan, int start, int end)
{
  // Pythagorean theorem: aa = bb + cc - 2bc cos(A)
  double dth = scan.angle_increment * (end - start);
  double d = std::sqrt(scan.ranges[start] * scan.ranges[start]
                       + scan.ranges[end] * scan.ranges[end]
                       - scan.ranges[end] * scan.ranges[start] * std::cos(dth) * 2);

  // length of the reflective tape (~0.1m)
  if (d > *config_reflector_.min_width && d < *config_reflector_.max_width)
  {
    int size = static_cast<int>(scan.intensities.size());
    int idx_2 = -1;
    int idx = start + end;
    if (start > end)
    {
      idx += size;
    }
    if (idx & 1)
    {
      idx >>= 1;
      idx_2 = idx + 1;
      if (idx_2 >= size)
      {
        idx_2 -= size;
      }
    }
    else
    {
      idx >>= 1;
    }
    if (idx >= size)
    {
      idx -= size;
    }
    if (idx_2 >= 0)
    {
      if (scan.ranges[idx] > scan.ranges[idx_2])
      {
        idx = idx_2;
      }
    }
    if (scan.ranges[idx] <= trim_range_max_)
    {
      return idx;
    }
  }
  return -1;
}

}  // namespace marker_localization

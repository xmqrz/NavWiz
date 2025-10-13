/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include "agv05_lidar/lidar_watchdog.h"

void Watchdog::init(ros::NodeHandle& nh)
{
  // init pub sub
  scan_sub = nh.subscribe("/" + topic, 1, &Watchdog::callbackLaserScan, this);
}

void Watchdog::callbackLaserScan(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan = msg;
  scan_time = ros::Time::now();
}

WatchdogActivation Watchdog::processActivation(const double& now, const LaserArea& area, const agv05_msgs::ObstacleSensor& inspector_activation, const uint8_t& min_activation)
{
  if (!transform_initialized && scan)
  {
    if (!tfl)
    {
      tfl = boost::make_shared<tf::TransformListener>(ros::Duration(1.0));
    }
    try
    {
      tfl->waitForTransform("base", scan->header.frame_id, ros::Time(0), ros::Duration(0.5));
      tfl->lookupTransform("base", scan->header.frame_id, ros::Time(0), laser_transform);
      tfl.reset();
      transform_initialized = true;
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN_STREAM(ex.what());
    }
  }

  WatchdogActivation activation;

  // Check for malfunctions
  if (now - scan_time.toSec() > HEARTBEAT_TIMEOUT)
  {
    activation.malfunction = true;
    activation.error_msg = "Lidar Malfunction (Topic: " + topic + ")";
  }
  else if (!transform_initialized)
  {
    activation.malfunction = true;
    activation.error_msg = "Invalid Laser Transformation (Topic: " + topic + ")";
  }
  else
  {
    activation.malfunction = false;
    processLaserScan(area, inspector_activation, activation, min_activation);
  }

  return activation;
}

void Watchdog::processLaserScan(const LaserArea& area, const agv05_msgs::ObstacleSensor& inspector_activation, WatchdogActivation& activation, const uint8_t& min_activation)
{
  if (!scan || area.near.size() < 3 || area.middle.size() < 3 || area.far.size() < 3)
  {
    activation.near_blocked = false;
    activation.middle_blocked = false;
    activation.far_blocked = false;
    return;
  }

  // Process laser msg
  int near_hits = 0;
  int middle_hits = 0;
  int far_hits = 0;

  const sensor_msgs::LaserScan& scan_data = *scan;

  // Loop through each laser point
  for (size_t i = 0, n = scan_data.ranges.size(); i < n; ++i)
  {
    float r = scan_data.ranges[i];
    if (r <= scan_data.range_min || r >= scan_data.range_max) continue;  // discard invalid data
    float theta = scan_data.angle_min + scan_data.angle_increment * i;
    complex pt = transformPolarToBase(r, theta, laser_transform);

    // near zone
    if (!inspector_activation.near_blocked && near_hits < min_activation && computeHit(area.near, area.near_min, area.near_max, pt))
    {
      ++near_hits;
    }

    // middle zone
    if (!inspector_activation.middle_blocked && middle_hits < min_activation && computeHit(area.middle, area.middle_min, area.middle_max, pt))
    {
      ++middle_hits;
    }

    // far zone
    if (!inspector_activation.far_blocked && far_hits < min_activation && computeHit(area.far, area.far_min, area.far_max, pt))
    {
      ++far_hits;
    }
  }

  // Update activation
  activation.near_blocked = near_hits >= min_activation;
  activation.middle_blocked = middle_hits >= min_activation;
  activation.far_blocked = far_hits >= min_activation;
}

complex Watchdog::transformPolarToBase(const float& radius, const float& theta, const tf::StampedTransform& transform)
{
    complex pt = std::polar(radius, theta);
    tf::Vector3 pVector(pt.real(), pt.imag(), 0.0);
    pVector = transform(pVector);
    pt.real(pVector.x());
    pt.imag(pVector.y());
    return pt;
}

bool Watchdog::computeHit(const std::vector<complex>& region, const complex& min, const complex& max, complex& p)
{
  if (p.real() < min.real() || p.real() > max.real() || p.imag() < min.imag() || p.imag() > max.imag())
  {
    return false;
  }

  std::vector<complex>::const_iterator p_region = region.begin();
  std::vector<complex>::const_iterator p_region_end = region.end();

  bool inside = false;
  while (p_region != p_region_end)
  {
    complex i = *p_region;
    complex j = p_region == region.begin()? *(region.end() - 1) : *(p_region - 1);

    // vertex hit
    if (i.real() == p.real() && i.imag() == p.imag()) {
      return true;
    }

    // parallel border hit
    if (i.imag() == p.imag() && j.imag() == p.imag() && (i.real() > p.real()) != (j.real() > p.real())) {
      return true;
    }

    if ((i.imag() > p.imag()) != (j.imag() > p.imag())) {
      float x_intercept = (j.real() - i.real()) * (p.imag() - i.imag()) / (j.imag() - i.imag()) + i.real();

      // border hit
      if (p.real() == x_intercept) {
        return true;
      } else if (p.real() < x_intercept) {
        inside = !inside;
      }
    }

    p_region++;
  }
  return inside;
}

void LidarInspector::init(ros::NodeHandle& nh, std::string obstacle_name)
{
  dict topics_dict;
  if (!obtainParam(nh, param_prefix_.at(obstacle_name), topics_dict))
  {
    status.activation = "Disabled";
    status.activation_hint = "";
    return;
  }

  name = obstacle_name;

  // obstacle namespace
  boost::algorithm::replace_all(obstacle_name, " ", "_");
  boost::to_lower(obstacle_name);
  std::string obstacle_namespace = ros::this_node::getName();
  obstacle_namespace += "/" + obstacle_name;

  // init pub sub
  activation_pub = nh.advertise<agv05_msgs::ObstacleSensor>(obstacle_namespace + "/activation", 1, true);
  area_sub = nh.subscribe(obstacle_namespace + "/area", 1, &LidarInspector::callbackArea, this);

  // init watchdog
  watchdogs.clear();
  watchdogs.reserve(topics_dict.size());
  dict::const_iterator it = topics_dict.begin();
  for (; it != topics_dict.end(); it++)
  {
    watchdogs.push_back(Watchdog(it->first, it->second));
    watchdogs.back().init(nh);
  }

  enabled = true;
}

void LidarInspector::callbackArea(const std_msgs::UInt8& msg)
{
  area = msg.data;
}

void LidarInspector::processActivation(const double& now)
{
  if (!enabled)
  {
    return;
  }

  agv05_msgs::ObstacleSensor activation;
  status.area = area;
  LaserArea laserArea = laser_areas[status.area];

  if (!laserArea.valid)
  {
    activation.malfunction = true;
    activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MALFUNCTION;
    status.activation = "Invalid Area Configuration";
    status.activation_hint = "";
  }
  else
  {
    std::vector<Watchdog>::iterator it = watchdogs.begin();
    std::vector<Watchdog>::iterator it_end = watchdogs.end();
    for (; it != it_end; it++)
    {
      Watchdog& wd = *it;
      WatchdogActivation result = wd.processActivation(now, laserArea, activation, min_activation);
      if (result.malfunction)
      {
        activation.near_blocked = false;
        activation.middle_blocked = false;
        activation.far_blocked = false;

        activation.malfunction = true;
        activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MALFUNCTION;
        activation.hint = wd.hint;
        status.activation = result.error_msg;
        status.activation_hint = wd.hint;
        break;
      }
      activation.near_blocked = activation.near_blocked || result.near_blocked;
      activation.middle_blocked = activation.middle_blocked || result.middle_blocked;
      activation.far_blocked = activation.far_blocked || result.far_blocked;
      if (result.near_blocked) {
        activation.hint = wd.hint;
      }
    }

    if (!activation.malfunction)
    {
      // Summarize
      status.activation_hint = activation.hint;
      if (activation.near_blocked)
      {
        activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_NEAR_BLOCKED;
        status.activation = "Near blocked";
      }
      else if (activation.middle_blocked)
      {
        activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MIDDLE_BLOCKED;
        status.activation = "Middle blocked";
      }
      else if (activation.far_blocked)
      {
        activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_FAR_BLOCKED;
        status.activation = "Far blocked";
      }
      else
      {
        activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_NORMAL;
        status.activation = "No obstacle";
      }
    }
  }
  activation_pub.publish(activation);
}

void LidarInspector::updateConfig(const std::string data_list[NUM_AREAS - 1], const uint8_t& minimum_activation)
{
  min_activation = minimum_activation;
  laser_areas[0].valid = true;
  for (int i = 0; i < NUM_AREAS - 1; i++)
  {
    extractArea(data_list[i], laser_areas[i + 1]);
  }
}

bool LidarInspector::obtainParam(ros::NodeHandle& nh, std::string prefix, dict& topics_dict)
{
  std::string param_prefix = nh.resolveName(prefix);
  std::string topics;
  nh.getParam(param_prefix, topics);
  if (!topics.empty()) {
    std::istringstream iss(topics);
    std::string topic;
    while (iss >> topic)
    {
      topics_dict[topic] = topic;
    }
    return true;
  }

  std::vector<std::string> params;
  nh.getParamNames(params);
  std::vector<std::string>::const_iterator it = params.begin();
  for (; it != params.end(); it++) {
    std::string param_name = *it;
    if (param_name.rfind(param_prefix, 0) != 0) {
      continue;
    }
    std::string param_value;
    nh.getParam(param_name, param_value);
    if (param_value.empty()) {
      continue;
    }
    topics_dict[param_name.substr(param_prefix.size() + 1)] = param_value;
  }
  return topics_dict.size() > 0;
}

void LidarInspector::extractArea(const std::string & json, LaserArea & area)
{
  area.near.clear();
  area.middle.clear();
  area.far.clear();
  area.near_min = complex(1000.0f, 1000.0f);
  area.middle_min = complex(1000.0f, 1000.0f);
  area.far_min = complex(1000.0f, 1000.0f);
  area.near_max = complex(-1000.0f, -1000.0f);
  area.middle_max = complex(-1000.0f, -1000.0f);
  area.far_max = complex(-1000.0f, -1000.0f);
  area.valid = false;

  Json::Reader reader;
  Json::Value value;
  if (!reader.parse(json, value, false) || !value.isObject())
  {
    return;
  }

  extractPointListAndBBox(value["near"], area.near, area.near_min, area.near_max);
  extractPointListAndBBox(value["middle"], area.middle, area.middle_min, area.middle_max);
  extractPointListAndBBox(value["far"], area.far, area.far_min, area.far_max);

  if (area.near.size() >= 3 && area.middle.size() >= 3 && area.far.size() >= 3)
  {
    area.valid = true;
  }
}

void LidarInspector::extractPointListAndBBox(const Json::Value & value, std::vector<complex>& pointList, complex& min, complex& max)
{
  if (!value.isArray())
  {
    return;
  }
  int minX = 1000000, minY = 1000000;
  int maxX = -1000000, maxY = -1000000;
  for (Json::ValueConstIterator it = value.begin(), it_end = value.end(); it != it_end; ++it)
  {
    const Json::Value& vt = *it;
    if (vt.isArray() && vt.size() == 2 && vt[0].isInt() && vt[1].isInt())
    {
      int x = vt[0].asInt();
      int y = vt[1].asInt();

      if (x < minX)
      {
        minX = x;
      }
      if (x > maxX)
      {
        maxX = x;
      }
      if (y < minY)
      {
        minY = y;
      }
      if (y > maxY)
      {
        maxY = y;
      }

      // Data format in dyncfg storage is X(North) and Y(West), in unit millimeter. Therefore we
      // need to convert to ROS convention, X(North) and Y(West), in unit meter.
      pointList.push_back(complex(x * 0.001f, y * 0.001f));
    }
  }
  min.real(minX * 0.001f);
  min.imag(minY * 0.001f);
  max.real(maxX * 0.001f);
  max.imag(maxY * 0.001f);
}

const dict LidarInspector::param_prefix_ =
{
  {"Primary Lidar Obstacle Sensors", "primary_obstacle_scan_topics"},
  {"Secondary Lidar Obstacle Sensors", "secondary_obstacle_scan_topics"},
  {"Tertiary Lidar Obstacle Sensors", "tertiary_obstacle_scan_topics"},
  {"Quaternary Lidar Obstacle Sensors", "quaternary_obstacle_scan_topics"},
  {"Quinary Lidar Obstacle Sensors", "quinary_obstacle_scan_topics"}
};

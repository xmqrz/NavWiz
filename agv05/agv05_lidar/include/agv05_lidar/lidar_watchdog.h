/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#ifndef AGV05_LIDAR_LIDAR_WATCHDOG_H
#define AGV05_LIDAR_LIDAR_WATCHDOG_H

#include <boost/algorithm/string.hpp>
#include <complex>
#include <json/json.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <agv05_msgs/ObstacleSensor.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>

/* Constants */
const int NUM_AREAS = 32;
const float HEARTBEAT_TIMEOUT = 0.5;

/* Data structure */
typedef std::complex<float> complex;
typedef std::map<std::string, std::string> dict;
struct LaserArea
{
  std::vector<complex> near;
  std::vector<complex> middle;
  std::vector<complex> far;
  complex near_min;
  complex near_max;
  complex middle_min;
  complex middle_max;
  complex far_min;
  complex far_max;
  bool valid;
};

struct LaserStatus
{
  int area;
  std::string activation;
  std::string activation_hint;
};

struct WatchdogActivation
{
  bool near_blocked;
  bool middle_blocked;
  bool far_blocked;
  bool malfunction;
  std::string error_msg;
};

class Watchdog
{
public:
  std::string hint;

  Watchdog(const std::string& _topic, const std::string& _hint):
    topic(_topic),
    hint(_hint),
    transform_initialized(false)
  {};
  void init(ros::NodeHandle& nh);
  WatchdogActivation processActivation(const double& now, const LaserArea& area, const agv05_msgs::ObstacleSensor& inspector_activation, const uint8_t& min_activation);
  complex transformPolarToBase(const float& radius, const float& theta, const tf::StampedTransform& transform);
  bool computeHit(const std::vector<complex>& region, const complex& minBBox, const complex& maxBBox, complex& p);

private:
  std::string topic;
  ros::Subscriber scan_sub;
  ros::Time scan_time;
  sensor_msgs::LaserScanConstPtr scan;
  bool transform_initialized;
  tf::StampedTransform laser_transform;
  boost::shared_ptr<tf::TransformListener> tfl;

  void callbackLaserScan(const sensor_msgs::LaserScanConstPtr& msg);
  void processLaserScan(const LaserArea& area, const agv05_msgs::ObstacleSensor& inspector_activation, WatchdogActivation& activation, const uint8_t& min_activation);
};

class LidarInspector
{
public:
  bool enabled;
  LaserStatus status;

  LidarInspector():
    area(0),
    enabled(false),
    min_activation(4)
  {
    status.area = 0;
    status.activation = "No obstacle";
  };
  void init(ros::NodeHandle& nh, std::string obstacle_name);
  void processActivation(const double& now);
  void updateConfig(const std::string data_list[NUM_AREAS - 1], const uint8_t& minimum_activation);
  static bool obtainParam(ros::NodeHandle& nh, std::string prefix, dict& topic_dict);

  static const dict param_prefix_;

private:
  ros::Publisher activation_pub;
  ros::Subscriber area_sub;
  std::string name;
  uint8_t area;
  uint8_t min_activation;
  LaserArea laser_areas[NUM_AREAS];
  std::vector<Watchdog> watchdogs;

  void callbackArea(const std_msgs::UInt8& msg);
  void extractArea(const std::string& json, LaserArea& area);
  void extractPointListAndBBox(const Json::Value& value, std::vector<complex>& pointList, complex& min, complex& max);
};

#endif  // AGV05_LIDAR_LIDAR_WATCHDOG_H

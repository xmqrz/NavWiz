/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <agv05_lidar/lidar_watchdog.h>
#include <json/json.h>
#include <sstream>

#include "agv05_navx/components.h"


namespace agv05
{

DynamicObstacleSensor::DynamicObstacleSensor()
{
  ros::NodeHandle nh_private("~");
  srv_.resize(0);
  srv_.push_back(nh_private.advertiseService("enum_src", &DynamicObstacleSensor::enumServerCallback, this));
}

void DynamicObstacleSensor::setObstacleSensors(const std::string& ns)
{
  ros::NodeHandle nh, nh_private("~");
  dict topics_dict;
  std::string param, prefix, observation_sources = "";
  nh_private.getParam("dynplan_obstacle_sensors", param);
  std::istringstream iss_param(param);

  while (std::getline(iss_param, prefix, ','))
  {
    LidarInspector::obtainParam(nh, prefix, topics_dict);
  }

  for (auto& topic : topics_dict)
  {
    std::istringstream iss_topic(topic.first);
    std::string sensor;
    std::getline(iss_topic, sensor, '/');
    if (sensor.size())
    {
      if (observation_sources.size())
      {
        observation_sources += ' ';
      }
      observation_sources += sensor;
    }
  }

  ns_ = ns + "/obstacle_layer";
  nh_private.setParam(ns_ + "/observation_sources", observation_sources);
}

void DynamicObstacleSensor::clearObstacle()
{
  ros::NodeHandle nh("~" + ns_);
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("reset_map");
  std_srvs::Empty srv;
  client.call(srv);
}

bool DynamicObstacleSensor::enumServerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ros::NodeHandle nh("agv05_obstacle_sensor");
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("enum_src");
  std_srvs::Trigger srv;

  if (!client.call(srv) || !srv.response.success)
  {
    return false;
  }

  Json::Value enum_src, enum_src_navx = Json::objectValue;
  Json::Reader reader;
  Json::FastWriter writer;

  reader.parse(srv.response.message, enum_src);
  Json::Value::Members members = enum_src.getMemberNames();
  Json::Value& v = enum_src[members[0]];
  for (int i = 0; i < v.size(); ++i)
  {
    std::string key = v[i][1].asString();
    dict::const_iterator it = LidarInspector::param_prefix_.find(key);
    if (it != LidarInspector::param_prefix_.end())
    {
      Json::Value item;
      item.append(it->second);
      item.append(key);
      enum_src_navx[members[0]].append(item);
    }
  }

  res.success = true;
  res.message = writer.write(enum_src_navx);
  return true;
}

}  // namespace agv05

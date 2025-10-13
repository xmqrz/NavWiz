/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/components.h"


namespace agv05
{

LaserSensor::LaserSensor(ros::NodeHandle& nh) :
  profile_(1), area_(agv05_msgs::ObstacleSensorArea::AREA_DISABLE), stop_after_(0)
{
  area_pub_ = nh.advertise<agv05_msgs::ObstacleSensorArea>("agv05/obstacle_sensor/area", 1, true);

  area_sub_ = nh.subscribe(area_pub_.getTopic(), 1, &LaserSensor::handleArea, this);
  activation_sub_ = nh.subscribe("agv05/obstacle_sensor/activation", 10, &LaserSensor::handleActivation, this);
}

void LaserSensor::selectArea(uint8_t area, uint8_t profile)
{
  if (!stop_after_.isZero())
  {
    ROS_DEBUG("self-cancel stop delay: %fs", (stop_after_ - ros::Time::now()).toSec());
    stop_after_ = ros::Time(0);
  }

  if (profile == 0xff)
  {
    profile = profile_;
  }
  agv05_msgs::ObstacleSensorArea config;
  config.profile = profile;
  config.area = area;
  area_pub_.publish(config);
}

LaserSensor::StopDelay LaserSensor::stopDelay(double delay_sec)
{
  if (delay_sec > 0.0)
  {
    stop_after_ = ros::Time::now() + ros::Duration(delay_sec);
  }
  else if (stop_after_.isZero())
  {
    return STOP_DELAY_OFF;
  }
  else if (stop_after_ < ros::Time::now())
  {
    return STOP_DELAY_DONE;
  }
  return STOP_DELAY_ON;
}

void LaserSensor::handleArea(const agv05_msgs::ObstacleSensorArea& msg)
{
  area_ = msg.area;
  if (!stop_after_.isZero())
  {
    ROS_WARN("non-self-cancel stop delay: %fs", (stop_after_ - ros::Time::now()).toSec());
    stop_after_ = ros::Time(0);
  }
}

void LaserSensor::handleActivation(const agv05_msgs::ObstacleSensor& activation)
{
  activation_ = activation;
  timeout_ = ros::Time::now() + ros::Duration(SAFETY_HEARTBEAT_TIMEOUT);
}

}  // namespace agv05

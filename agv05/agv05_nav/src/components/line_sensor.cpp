/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/components.h"

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>


namespace agv05
{

LineSensor::LineSensor(ros::NodeHandle& nh)
{
  front_calibrate_pub_ = nh.advertise<std_msgs::Bool>("agv05/line_sensor/front_calibration", 1, true);
  rear_calibrate_pub_ = nh.advertise<std_msgs::Bool>("agv05/line_sensor/rear_calibration", 1, true);

  front_preferred_side_pub_ = nh.advertise<std_msgs::UInt8>("agv05/line_sensor/front_prefer_side", 1, true);
  rear_preferred_side_pub_ = nh.advertise<std_msgs::UInt8>("agv05/line_sensor/rear_prefer_side", 1, true);

  front_data_sub_ = nh.subscribe("agv05/line_sensor/front_sensor", 10, &LineSensor::handleMsbFront, this);
  rear_data_sub_ = nh.subscribe("agv05/line_sensor/rear_sensor", 10, &LineSensor::handleMsbRear, this);
}

void LineSensor::activateCalibration(bool activate)
{
  std_msgs::Bool msg;
  msg.data = activate;
  front_calibrate_pub_.publish(msg);
  rear_calibrate_pub_.publish(msg);
}

void LineSensor::setPreferredSide(uint8_t preferred_side)
{
  std_msgs::UInt8 msg;
  msg.data = preferred_side;
  front_preferred_side_pub_.publish(msg);
  rear_preferred_side_pub_.publish(msg);
}

agv05_msgs::LineSensor LineSensor::getFrontData()
{
  if (front_data_.enable && !front_data_.sensor_error && front_timeout_ < ros::Time::now().toSec())
  {
    front_data_.sensor_error = agv05_msgs::LineSensor::COMMUNICATION_ERROR;
  }
  return front_data_;
}

agv05_msgs::LineSensor LineSensor::getRearData()
{
  if (rear_data_.enable && !rear_data_.sensor_error && rear_timeout_ < ros::Time::now().toSec())
  {
    rear_data_.sensor_error = agv05_msgs::LineSensor::COMMUNICATION_ERROR;
  }
  return rear_data_;
}

void LineSensor::handleMsbFront(const agv05_msgs::LineSensor& msb_data)
{
  front_data_ = msb_data;
  front_timeout_ = ros::Time::now().toSec() + SAFETY_HEARTBEAT_TIMEOUT;
}

void LineSensor::handleMsbRear(const agv05_msgs::LineSensor& msb_data)
{
  rear_data_ = msb_data;
  rear_timeout_ = ros::Time::now().toSec() + SAFETY_HEARTBEAT_TIMEOUT;
}

}  // namespace agv05

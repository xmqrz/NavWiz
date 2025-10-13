/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/components.h"


namespace agv05
{

Safety::Safety(ros::NodeHandle& nh)
{
  mute_bumper_pub_ = nh.advertise<std_msgs::Bool>("agv05/safety/mute/bumper", 1);
  nav_trigger_pub_ = nh.advertise<std_msgs::Bool>("agv05/safety/nav_trigger", 1, true);
  safety_internal_message_pub_ = nh.advertise<std_msgs::String>("agv05/safety/safety_internal_message", 1, true);
  safety_heartbeat_pub_ = nh.advertise<std_msgs::UInt8>("agv05/safety/heartbeat/" + ros::this_node::getName(), 1);

  motor_fault_hint_sub_ = nh.subscribe("agv05/motor/fault_hint", 1, &Safety::handleMotorFaultHint, this);
  safety_system_hint_sub_ = nh.subscribe("agv05/safety/system_hint", 1, &Safety::handleSafetySystemHint, this);
  safety_sub_ = nh.subscribe("agv05/safety/safety_triggers", 10, &Safety::handleSafety, this);

  // Set all safety to triggered state until the actual state is received.
  safety_trigger_.bumper_front = true;
  safety_trigger_.bumper_rear = true;
  safety_trigger_.emergency_button = true;
  safety_trigger_.safety_in_1 = true;
  safety_trigger_.safety_in_2 = true;
  safety_trigger_.motor_fault = true;
  safety_trigger_.wheel_slippage = true;
  safety_trigger_.charger_connected = true;
  safety_trigger_.system_error = true;
}

void Safety::publishMuteBumper(bool mute)
{
  if (mute_bumper_ == mute)
  {
    return;
  }
  std_msgs::Bool msg;
  msg.data = mute;
  mute_bumper_pub_.publish(msg);
  mute_bumper_ = mute;
}

void Safety::publishNavTrigger(bool trigger, const std::string& safety_internal_message)
{
  if (safety_internal_message_.data != safety_internal_message)
  {
    safety_internal_message_.data = safety_internal_message;
    safety_internal_message_pub_.publish(safety_internal_message_);
  }

  if (nav_trigger_ == trigger)
  {
    return;
  }
  std_msgs::Bool msg;
  msg.data = trigger;
  nav_trigger_pub_.publish(msg);
  nav_trigger_ = trigger;
}

void Safety::publishSafetyHeartbeat()
{
  double now = ros::Time::now().toSec();
  if (safety_heartbeat_timeout_ < now)
  {
    safety_heartbeat_timeout_ = now + SAFETY_HEARTBEAT_TIMEOUT * 0.2;
    safety_heartbeat_pub_.publish(std_msgs::UInt8());
  }
  if (safety_trigger_timeout_ < now)
  {
    safety_trigger_.system_error = true;
  }
}

void Safety::handleMotorFaultHint(const std_msgs::String& msg)
{
  motor_fault_hint_ = msg.data;
}

void Safety::handleSafetySystemHint(const std_msgs::String& msg)
{
  safety_system_hint_ = msg.data;
}

void Safety::handleSafety(const agv05_msgs::SafetyTriggers& safety_trigger)
{
  safety_trigger_ = safety_trigger;
  safety_trigger_timeout_ = ros::Time::now().toSec() + SAFETY_HEARTBEAT_TIMEOUT;
}

}  // namespace agv05

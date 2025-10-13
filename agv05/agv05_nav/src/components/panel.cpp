/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/components.h"

#include <agv05_msgs/AudioControl.h>
#include <agv05_msgs/LedControl.h>
#include <agv05_msgs/NavActionAction.h>


namespace agv05
{

Panel::Panel(ros::NodeHandle& nh) :
  alarm_activated_(false)
{
  alarm_control_pub_ = nh.advertise<agv05_msgs::AudioControl>("agv05/audio/alarm_control", 1, true);
  led_control_pub_ = nh.advertise<agv05_msgs::LedControl>("agv05/led/control/navigation", 1, true);
}

void Panel::update(uint8_t status, uint8_t led_side)
{
  typedef agv05_msgs::NavActionFeedback Feedback;

  switch (status)
  {
  case Feedback::STATUS_NORMAL:
    switch (led_side)
    {
    case 0:
      setLed(agv05_msgs::LedControl::NORMAL);
      break;
    case 1:
      setLed(agv05_msgs::LedControl::NORMAL_LEFT);
      break;
    case 2:
      setLed(agv05_msgs::LedControl::NORMAL_RIGHT);
      break;
    }
    setAlarm(false);
    break;
  case Feedback::STATUS_PAUSED:
    setLed(agv05_msgs::LedControl::PAUSE);
    setAlarm(false);
    break;
  case Feedback::STATUS_NAVIGATION_FAILED:
  case Feedback::STATUS_OUT_OF_LINE:
  case Feedback::STATUS_BUMPER_BLOCKED:
  case Feedback::STATUS_EXTERNAL_SAFETY_TRIGGER:
  case Feedback::STATUS_EMERGENCY_BUTTON_PRESSED:
  case Feedback::STATUS_CHARGER_CONNECTED:
  case Feedback::STATUS_LASER_MALFUNCTION:
  case Feedback::STATUS_LINE_SENSOR_ERROR:
  case Feedback::STATUS_DRIVE_OVERLIMIT_ERROR:
  case Feedback::STATUS_MOTOR_FAULT:
  case Feedback::STATUS_WHEEL_SLIPPAGE:
  case Feedback::STATUS_SYSTEM_ERROR:
  case Feedback::STATUS_PLAN_EMPTY:
  case Feedback::STATUS_OBSTACLE_BLOCKED:
    setLed(agv05_msgs::LedControl::ERROR);
    setAlarm(true);
    break;
  case Feedback::STATUS_OBSTACLE_IN_RANGE:
    setLed(agv05_msgs::LedControl::NORMAL_WARNING);
    setAlarm(false);
    break;
  case Feedback::STATUS_WAIT_TRAFFIC:
    setLed(agv05_msgs::LedControl::ACTION);
    setAlarm(false);
    break;
  case Feedback::STATUS_SAFETY_TRIGGERED:
    setLed(agv05_msgs::LedControl::SAFETY_TRIGGER);
    setAlarm(true);
    break;
  }
}

void Panel::setAlarm(bool activate)
{
  if (alarm_activated_ == activate)
  {
    return;
  }

  agv05_msgs::AudioControl ac;
  if (activate)
  {
    ac.operation = ac.PLAY;
  }
  else
  {
    ac.operation = ac.STOP;
  }
  alarm_control_pub_.publish(ac);
  alarm_activated_ = activate;
}

void Panel::setLed(uint8_t mode)
{
  if (led_mode_ == mode)
  {
    return;
  }
  agv05_msgs::LedControl lc;
  lc.mode = mode;
  led_control_pub_.publish(lc);
  led_mode_ = mode;
}

}  // namespace agv05

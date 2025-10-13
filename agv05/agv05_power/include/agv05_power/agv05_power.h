/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#ifndef AGV05_POWER_AGV05_POWER_H
#define AGV05_POWER_AGV05_POWER_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <agv05_msgs/BatteryState.h>
#include <agv05_msgs/LedControl.h>
#include <agv05_msgs/PowerSource.h>
#include <agv05_power/PowerConfig.h>
#include <std_msgs/Bool.h>


namespace agv05
{

class Filter
{
public:
  Filter(bool initial_state, float threshold) :
    threshold_(threshold)
  {
    setState(initial_state);
  }

  bool update(bool state, float elapsed_time)
  {
    if (state_ != state)
    {
      accumulated_ += elapsed_time;
      if (accumulated_ > threshold_)
      {
        setState(state);
      }
    }
    else
    {
      accumulated_ = 0;
    }
    return state_;
  }

  bool getState()
  {
    return state_;
  }
  void setState(bool state)
  {
    state_ = state;
    accumulated_ = 0;
  }
  float getRemaining()
  {
    return threshold_ - accumulated_;
  }

private:
  bool state_;
  float accumulated_;
  float threshold_;
};


class Agv05Power
{
public:
  Agv05Power();
  void process(float period);

private:
  void processPercentage(float period);
  void processState(float period);

  void enterShutdown();

  void diagnosticBatteryStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void callbackPowerSource(const agv05_msgs::PowerSource& msg)
  {
    source_ = msg;
  }
  void callbackBatteryState(const agv05_msgs::BatteryState& msg)
  {
    battery_state_ = msg;
  }
  void callbackConfig(agv05_power::PowerConfig& config, uint32_t level)
  {
    ROS_INFO_STREAM("agv05_power: config received");
    config_ = config;
    if (config_.disable_led_while_charging)
    {
      led_control_.mode = agv05_msgs::LedControl::OFF;
      led_control_pub_.publish(led_control_);
    }
  }

private:
  /* ROS publishers */
  ros::Publisher charger_connected_pub_;
  ros::Publisher led_control_pub_;
  ros::Publisher led_low_batt_pub_;
  ros::Publisher shutdown_pub_;

  /* ROS subscribers */
  ros::Subscriber source_sub_;
  ros::Subscriber battery_state_sub_;

  /* Dynamic reconfigure server */
  dynamic_reconfigure::Server<agv05_power::PowerConfig> ds_;

  /* ROS diagnostic */
  double expected_process_frequency_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::FrequencyStatus diagnostic_frequency_;

  /* Data */
  float low_battery_level_;
  agv05_power::PowerConfig config_;
  agv05_msgs::PowerSource source_;
  agv05_msgs::BatteryState battery_state_;
  agv05_msgs::LedControl led_control_;
  bool battery_low_;
  bool battery_empty_;

  Filter shutdown_period_;
};

}  // namespace agv05

#endif  // AGV05_POWER_AGV05_POWER_H

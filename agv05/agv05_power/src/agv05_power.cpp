/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#include "agv05_power/agv05_power.h"

#define LOOP_FREQUENCY 2


namespace agv05
{

Agv05Power::Agv05Power() :
  expected_process_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_process_frequency_,
                        &expected_process_frequency_), "Process Frequency"),
  battery_low_(false), battery_empty_(false),
  shutdown_period_(false, 600.0f)  // 10 mins
{
  battery_state_.percentage = -1.0f;

  ros::NodeHandle nh;
  ros::NodeHandle nh_power("agv05/power");
  ros::NodeHandle private_nh("~");

  // ROS publishers
  charger_connected_pub_ = nh.advertise<std_msgs::Bool>("agv05/safety/charger_connected", 1, true);
  led_control_pub_ = nh.advertise<agv05_msgs::LedControl>("agv05/led/control/power", 1, true);
  led_low_batt_pub_ = nh.advertise<std_msgs::Bool>("agv05/panel/control/led_low_batt", 1, true);
  shutdown_pub_ = nh.advertise<std_msgs::Bool>("shutdown", 1, true);

  // ROS subscribers
  source_sub_ = nh_power.subscribe("source", 1, &Agv05Power::callbackPowerSource, this);
  battery_state_sub_ = nh_power.subscribe("battery_state", 1, &Agv05Power::callbackBatteryState, this);

  // ROS parameters
  private_nh.param("low_battery_level", low_battery_level_, 30.0f);  // from agv05_executor

  // dynamic reconfigure
  ds_.setCallback(boost::bind(&Agv05Power::callbackConfig, this, _1, _2));

  // diagnostic updater
  diagnostic_updater_.setHardwareID("AGV05");
  diagnostic_updater_.add(diagnostic_frequency_);
  diagnostic_updater_.add("Status", this, &Agv05Power::diagnosticBatteryStatus);
}

void Agv05Power::process(float period)
{
  processPercentage(period);
  processState(period);

  if (!config_.disable_led_while_charging)
  {
    led_control_pub_.publish(led_control_);
  }

  std_msgs::Bool msg;
  msg.data = source_.manual_charger_connected;
  charger_connected_pub_.publish(msg);

  msg.data = battery_low_;
  led_low_batt_pub_.publish(msg);

  diagnostic_frequency_.tick();
  diagnostic_updater_.update();
}

void Agv05Power::processPercentage(float period)
{
  if (battery_state_.percentage < 0)
  {
    return;
  }

  // battery low detection
  battery_low_ = battery_state_.percentage < low_battery_level_;
}

void Agv05Power::processState(float period)
{
  switch (battery_state_.state)
  {
  case agv05_msgs::BatteryState::AUTO_CHARGING:
  case agv05_msgs::BatteryState::MANUAL_CHARGING:
    led_control_.mode = agv05_msgs::LedControl::CHARGING;
    battery_empty_ = false;
    shutdown_period_.setState(false);
    break;

  default:
    led_control_.mode = agv05_msgs::LedControl::OFF;

    // battery empty detection
    battery_empty_ = battery_state_.percentage < config_.auto_shutdown_percentage &&
                     source_.battery1_voltage > 0.0f;  // prevent shutdown when battery not detected

    if (battery_empty_)
    {
      ROS_ERROR_THROTTLE(60.0, "Battery is empty. System will shutdown if battery is not being charged immediately.");
    }
    break;
  }

  // update shutdown counter
  if (shutdown_period_.update(battery_empty_, period))
  {
    enterShutdown();
  }
}

void Agv05Power::enterShutdown()
{
  ROS_ERROR("System will shutdown due to battery empty.");

  std_msgs::Bool msg;
  msg.data = true;
  shutdown_pub_.publish(msg);

  ros::Duration(3.0).sleep();
  system("sudo poweroff");
}

void Agv05Power::diagnosticBatteryStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::string battery_state;
  switch (battery_state_.state)
  {
  case agv05_msgs::BatteryState::IDLE:
    battery_state = "Idle";
    break;
  case agv05_msgs::BatteryState::AUTO_CHARGER_ENGAGING:
    battery_state = "Auto Charger Engaging";
    break;
  case agv05_msgs::BatteryState::AUTO_CHARGING:
    battery_state = "Auto Charging";
    break;
  case agv05_msgs::BatteryState::AUTO_CHARGER_CONNECTION_FAILED:
    battery_state = "Auto Charger Connection Failed";
    break;
  case agv05_msgs::BatteryState::AUTO_CHARGING_COMPLETED:
    battery_state = "Auto Charging Completed";
    break;
  case agv05_msgs::BatteryState::AUTO_CHARGER_DISENGAGING:
    battery_state = "Auto Charger Disengaging";
    break;
  case agv05_msgs::BatteryState::MANUAL_CHARGING:
    battery_state = "Manual Charging";
    break;
  case agv05_msgs::BatteryState::BATTERY_ERROR:
    battery_state = "Battery Error";
    break;
  default:
    battery_state = "Unknown ";
    battery_state += std::to_string(battery_state_.state);
    break;
  }

  stat.add("Battery State", battery_state);
  stat.addf("Battery Percentage", "%.1f", battery_state_.percentage);
  stat.addf("Battery1 Voltage", "%.2f", source_.battery1_voltage);
  stat.addf("Battery2 Voltage", "%.2f", source_.battery2_voltage);
  stat.addf("Battery Current", "%.3f", source_.battery_current);
  stat.addf("Auto Charger Voltage", "%.2f", source_.auto_charger_voltage);
  stat.addf("Manual Charger Voltage", "%.2f", source_.manual_charger_voltage);
  stat.add("Manual Charger Connected", static_cast<bool>(source_.manual_charger_connected));
  stat.add("Battery Low", battery_low_);
  stat.add("Battery Empty", battery_empty_);

  if (battery_empty_)
  {
    stat.add("Battery Empty Shutdown Counter", static_cast<int>(shutdown_period_.getRemaining()));
  }
  else
  {
    stat.add("Battery Empty Shutdown Counter", "-");
  }

  if (battery_state_.state == agv05_msgs::BatteryState::BATTERY_ERROR)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "BATTERY ERROR");
  }
  else if (battery_empty_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "BATTERY EMPTY");
  }
  else if (battery_low_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "BATTERY LOW");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
}

}  // namespace agv05

/* main function */
int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "agv05_power");
  ROS_INFO_STREAM("agv05_power start");

  agv05::Agv05Power power;

  ros::Rate r(LOOP_FREQUENCY);
  while (ros::ok())
  {
    ros::spinOnce();
    power.process(1.0f / LOOP_FREQUENCY);
    r.sleep();
  }
}

/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#ifndef AGV05_SAFETY_AGV05_SAFETY_H
#define AGV05_SAFETY_AGV05_SAFETY_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <agv05_msgs/SafetyTriggers.h>
#include <agv05_safety/SafetyConfig.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>


namespace agv05
{

typedef std::map<std::string, std::string> dict;

class Agv05Safety
{
public:
  Agv05Safety();
  void process();

private:
  void obtainParam(ros::NodeHandle& nh, std::string prefix, dict& topics_dict);
  void callbackInput(uint8_t agv05_msgs::SafetyTriggers::*flag, const std_msgs::Bool& msg);
  void callbackSafetyExternal1(int index, const std_msgs::Bool& msg);
  void callbackSafetyExternal2(int index, const std_msgs::Bool& msg);
  void callbackSafetyHeartbeat(int index, const std_msgs::UInt8& msg);
  void callbackConfig(agv05_safety::SafetyConfig &config, uint32_t level);
  std::string checkHeartbeat();
  bool checkTimeJump();
  bool checkSystemOverload();
  bool summarizeSafetyTriggers(const agv05_msgs::SafetyTriggers& safety_triggers);
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

  boost::function<void(const std_msgs::Bool&)> callbackInput(uint8_t agv05_msgs::SafetyTriggers::*flag)
  {
    return boost::bind(&Agv05Safety::callbackInput, this, flag, _1);
  }
  boost::function<void(const std_msgs::Bool&)> callbackSafetyExternal1(int index)
  {
    return boost::bind(&Agv05Safety::callbackSafetyExternal1, this, index, _1);
  }
  boost::function<void(const std_msgs::Bool&)> callbackSafetyExternal2(int index)
  {
    return boost::bind(&Agv05Safety::callbackSafetyExternal2, this, index, _1);
  }
  boost::function<void(const std_msgs::UInt8&)> callbackSafetyHeartbeat(int index)
  {
    return boost::bind(&Agv05Safety::callbackSafetyHeartbeat, this, index, _1);
  }

private:
  /* ROS publishers */
  ros::Publisher safety_triggers_pub_;
  ros::Publisher safety_trigger_pub_;
  ros::Publisher safety_external_message_pub_;
  ros::Publisher safety_system_hint_pub_;
  ros::Publisher peripherals_relay_control_pub_;

  /* ROS subscribers */
  ros::Subscriber input_subs_[7];
  std::vector<ros::Subscriber> safety_external1_subs_;
  std::vector<ros::Subscriber> safety_external2_subs_;
  std::vector<ros::Subscriber> safety_heartbeat_subs_;

  /* Dynamic reconfigure server */
  dynamic_reconfigure::Server<agv05_safety::SafetyConfig> ds_;

  /* ROS diagnostic */
  double expected_process_frequency_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::FrequencyStatus diagnostic_frequency_;

  /* Data */
  agv05_safety::SafetyConfig config_;
  agv05_msgs::SafetyTriggers safety_triggers_;
  agv05_msgs::SafetyTriggers safety_triggers_hold_;
  std_msgs::Bool safety_trigger_;
  std_msgs::String safety_external_message_;
  std_msgs::String safety_external_message_hold_;
  std_msgs::String safety_system_hint_;

  std::vector<bool> safety_external1_states_;
  std::vector<bool> safety_external2_states_;
  std::vector<std::string> safety_external1_messages_;
  std::vector<std::string> safety_external2_messages_;

  std::vector<uint8_t> safety_heartbeat_states_;
  std::vector<double> safety_heartbeat_timeouts_;
  std::vector<std::string> safety_heartbeat_messages_;
};

}  // namespace agv05

#endif  // AGV05_SAFETY_AGV05_SAFETY_H

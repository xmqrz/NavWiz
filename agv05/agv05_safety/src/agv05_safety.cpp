/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#include "agv05_safety/agv05_safety.h"
#include <fstream>

#define LOOP_FREQUENCY 20
#define TIME_JUMP_TOLERANCE 0.2
#define SAFETY_HEARTBEAT_TIMEOUT 0.5


namespace agv05
{

Agv05Safety::Agv05Safety() :
  expected_process_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_process_frequency_,
                        &expected_process_frequency_, 0.1, 1), "Process Frequency")
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_power("agv05/power");
  ros::NodeHandle nh_safety("agv05/safety");
  ros::NodeHandle private_nh("~");

  /* ROS Publishers */
  safety_triggers_pub_ = nh_safety.advertise<agv05_msgs::SafetyTriggers>("safety_triggers", 1, true);
  safety_trigger_pub_ = nh_safety.advertise<std_msgs::Bool>("safety_trigger", 1, true);
  safety_external_message_pub_ = nh_safety.advertise<std_msgs::String>("safety_external_message", 1, true);
  safety_system_hint_pub_ = nh_safety.advertise<std_msgs::String>("system_hint", 1, true);
  peripherals_relay_control_pub_ = nh_power.advertise<std_msgs::Bool>("peripherals_relay_control", 1, true);

  /* ROS Subscribers */
  typedef agv05_msgs::SafetyTriggers ST;
  input_subs_[0] = nh_safety.subscribe<std_msgs::Bool>("bumper_front", 1, callbackInput(&ST::bumper_front));
  input_subs_[1] = nh_safety.subscribe<std_msgs::Bool>("bumper_rear", 1, callbackInput(&ST::bumper_rear));
  input_subs_[2] = nh_safety.subscribe<std_msgs::Bool>("emergency_button", 1, callbackInput(&ST::emergency_button));
  input_subs_[3] = nh_safety.subscribe<std_msgs::Bool>("motor_fault", 1, callbackInput(&ST::motor_fault));
  input_subs_[4] = nh_safety.subscribe<std_msgs::Bool>("wheel_slippage", 1, callbackInput(&ST::wheel_slippage));
  input_subs_[5] = nh_safety.subscribe<std_msgs::Bool>("charger_connected", 1, callbackInput(&ST::charger_connected));
  input_subs_[6] = nh_safety.subscribe<std_msgs::Bool>("nav_trigger", 1, callbackInput(&ST::nav_trigger));

  // list of safety external 1 input topics separated by whitespace
  dict safety_external_dict;
  obtainParam(private_nh, "safety_external1_topics", safety_external_dict);
  if (safety_external_dict.empty())
  {
    safety_external_dict["agv05/safety/safety_external1"] = "";
  }

  dict::const_iterator it = safety_external_dict.begin();
  safety_external1_messages_.reserve(safety_external_dict.size());
  for (; it != safety_external_dict.end(); it++)
  {
    int i = safety_external1_subs_.size();
    ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(it->first, 1, callbackSafetyExternal1(i));
    safety_external1_subs_.push_back(sub);
    safety_external1_messages_.push_back(it->second);
  }
  safety_external1_states_.resize(safety_external1_subs_.size());

  // list of safety external 2 input topics separated by whitespace
  safety_external_dict.clear();
  obtainParam(private_nh, "safety_external2_topics", safety_external_dict);
  if (safety_external_dict.empty())
  {
    safety_external_dict["agv05/safety/safety_external2"] = "";
  }

  it = safety_external_dict.begin();
  safety_external2_messages_.reserve(safety_external_dict.size());
  for (; it != safety_external_dict.end(); it++)
  {
    int i = safety_external2_subs_.size();
    ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(it->first, 1, callbackSafetyExternal2(i));
    safety_external2_subs_.push_back(sub);
    safety_external2_messages_.push_back(it->second);
  }

  safety_external2_states_.resize(safety_external2_subs_.size());

  // list of safety heartbeat input topics separated by whitespace
  safety_external_dict.clear();
  obtainParam(private_nh, "safety_heartbeat_topics", safety_external_dict);

  ros::NodeHandle nh_heartbeat("agv05/safety/heartbeat");
  safety_heartbeat_states_.resize(safety_external_dict.size());
  safety_heartbeat_timeouts_.resize(safety_external_dict.size());
  safety_heartbeat_messages_.reserve(safety_external_dict.size());
  for (it = safety_external_dict.begin(); it != safety_external_dict.end(); ++it)
  {
    safety_heartbeat_subs_.push_back(
      nh_heartbeat.subscribe<std_msgs::UInt8>(it->first, 1, callbackSafetyHeartbeat(safety_heartbeat_subs_.size())));
    safety_heartbeat_messages_.push_back(it->second);
  }

  // dynamic reconfigure
  ds_.setCallback(boost::bind(&Agv05Safety::callbackConfig, this, _1, _2));

  // diagnostic updater
  diagnostic_updater_.setHardwareID("AGV05");
  diagnostic_updater_.add(diagnostic_frequency_);
  diagnostic_updater_.add("Status", this, &Agv05Safety::diagnosticStatus);
}

void Agv05Safety::obtainParam(ros::NodeHandle& nh, std::string prefix, dict& topics_dict)
{
  std::string param_prefix = nh.resolveName(prefix);
  std::string topics;
  nh.getParam(param_prefix, topics);
  if (!topics.empty())
  {
    std::istringstream iss(topics);
    std::string topic;
    while (iss >> topic)
    {
      topics_dict[topic] = topic;
    }
    return;
  }

  std::vector<std::string> params;
  nh.getParamNames(params);
  std::vector<std::string>::const_iterator it = params.begin();
  for (; it != params.end(); it++)
  {
    std::string param_name = *it;
    if (param_name.rfind(param_prefix, 0) != 0)
    {
      continue;
    }
    std::string param_value;
    nh.getParam(param_name, param_value);
    if (param_value.empty())
    {
      continue;
    }
    topics_dict[param_name.substr(param_prefix.size() + 1)] = param_value;
  }
}

void Agv05Safety::process()
{
  // check heartbeat and time jump and system overload
  std::string safety_system_hint = checkHeartbeat();
  if (checkTimeJump())
  {
    safety_system_hint = "Time jumped";
  }
  else if (checkSystemOverload())
  {
    safety_system_hint = "System overload";
  }
  if (!safety_system_hint.empty())
  {
    safety_triggers_.system_error = safety_triggers_hold_.system_error = true;
  }
  else
  {
    safety_triggers_.system_error = false;
  }
  if (safety_system_hint_.data != safety_system_hint)
  {
    safety_system_hint_.data = safety_system_hint;
    safety_system_hint_pub_.publish(safety_system_hint_);
  }

  // summarize external safety 1 flag
  safety_triggers_.safety_in_1 = false;
  safety_external_message_.data = "";
  for (int i = 0, n = safety_external1_states_.size(); i < n; ++i)
  {
    if (safety_external1_states_[i])
    {
      safety_triggers_.safety_in_1 = safety_triggers_hold_.safety_in_1 = true;
      safety_external_message_.data = safety_external1_messages_[i];
      break;
    }
  }

  // summarize external safety 2 flag
  safety_triggers_.safety_in_2 = false;
  for (int i = 0, n = safety_external2_states_.size(); i < n; ++i)
  {
    if (safety_external2_states_[i])
    {
      safety_triggers_.safety_in_2 = safety_triggers_hold_.safety_in_2 = true;
      if (safety_external_message_.data.empty())
      {
        safety_external_message_.data = safety_external2_messages_[i];
      }
      break;
    }
  }

  if (!safety_external_message_.data.empty())
  {
    safety_external_message_hold_.data = safety_external_message_.data;
  }

  // summarize safety trigger flag
  safety_trigger_.data = summarizeSafetyTriggers(safety_triggers_);

  safety_external_message_pub_.publish(safety_external_message_);
  safety_triggers_pub_.publish(safety_triggers_);
  safety_trigger_pub_.publish(safety_trigger_);

  std_msgs::Bool msg;
  msg.data = !config_.peripherals_cut_off || !safety_trigger_.data;
  peripherals_relay_control_pub_.publish(msg);

  diagnostic_frequency_.tick();
  diagnostic_updater_.update();
}

void Agv05Safety::callbackInput(uint8_t agv05_msgs::SafetyTriggers::*flag, const std_msgs::Bool& msg)
{
  safety_triggers_.*flag = msg.data;
  safety_triggers_hold_.*flag |= msg.data;
}

void Agv05Safety::callbackSafetyExternal1(int index, const std_msgs::Bool& msg)
{
  safety_external1_states_[index] = msg.data;
}

void Agv05Safety::callbackSafetyExternal2(int index, const std_msgs::Bool& msg)
{
  safety_external2_states_[index] = msg.data;
}

void Agv05Safety::callbackSafetyHeartbeat(int index, const std_msgs::UInt8& msg)
{
  safety_heartbeat_states_[index] = msg.data;
  safety_heartbeat_timeouts_[index] = ros::Time::now().toSec() + SAFETY_HEARTBEAT_TIMEOUT;
}

void Agv05Safety::callbackConfig(agv05_safety::SafetyConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("agv05_safety: config received");
  config_ = config;
}

std::string Agv05Safety::checkHeartbeat()
{
  static double last_heartbeat = 0;
  static size_t last_index = 0;

  double now = ros::Time::now().toSec();
  for (int i = 0, n = safety_heartbeat_states_.size(); i < n; ++i)
  {
    if (safety_heartbeat_states_[i] || safety_heartbeat_timeouts_[i] < now)
    {
      last_heartbeat = now;
      last_index = i;
      break;
    }
  }

  // apply a 2-second timeout prior to clearing the time jump status
  if (now - last_heartbeat < 2.0)
  {
    return safety_heartbeat_messages_[last_index] + " Heartbeat (" +
           std::to_string(safety_heartbeat_states_[last_index]) + ')';
  }
  return "";
}

bool Agv05Safety::checkTimeJump()
{
  static double last_time_jump = 0;
  static double last_loop = ros::Time::now().toSec();

  double now = ros::Time::now().toSec();
  double period = now - last_loop;
  last_loop = now;

  if (period < 0 || period > TIME_JUMP_TOLERANCE)
  {
    last_time_jump = now;
    return true;
  }
  // apply a 2-second timeout prior to clearing the time jump status
  else if (now - last_time_jump < 2.0)
  {
    return true;
  }
  return false;
}

bool Agv05Safety::checkSystemOverload()
{
  static bool last_overload = false;
  static double last_check = 0;

  double now = ros::Time::now().toSec();
  double period = now - last_check;

  if (period >= 0 && period < 1.0)
  {
    return last_overload;
  }
  last_check = now;

  // Reference: https://github.com/giampaolo/psutil/blob/master/psutil/__init__.py
  // On Linux, guest times are already accounted in "user" ot "nice" times, so we skip reading them.
  union CPUTimes
  {
    struct
    {
      uint64_t user;
      uint64_t nice;
      uint64_t system;
      uint64_t idle;
      uint64_t iowait;
      uint64_t irq;
      uint64_t softirq;
      uint64_t steal;
    };
    uint64_t values[8];
  };

  static CPUTimes last_cpu_times = {0, 0, 0, 0, 0, 0, 0, 0};
  CPUTimes curr_cpu_times, cpu_times;

  {
    std::ifstream ifs("/proc/stat", std::ios_base::binary);
    std::string _;
    ifs >> _;
    for (int i = 0; i < 8; ++i)
    {
      ifs >> curr_cpu_times.values[i];
    }
  }

  uint64_t total = 0;
  for (int i = 0; i < 8; ++i)
  {
    // CPU times are always supposed to increase over time but
    // surprisingly sometimes this might not be the case
    if (curr_cpu_times.values[i] > last_cpu_times.values[i])
    {
      cpu_times.values[i] = curr_cpu_times.values[i] - last_cpu_times.values[i];
    }
    else
    {
      cpu_times.values[i] = 0;
    }
    last_cpu_times.values[i] = curr_cpu_times.values[i];
    total += cpu_times.values[i];
  }
  uint64_t busy = total - cpu_times.idle - cpu_times.iowait;
  bool overload = busy >= total * 0.9;

  // check memory usage
  uint64_t mem_total;
  uint64_t mem_available;
  {
    std::ifstream ifs("/proc/meminfo", std::ios_base::binary);
    std::string key;
    uint64_t value;
    std::string kb;

    bool found_total = false;
    bool found_avaiable = false;

    while (ifs >> key >> value >> kb)
    {
      if (key == "MemTotal:")
      {
        mem_total = value;
        found_total = true;
      }
      else if (key == "MemAvailable:")
      {
        mem_available = value;
        found_avaiable = true;
      }
      if (found_total && found_avaiable)
      {
        break;
      }
    }
  }
  if (mem_available <= mem_total * 0.15)
  {
    overload = true;
  }

  // maintain 2 seconds to toggle state
  if (overload != last_overload)
  {
    last_overload = overload;
    return !overload;
  }
  else
  {
    return overload;
  }
}

bool Agv05Safety::summarizeSafetyTriggers(const agv05_msgs::SafetyTriggers& safety_triggers)
{
  return safety_triggers.bumper_front || safety_triggers.bumper_rear || safety_triggers.emergency_button ||
         safety_triggers.safety_in_1 || safety_triggers.safety_in_2 || safety_triggers.motor_fault ||
         safety_triggers.wheel_slippage || safety_triggers.charger_connected || safety_triggers.system_error ||
         safety_triggers.nav_trigger;
}

void Agv05Safety::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Bumper Front", static_cast<bool>(safety_triggers_hold_.bumper_front));
  stat.add("Bumper Rear", static_cast<bool>(safety_triggers_hold_.bumper_rear));
  stat.add("Emergency Button", static_cast<bool>(safety_triggers_hold_.emergency_button));
  stat.add("External Safety 1", static_cast<bool>(safety_triggers_hold_.safety_in_1));
  stat.add("External Safety 2", static_cast<bool>(safety_triggers_hold_.safety_in_2));
  stat.add("Motor Fault", static_cast<bool>(safety_triggers_hold_.motor_fault));
  stat.add("Wheel Slippage", static_cast<bool>(safety_triggers_hold_.wheel_slippage));
  stat.add("Charger Connected", static_cast<bool>(safety_triggers_hold_.charger_connected));
  stat.add("Navigation Trigger", static_cast<bool>(safety_triggers_hold_.nav_trigger));
  stat.add("System Error", safety_system_hint_.data.empty() ? "-" : safety_system_hint_.data);
  stat.add("External Safety Message", safety_external_message_hold_.data.empty() ? "-" :
                                      safety_external_message_hold_.data);

  if (summarizeSafetyTriggers(safety_triggers_hold_))
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Safety Triggered");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status OK");
  }

  safety_triggers_hold_ = safety_triggers_;
  safety_external_message_hold_.data = safety_external_message_.data;
}

}  // namespace agv05

/* main function */
int main(int argc, char** argv)
{
  // Initialize ROS node.
  ros::init(argc, argv, "agv05_safety");
  ROS_INFO_STREAM("agv05_safety start");

  agv05::Agv05Safety safety;

  ros::Rate r(LOOP_FREQUENCY);
  while (ros::ok())
  {
    ros::spinOnce();
    safety.process();
    r.sleep();
  }
}

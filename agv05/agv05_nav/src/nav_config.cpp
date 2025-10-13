/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <json/json.h>

#include "agv05_nav/nav.h"


namespace agv05
{

bool NavConfig::enumProfileCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  Json::Value enum_src;
  Json::FastWriter writer;

  // convert to Json::Value
  size_t size = sizeof(defaults_) / sizeof(ProfileDefault);
  for (Json::Int i = 0; i <= size; i++)
  {
    Json::Value item;
    item.append(i);
    item.append(i ? defaults_[i - 1].name : "Skip (No Change)");
    enum_src["nav_profiles"].append(item);
  }

  res.message = writer.write(enum_src);
  res.success = true;
  return true;
}

void NavConfig::setupProfileDefault(size_t index)
{
  double param;
  std::string name("profile_");

  if (index >= (sizeof(defaults_) / sizeof(ProfileDefault))) return;
  const agv05_nav::NavConfig& cfg_default = agv05_nav::NavConfig::__getDefault__();

  char c = 'a' + index;
  name += c;
  ros::NodeHandle nh("~/" + name);

  nh.param("name", defaults_[index].name, name);

#define LOAD_PARAM(v, p, d) nh.param(p, param, d); v = param

  LOAD_PARAM(defaults_[index].profile.line_follow_p, "line_follow_p", cfg_default.line_follow_p1);
  LOAD_PARAM(defaults_[index].profile.line_follow_i, "line_follow_i", cfg_default.line_follow_i1);
  LOAD_PARAM(defaults_[index].profile.line_follow_d, "line_follow_d", cfg_default.line_follow_d1);
  LOAD_PARAM(defaults_[index].profile.line_follow_ahead_distance, "line_follow_ahead_distance", cfg_default.line_follow_ahead_distance1);
  LOAD_PARAM(defaults_[index].profile.line_follow_error_min, "line_follow_error_min", cfg_default.line_follow_error_min1);
  LOAD_PARAM(defaults_[index].profile.line_follow_error_max, "line_follow_error_max", cfg_default.line_follow_error_max1);
  nh.param("line_follow_pid_traditional", defaults_[index].profile.line_follow_pid_traditional, cfg_default.line_follow_pid_traditional1);

  LOAD_PARAM(defaults_[index].profile.straight_jerk_acc, "straight_jerk_acc", cfg_default.straight_jerk_acc1);
  LOAD_PARAM(defaults_[index].profile.straight_jerk_dec, "straight_jerk_dec", cfg_default.straight_jerk_dec1);
  LOAD_PARAM(defaults_[index].profile.straight_normal_acc, "straight_normal_acc", cfg_default.straight_normal_acc1);
  LOAD_PARAM(defaults_[index].profile.straight_normal_dec, "straight_normal_dec", cfg_default.straight_normal_dec1);
  LOAD_PARAM(defaults_[index].profile.straight_max_speed, "straight_max_speed", cfg_default.straight_max_speed1);
  LOAD_PARAM(defaults_[index].profile.bezier_max_speed, "bezier_max_speed", cfg_default.bezier_max_speed1);

  LOAD_PARAM(defaults_[index].profile.turn_jerk_acc, "turn_jerk_acc", cfg_default.turn_jerk_acc1);
  LOAD_PARAM(defaults_[index].profile.turn_jerk_dec, "turn_jerk_dec", cfg_default.turn_jerk_dec1);
  LOAD_PARAM(defaults_[index].profile.turn_normal_acc, "turn_normal_acc", cfg_default.turn_normal_acc1);
  LOAD_PARAM(defaults_[index].profile.turn_normal_dec, "turn_normal_dec", cfg_default.turn_normal_dec1);
  LOAD_PARAM(defaults_[index].profile.turn_max_speed, "turn_max_speed", cfg_default.turn_max_speed1);

  LOAD_PARAM(defaults_[index].stop.forward.offset, "forward_stopping_offset", cfg_default.forward_stopping_offset);
  LOAD_PARAM(defaults_[index].stop.forward.offset_prior_turning_left, "forward_stopping_offset_prior_turning_left", cfg_default.forward_stopping_offset_prior_turning_left);
  LOAD_PARAM(defaults_[index].stop.forward.offset_prior_turning_right, "forward_stopping_offset_prior_turning_right", cfg_default.forward_stopping_offset_prior_turning_right);
  LOAD_PARAM(defaults_[index].stop.reverse.offset, "reverse_stopping_offset", cfg_default.reverse_stopping_offset);
  LOAD_PARAM(defaults_[index].stop.reverse.offset_prior_turning_left, "reverse_stopping_offset_prior_turning_left", cfg_default.reverse_stopping_offset_prior_turning_left);
  LOAD_PARAM(defaults_[index].stop.reverse.offset_prior_turning_right, "reverse_stopping_offset_prior_turning_right", cfg_default.reverse_stopping_offset_prior_turning_right);

  LOAD_PARAM(defaults_[index].stop.straight.decel, "straight_junction_dec", cfg_default.straight_junction_dec);
  LOAD_PARAM(defaults_[index].stop.straight.speed, "straight_junction_speed", cfg_default.straight_junction_speed);
  LOAD_PARAM(defaults_[index].stop.straight.speed_min, "straight_junction_min_speed", cfg_default.straight_junction_min_speed);

  nh.param("persist_error_over_junction", defaults_[index].stop.straight.persist_error, cfg_default.persist_error_over_junction);

  LOAD_PARAM(defaults_[index].stop.forward.junction, "forward_junction_distance", cfg_default.forward_junction_distance);
  LOAD_PARAM(defaults_[index].stop.reverse.junction, "reverse_junction_distance", cfg_default.reverse_junction_distance);
  LOAD_PARAM(defaults_[index].stop.straight.tolerance, "overshoot_stopping_distance", cfg_default.overshoot_stopping_distance);
  LOAD_PARAM(defaults_[index].stop.straight.undershoot, "undershoot_stopping_distance", cfg_default.undershoot_stopping_distance);
  LOAD_PARAM(defaults_[index].stop.straight.timeout, "straight_alignment_max_time", cfg_default.straight_alignment_max_time);

  LOAD_PARAM(defaults_[index].stop.io_trigger.jerk, "io_trigger_stop_jerk", cfg_default.io_trigger_stop_jerk);
  LOAD_PARAM(defaults_[index].stop.io_trigger.decel, "io_trigger_stop_dec", cfg_default.io_trigger_stop_dec);
  LOAD_PARAM(defaults_[index].stop.straight.distance, "io_trigger_stop_distance", cfg_default.io_trigger_stop_distance);
  LOAD_PARAM(defaults_[index].stop.io_trigger.debounce, "io_trigger_debounce", cfg_default.io_trigger_debounce);

  LOAD_PARAM(defaults_[index].turn.speed, "turn_normal_speed", cfg_default.turn_normal_speed);
  LOAD_PARAM(defaults_[index].turn.align.speed, "turn_search_line_speed", cfg_default.turn_search_line_speed);
  LOAD_PARAM(defaults_[index].turn.align.distance, "turn_search_distance", cfg_default.turn_search_distance);
  LOAD_PARAM(defaults_[index].turn.align.kp, "turn_alignment_gain", cfg_default.turn_alignment_gain);
  LOAD_PARAM(defaults_[index].turn.align.timeout, "turn_alignment_max_time", cfg_default.turn_alignment_max_time);
  LOAD_PARAM(defaults_[index].turn.align.hysteresis.time, "turn_alignment_center_min_time", cfg_default.turn_alignment_center_min_time);
  LOAD_PARAM(defaults_[index].turn.align.hysteresis.distance, "turn_alignment_center_error", cfg_default.turn_alignment_center_error);

#undef LOAD_PARAM
}

NavConfig& NavConfig::operator=(agv05_nav::NavConfig& config)
{
#define ROUND(f) (round((f) * 1e7) * 1e-7)
#define RESET_PROFILE(i) \
  if (config.reset_profile_##i##_) \
  { \
    size_t index = config.reset_profile_##i##_ - 1; \
    config.reset_profile_##i##_ = 0; \
    \
    config.line_follow_p##i = ROUND(defaults_[index].profile.line_follow_p); \
    config.line_follow_i##i = ROUND(defaults_[index].profile.line_follow_i); \
    config.line_follow_d##i = ROUND(defaults_[index].profile.line_follow_d); \
    config.line_follow_ahead_distance##i = ROUND(defaults_[index].profile.line_follow_ahead_distance); \
    config.line_follow_error_min##i = ROUND(defaults_[index].profile.line_follow_error_min); \
    config.line_follow_error_max##i = ROUND(defaults_[index].profile.line_follow_error_max); \
    config.line_follow_pid_traditional##i = defaults_[index].profile.line_follow_pid_traditional; \
    \
    config.straight_jerk_acc##i = ROUND(defaults_[index].profile.straight_jerk_acc); \
    config.straight_jerk_dec##i = ROUND(defaults_[index].profile.straight_jerk_dec); \
    config.straight_normal_acc##i = ROUND(defaults_[index].profile.straight_normal_acc); \
    config.straight_normal_dec##i = ROUND(defaults_[index].profile.straight_normal_dec); \
    config.straight_max_speed##i = ROUND(defaults_[index].profile.straight_max_speed); \
    config.bezier_max_speed##i = ROUND(defaults_[index].profile.bezier_max_speed); \
    \
    config.turn_jerk_acc##i = ROUND(defaults_[index].profile.turn_jerk_acc); \
    config.turn_jerk_dec##i = ROUND(defaults_[index].profile.turn_jerk_dec); \
    config.turn_normal_acc##i = ROUND(defaults_[index].profile.turn_normal_acc); \
    config.turn_normal_dec##i = ROUND(defaults_[index].profile.turn_normal_dec); \
    config.turn_max_speed##i = ROUND(defaults_[index].profile.turn_max_speed); \
  }

  RESET_PROFILE(1);
  RESET_PROFILE(2);
  RESET_PROFILE(3);
  RESET_PROFILE(4);
  RESET_PROFILE(5);

#undef RESET_PROFILE

  if (config.reset_profile_stop_)
  {
    size_t index = config.reset_profile_stop_ - 1;
    config.reset_profile_stop_ = 0;

    config.forward_stopping_offset = ROUND(defaults_[index].stop.forward.offset);
    config.forward_stopping_offset_prior_turning_left = ROUND(defaults_[index].stop.forward.offset_prior_turning_left);
    config.forward_stopping_offset_prior_turning_right = ROUND(defaults_[index].stop.forward.offset_prior_turning_right);
    config.reverse_stopping_offset = ROUND(defaults_[index].stop.reverse.offset);
    config.reverse_stopping_offset_prior_turning_left = ROUND(defaults_[index].stop.reverse.offset_prior_turning_left);
    config.reverse_stopping_offset_prior_turning_right = ROUND(defaults_[index].stop.reverse.offset_prior_turning_right);

    config.straight_junction_dec = ROUND(defaults_[index].stop.straight.decel);
    config.straight_junction_speed = ROUND(defaults_[index].stop.straight.speed);
    config.straight_junction_min_speed = ROUND(defaults_[index].stop.straight.speed_min);
    config.persist_error_over_junction = defaults_[index].stop.straight.persist_error;
    config.forward_junction_distance = ROUND(defaults_[index].stop.forward.junction);
    config.reverse_junction_distance = ROUND(defaults_[index].stop.reverse.junction);
    config.overshoot_stopping_distance = ROUND(defaults_[index].stop.straight.tolerance);
    config.undershoot_stopping_distance = ROUND(defaults_[index].stop.straight.undershoot);
    config.straight_alignment_max_time = ROUND(defaults_[index].stop.straight.timeout);

    config.io_trigger_stop_jerk = ROUND(defaults_[index].stop.io_trigger.jerk);
    config.io_trigger_stop_dec = ROUND(defaults_[index].stop.io_trigger.decel);
    config.io_trigger_stop_distance = ROUND(defaults_[index].stop.straight.distance);
    config.io_trigger_debounce = ROUND(defaults_[index].stop.io_trigger.debounce);
  }

  if (config.reset_profile_turn_)
  {
    size_t index = config.reset_profile_turn_ - 1;
    config.reset_profile_turn_ = 0;

    config.turn_normal_speed = ROUND(defaults_[index].turn.speed);
    config.turn_search_line_speed = ROUND(defaults_[index].turn.align.speed);
    config.turn_search_distance = ROUND(defaults_[index].turn.align.distance);
    config.turn_alignment_gain = ROUND(defaults_[index].turn.align.kp);
    config.turn_alignment_max_time = ROUND(defaults_[index].turn.align.timeout);
    config.turn_alignment_center_min_time = ROUND(defaults_[index].turn.align.hysteresis.time);
    config.turn_alignment_center_error = ROUND(defaults_[index].turn.align.hysteresis.distance);
  }
#undef ROUND

  agv05_nav::NavConfig::operator=(config);

#define LOAD_PROFILE(i) \
  profiles_[(i)-1].line_follow_p = line_follow_p##i; \
  profiles_[(i)-1].line_follow_i = line_follow_i##i; \
  profiles_[(i)-1].line_follow_d = line_follow_d##i; \
  profiles_[(i)-1].line_follow_ahead_distance = line_follow_ahead_distance##i; \
  profiles_[(i)-1].line_follow_error_min = line_follow_error_min##i; \
  profiles_[(i)-1].line_follow_error_max = line_follow_error_max##i; \
  profiles_[(i)-1].line_follow_pid_traditional = line_follow_pid_traditional##i; \
  \
  profiles_[(i)-1].straight_jerk_acc = straight_jerk_acc##i; \
  profiles_[(i)-1].straight_jerk_dec = straight_jerk_dec##i; \
  profiles_[(i)-1].straight_normal_acc = straight_normal_acc##i; \
  profiles_[(i)-1].straight_normal_dec = straight_normal_dec##i; \
  profiles_[(i)-1].straight_max_speed = straight_max_speed##i; \
  profiles_[(i)-1].bezier_max_speed = bezier_max_speed##i; \
  \
  profiles_[(i)-1].turn_jerk_acc = turn_jerk_acc##i; \
  profiles_[(i)-1].turn_jerk_dec = turn_jerk_dec##i; \
  profiles_[(i)-1].turn_normal_acc = turn_normal_acc##i; \
  profiles_[(i)-1].turn_normal_dec = turn_normal_dec##i; \
  profiles_[(i)-1].turn_max_speed = turn_max_speed##i;

  LOAD_PROFILE(1)
  LOAD_PROFILE(2)
  LOAD_PROFILE(3)
  LOAD_PROFILE(4)
  LOAD_PROFILE(5)

#undef LOAD_PROFILE

  return *this;
}

}  // namespace agv05

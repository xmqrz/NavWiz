/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#ifndef AGV05_NAV_NAV_H
#define AGV05_NAV_NAV_H

#include <actionlib/server/simple_action_server.h>
#include <agv05_msgs/NavActionAction.h>
#include <agv05_msgs/NavControl.h>
#include <agv05_nav/NavConfig.h>
#include <agv05_nav/components.h>
#include <agv05_nav/utils.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/reverse_lock.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#define LOOP_FREQUENCY 100


namespace agv05
{

typedef agv05_msgs::NavActionGoal Goal;
typedef agv05_msgs::NavActionGoalConstPtr GoalConstPtr;
typedef agv05_msgs::NavActionFeedback Feedback;
typedef agv05_msgs::NavActionResult Result;
typedef agv05_msgs::NavControl NavControl;

class ActionProcessor;

class NavConfig: public agv05_nav::NavConfig
{
public:
  struct Profile
  {
    float line_follow_p;
    float line_follow_i;
    float line_follow_d;
    float line_follow_ahead_distance;
    float line_follow_error_min;
    float line_follow_error_max;
    bool line_follow_pid_traditional;

    float straight_jerk_acc;
    float straight_jerk_dec;
    float straight_normal_acc;
    float straight_normal_dec;
    float straight_max_speed;
    float bezier_max_speed;

    float turn_jerk_acc;
    float turn_jerk_dec;
    float turn_normal_acc;
    float turn_normal_dec;
    float turn_max_speed;
  };

  struct ProfileDefault
  {
    std::string name;
    Profile profile;
    StoppingConfig stop;
    TurningConfig turn;
  };

public:
  NavConfig() : profile_index_(0) {}

  NavConfig& operator=(agv05_nav::NavConfig& config);

  size_t getProfileIndex()
  {
    return profile_index_;
  }
  Profile& getProfile()
  {
    return profiles_[profile_index_];
  }
  void selectProfile(size_t index)
  {
    if (index < 5)
    {
      profile_index_ = index;
    }
  }
  void setupProfileDefault()
  {
    size_t size = sizeof(defaults_) / sizeof(ProfileDefault);
    for (size_t i = 0; i < size; i++) setupProfileDefault(i);

    ros::NodeHandle nh("~");
    enum_profile_server_ = nh.advertiseService("enum_profile", &NavConfig::enumProfileCallback, this);
  };

private:
  void setupProfileDefault(size_t index);

  // dynamic reconfigure enum profiles server
  bool enumProfileCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer enum_profile_server_;

  ProfileDefault defaults_[3];
  Profile profiles_[5];
  size_t profile_index_;
};

class Diagnostic
{
public:
  explicit Diagnostic(ros::NodeHandle& nh, NavConfig& config);

  void update(uint8_t nav, uint8_t status)
  {
    nav_ = nav;
    status_ = status;

    diagnostic_frequency_.tick();
    updater_.update();
  }

  void update(uint8_t nav, uint8_t status, float overshoot)
  {
    overshoot_ = overshoot;
    update(nav, status);
  }

private:
  void statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  double expected_frequency_;
  diagnostic_updater::FrequencyStatus diagnostic_frequency_;
  diagnostic_updater::Updater updater_;

  // dynamic reconfigure variable
  NavConfig& config_;

  uint8_t nav_;
  uint8_t status_;
  float overshoot_;
};

class Nav
{
  friend class ActionProcessor;

public:
  Nav();
  ~Nav();
  void process(float frequency);
  const agv05_nav::NavConfig& config()
  {
    return config_;
  }

private:
  // callbacks
  void handleConfig(agv05_nav::NavConfig& config, uint32_t level)
  {
    ROS_INFO("Configuration received.");
    config_ = config;
  }
  void handleGoal(const GoalConstPtr& goal);
  void handleNavControl(const NavControl& nav_control);

private:
  ros::NodeHandle nh_;

  // action server
  actionlib::SimpleActionServer<agv05_msgs::NavActionAction> as_;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<agv05_nav::NavConfig> ds_;

  // subscribers
  ros::Subscriber nav_control_sub_;

  // robot components
  Base base_;
  Io io_;
  LaserSensor laser_sensor_;
  LineSensor line_sensor_;
  Panel panel_;
  Safety safety_;

  // dynamic reconfigure variable
  NavConfig config_;

  // action processor
  boost::shared_ptr<ActionProcessor> processor_;

  // diagnostic
  Diagnostic diagnostic_;

  // mutex
  typedef boost::mutex::scoped_lock Lock;
  typedef boost::reverse_lock<Lock> Unlock;
  boost::mutex mutex_;
};

}  // namespace agv05

#endif  // AGV05_NAV_NAV_H

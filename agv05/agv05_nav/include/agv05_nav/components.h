/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAV_COMPONENTS_H
#define AGV05_NAV_COMPONENTS_H

#include <agv05_msgs/LineSensor.h>
#include <agv05_msgs/ObstacleSensor.h>
#include <agv05_msgs/ObstacleSensorArea.h>
#include <agv05_msgs/SafetyTriggers.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#define SAFETY_HEARTBEAT_TIMEOUT 0.5


namespace agv05
{

class Base
{
public:
  explicit Base(ros::NodeHandle& nh);

  void setSpeed(const geometry_msgs::Twist& cmd_vel);
  void setSpeed(float linear, float angular, float lateral = 0.0f);
  void turnLeft(float angular)
  {
    setSpeed(0.0f, std::fabs(angular));
  }
  void turnRight(float angular)
  {
    setSpeed(0.0f, -std::fabs(angular));
  }
  void stop()
  {
    setSpeed(0.0f, 0.0f);
  }

  bool getPose(geometry_msgs::Pose2D& pose, bool wheel = false)
  {
    pose = wheel ? pose_wheel_ : pose_;
    return true;
  }
  double getStraightDistance()
  {
    return straight_distance_;
  }
  double getRotationalDistance()
  {
    return rotational_distance_;
  }
  bool isNavigationEnabled()
  {
    return navigation_enabled_;
  }
  bool isSteeringAligned()
  {
    return steering_aligned_;
  }
  bool getMotorCmdVel(geometry_msgs::Twist& cmd_vel, bool max = false);
  geometry_msgs::Twist getManualCmdVel()
  {
    return manual_cmd_vel_;
  }
  void resetManualCmdVel()
  {
    manual_cmd_vel_ = geometry_msgs::Twist();
    manual_cmd_vel_timeout_ = 0.0;
  }
  void incrementManualCmdVelTimeout(double increment)
  {
    manual_cmd_vel_timeout_ += increment;
  }
  double getManualCmdVelTimeout()
  {
    return manual_cmd_vel_timeout_;
  }

private:
  void odomToPose2D(const nav_msgs::Odometry& msg, geometry_msgs::Pose2D& pose);
  void handleOdom(const nav_msgs::Odometry& msg);
  void handleOdomWheel(const nav_msgs::Odometry& msg);
  void handleStraightDistance(const std_msgs::Float64& msg);
  void handleRotationalDistance(const std_msgs::Float64& msg);
  void handleNavigationEnable(const std_msgs::Bool& msg);
  void handleSteeringAlign(const std_msgs::Bool& msg);
  void handleManualCmdVel(const geometry_msgs::Twist& msg);

private:
  ros::Publisher cmd_vel_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber odom_wheel_sub_;
  ros::Subscriber straight_distance_sub_;
  ros::Subscriber rotational_distance_sub_;
  ros::Subscriber navigation_enable_sub_;
  ros::Subscriber steering_align_sub_;
  ros::Subscriber manual_cmd_vel_sub_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Pose2D pose_wheel_;
  double straight_distance_;
  double rotational_distance_;
  bool navigation_enabled_;
  bool steering_aligned_;

  geometry_msgs::Twist manual_cmd_vel_;
  double manual_cmd_vel_timeout_;
};

class Io
{
public:
  enum
  {
    NUM_PORTS = 8,
    NUM_PINS = 16,
  };
  typedef size_t Inputs[NUM_PORTS];

public:
  explicit Io(ros::NodeHandle& nh);

  Inputs& getAllInputs()
  {
    return inputs_;
  }
  bool getInput(size_t port, size_t pin)
  {
    if (port - 1 >= NUM_PORTS || pin >= NUM_PINS)
    {
      return false;
    }
    return inputs_[port - 1] & 1 << pin;
  }

private:
  void handleInput(size_t port, const std_msgs::UInt16ConstPtr& msg);

private:
  ros::Subscriber input_subs_[NUM_PORTS];

  Inputs inputs_;
};

class LaserSensor
{
public:
  explicit LaserSensor(ros::NodeHandle& nh);

  void selectArea(uint8_t area, uint8_t profile = 0xff);

  void selectProfile(uint8_t profile)
  {
    profile_ = profile;
  }

  uint8_t getArea()
  {
    return area_;
  }

  agv05_msgs::ObstacleSensor getActivation()
  {
    if (timeout_ < ros::Time::now())
    {
      agv05_msgs::ObstacleSensor activation;
      activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MALFUNCTION;
      activation.malfunction = true;
      activation.hint = "Obstacle Sensor Stale";
      return activation;
    }
    return activation_;
  }

  enum StopDelay
  {
    STOP_DELAY_OFF = 0,
    STOP_DELAY_ON,
    STOP_DELAY_DONE,
  };
  StopDelay stopDelay(double delay_sec = -1.0);

private:
  void handleArea(const agv05_msgs::ObstacleSensorArea& msg);
  void handleActivation(const agv05_msgs::ObstacleSensor& activation);

private:
  ros::Publisher area_pub_;

  ros::Subscriber area_sub_;
  ros::Subscriber activation_sub_;

  agv05_msgs::ObstacleSensor activation_;
  ros::Time timeout_;
  uint8_t profile_;
  uint8_t area_;

  ros::Time stop_after_;  // delay deactivation after non-stopping motion
};

class LineSensor
{
public:
  explicit LineSensor(ros::NodeHandle& nh);

  void activateCalibration(bool activate);
  void setPreferredSide(uint8_t preferred_side);
  agv05_msgs::LineSensor getFrontData();
  agv05_msgs::LineSensor getRearData();

private:
  void handleMsbFront(const agv05_msgs::LineSensor& msb_data);
  void handleMsbRear(const agv05_msgs::LineSensor& msb_data);

private:
  ros::Publisher front_calibrate_pub_;
  ros::Publisher rear_calibrate_pub_;
  ros::Publisher front_preferred_side_pub_;
  ros::Publisher rear_preferred_side_pub_;

  ros::Subscriber front_data_sub_;
  ros::Subscriber rear_data_sub_;

  agv05_msgs::LineSensor front_data_;
  agv05_msgs::LineSensor rear_data_;

  double front_timeout_;
  double rear_timeout_;
};

class Panel
{
public:
  explicit Panel(ros::NodeHandle& nh);
  void update(uint8_t status, uint8_t led_side = 0);
  void setAlarm(bool activate);
  void setLed(uint8_t mode);

private:
  ros::Publisher alarm_control_pub_;
  ros::Publisher led_control_pub_;

  bool alarm_activated_;
  uint8_t led_mode_;
};

class Safety
{
public:
  explicit Safety(ros::NodeHandle& nh);

  void publishMuteBumper(bool mute);
  void publishNavTrigger(bool trigger, const std::string& safety_internal_message = "");
  void publishSafetyHeartbeat();

  const std::string& getMotorFaultHint()
  {
    return motor_fault_hint_;
  }

  const std::string& getSafetySystemHint()
  {
    return safety_system_hint_;
  }

  agv05_msgs::SafetyTriggers getSafetyTrigger()
  {
    return safety_trigger_;
  }

private:
  void handleMotorFaultHint(const std_msgs::String& msg);
  void handleSafetySystemHint(const std_msgs::String& msg);
  void handleSafety(const agv05_msgs::SafetyTriggers& safety_trigger);

private:
  ros::Publisher mute_bumper_pub_;
  ros::Publisher nav_trigger_pub_;
  ros::Publisher safety_internal_message_pub_;
  ros::Publisher safety_heartbeat_pub_;

  ros::Subscriber motor_fault_hint_sub_;
  ros::Subscriber safety_system_hint_sub_;
  ros::Subscriber safety_sub_;

  std::string motor_fault_hint_, safety_system_hint_;
  agv05_msgs::SafetyTriggers safety_trigger_;
  double safety_trigger_timeout_;
  bool mute_bumper_, nav_trigger_;
  std_msgs::String safety_internal_message_;
  double safety_heartbeat_timeout_;
};

}  // namespace agv05

#endif  // AGV05_NAV_COMPONENTS_H

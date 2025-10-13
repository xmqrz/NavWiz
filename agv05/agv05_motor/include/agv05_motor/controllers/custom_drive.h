/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef AGV05_MOTOR_CONTROLLERS_CUSTOM_DRIVE_H
#define AGV05_MOTOR_CONTROLLERS_CUSTOM_DRIVE_H

#include <agv05_motor/controller.h>


namespace agv05
{

class CustomDrive: public Controller
{
public:
  CustomDrive();

  // dynamic reconfigure
  void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update = false);

protected:
  // timer for speed smoothening
  void ctrlCallback(bool enable, float period = 0.0f, const std::vector< std::complex<double> >& speed = {});

  // drive feedback
  void callbackFeedback(const nav_msgs::Odometry& msg);
  void callbackMaxTwist(const geometry_msgs::Twist& msg)
  {
    max_twist_ = msg;
    angular_speed_ratio_ = std::abs(msg.angular.z) / std::max(std::abs(msg.linear.x), std::abs(msg.linear.y));
    if (config_.angular_millage_gain_ > (1.0 / angular_speed_ratio_))
    {
      config_.angular_millage_gain_ = 1.0 / angular_speed_ratio_;
    }
  }
  void callbackNavigationEnable(const std_msgs::Bool& msg)
  {
    navigation_enable_ = msg;
  }
  void callbackMotorFault(const std_msgs::Bool& msg)
  {
    motor_fault_ = msg;
  }
  void callbackMotorFaultHint(const std_msgs::String& msg)
  {
    motor_fault_hint_ = msg;
  }

  // latest output
  bool getTwist(bool max, geometry_msgs::Twist& twist)
  {
    twist = max ? max_twist_ : velocity_;
    return true;
  }

  // diagnostic
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // helper function
  void publishOdom(const ros::Time& now);
  void resetOdom(const ros::Time& now, bool pos = false)
  {
    base_.reset(pos);
    publishOdom(now);
  }

protected:
  /* ROS publishers */
  ros::Publisher enable_pub_;

  /* ROS subscribers */
  ros::Subscriber feedback_sub_;
  ros::Subscriber max_twist_sub_;
  ros::Subscriber navigation_enable_sub_;
  ros::Subscriber motor_fault_sub_;
  ros::Subscriber motor_fault_hint_sub_;

  /* Data inputs */
  geometry_msgs::Twist max_twist_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_CONTROLLERS_CUSTOM_DRIVE_H

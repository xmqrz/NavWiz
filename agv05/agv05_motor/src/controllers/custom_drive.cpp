/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <agv05_motor/controllers/custom_drive.h>
#include <angles/angles.h>


namespace agv05
{

CustomDrive::CustomDrive() :
  Controller(0)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_motor("agv05/motor");

  // ROS publishers
  enable_pub_ = nh_motor.advertise<std_msgs::Bool>("enable", 1, true);

  // ROS subscribers
  feedback_sub_ = nh.subscribe("odom/wheel", 10, &CustomDrive::callbackFeedback, this);
  max_twist_sub_ = nh_motor.subscribe("max_twist", 1, &CustomDrive::callbackMaxTwist, this);
  navigation_enable_sub_ = nh_motor.subscribe("navigation_enable", 1, &CustomDrive::callbackNavigationEnable, this);
  motor_fault_sub_ = nh.subscribe("agv05/safety/motor_fault", 1, &CustomDrive::callbackMotorFault, this);
  motor_fault_hint_sub_ = nh_motor.subscribe("fault_hint", 1, &CustomDrive::callbackMotorFaultHint, this);

  // Initial publish
  std_msgs::Bool msg;
  msg.data = true;
  steering_align_pub_.publish(msg);
}

void CustomDrive::callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update)
{
  if (force_update)
  {
    timer_.stop();

    if (max_twist_.angular.z == 0.0)
    {
      angular_speed_ratio_ = 1.0 / config.angular_millage_gain_;
      max_twist_.linear.x = max_speed_;
      max_twist_.angular.z = max_speed_ * angular_speed_ratio_;
    }
  }
  if (config.angular_millage_gain_ > (1.0 / angular_speed_ratio_))
  {
    config.angular_millage_gain_ = 1.0 / angular_speed_ratio_;
  }
  Controller::callbackConfig(config, level, force_update);
}

void CustomDrive::ctrlCallback(bool enable, float period, const std::vector< std::complex<double> >& speed)
{
  std_msgs::Bool msg;
  msg.data = enable;
  enable_pub_.publish(msg);
}

void CustomDrive::callbackFeedback(const nav_msgs::Odometry& msg)
{
  // Hack: Use the smallest positive denormal to signal free motor
  bool free_motor = *reinterpret_cast<uint64_t*>(&velocity_.linear.z) == 1LL || button_unbrake_;
  free_motor &= safety_trigger_;
  ctrlCallback(!free_motor);

  if (wheel_.odom_.header.stamp.isZero())
  {
    wheel_.odom_ = msg;
    return;
  }

  tf2::Quaternion q;
  tf2::fromMsg(wheel_.odom_.pose.pose.orientation, q);
  double theta = tf2::getYaw(q);

  tf2::fromMsg(msg.pose.pose.orientation, q);
  double dtheta = angles::normalize_angle(tf2::getYaw(q) - theta);
  std::complex<double> dxy = std::polar<double>(1.0, -theta) *
                             std::complex<double>(msg.pose.pose.position.x - wheel_.odom_.pose.pose.position.x,
                                                  msg.pose.pose.position.y - wheel_.odom_.pose.pose.position.y);

  double dist = std::max(std::abs(dxy), std::abs(dtheta * config_.angular_millage_gain_));
  if (dist > 2)
  {
    ROS_ERROR("Huge odom difference: %f: %f %f %f", dist, dxy.real(), dxy.imag(), dtheta);
    wheel_.odom_ = msg;
    return;
  }

  double dx = dxy.real();
  dtheta = yawChange(dtheta, dist > 0.0);
  base_.update(dx, dxy.imag(), dtheta, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z);

  publishOdom(msg.header.stamp);

  // publish only on change
  if (dist > 0.0 || dtheta != 0.0)
  {
    straight_.data += dx;
    rotational_.data += dtheta;

    straight_pub_.publish(straight_);
    rotational_pub_.publish(rotational_);

    // update mileage
    Lock lock(mutex_);
    mileage_ += dist;
  }

  // store feedback
  wheel_.odom_ = msg;
  diagnostic_frequency_.tick();
}

void CustomDrive::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Drive Type", "Custom Drive");

  Controller::diagnosticStatus(stat);
}

void CustomDrive::publishOdom(const ros::Time& now)
{
  if (config_.odom_tf_broadcast_)
  {
    // publish tf and odom
    base_.transform_.header.stamp = base_.odom_.header.stamp = now;
    tfb_.sendTransform(base_.transform_);
    odom_pub_.publish(base_.odom_);
  }

  wheel_.transform_.header.stamp = now;
  wheel_.transform_.transform.translation.x = wheel_.odom_.pose.pose.position.x;
  wheel_.transform_.transform.translation.y = wheel_.odom_.pose.pose.position.y;
  wheel_.transform_.transform.rotation = wheel_.odom_.pose.pose.orientation;
  tfb_.sendTransform(wheel_.transform_);
}

}  // namespace agv05

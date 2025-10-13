/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <agv05_msgs/GetTwist.h>
#include "agv05_nav/components.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace agv05
{

Base::Base(ros::NodeHandle& nh) :
  straight_distance_(0.0), rotational_distance_(0.0),
  navigation_enabled_(false), steering_aligned_(false),
  manual_cmd_vel_timeout_(0.0)
{
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  odom_sub_ = nh.subscribe("odom", 1, &Base::handleOdom, this);
  odom_wheel_sub_ = nh.subscribe("odom/wheel", 1, &Base::handleOdomWheel, this);
  straight_distance_sub_ = nh.subscribe("agv05/motor/straight_distance", 10, &Base::handleStraightDistance, this);
  rotational_distance_sub_ = nh.subscribe("agv05/motor/rotational_distance", 10, &Base::handleRotationalDistance, this);
  navigation_enable_sub_ = nh.subscribe("agv05/motor/navigation_enable", 10, &Base::handleNavigationEnable, this);
  steering_align_sub_ = nh.subscribe("agv05/motor/steering_align", 10, &Base::handleSteeringAlign, this);
  manual_cmd_vel_sub_ = nh.subscribe("agv05/nav/manual_cmd_vel", 10, &Base::handleManualCmdVel, this);
}

void Base::setSpeed(const geometry_msgs::Twist& cmd_vel)
{
  cmd_vel_pub_.publish(cmd_vel);
}

void Base::setSpeed(float linear, float angular, float lateral)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.linear.y = lateral;
  cmd_vel.angular.z = angular;
  setSpeed(cmd_vel);
}

bool Base::getMotorCmdVel(geometry_msgs::Twist& cmd_vel, bool max)
{
  ros::NodeHandle nh_motor("agv05/motor");
  ros::ServiceClient client = nh_motor.serviceClient<agv05_msgs::GetTwist>("get_twist");
  agv05_msgs::GetTwist srv;
  srv.request.max = max;

  if (client.call(srv))
  {
    cmd_vel = srv.response.data;
    return true;
  }
  return false;
}

void Base::odomToPose2D(const nav_msgs::Odometry& msg, geometry_msgs::Pose2D& pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg.pose.pose.orientation, q);
  pose.theta = tf2::getYaw(q);
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
}

void Base::handleOdom(const nav_msgs::Odometry& msg)
{
  odomToPose2D(msg, pose_);
}

void Base::handleOdomWheel(const nav_msgs::Odometry& msg)
{
  odomToPose2D(msg, pose_wheel_);
}

void Base::handleStraightDistance(const std_msgs::Float64& msg)
{
  straight_distance_ = msg.data;
}

void Base::handleRotationalDistance(const std_msgs::Float64& msg)
{
  rotational_distance_ = msg.data;
}

void Base::handleNavigationEnable(const std_msgs::Bool& msg)
{
  navigation_enabled_ = msg.data;
}

void Base::handleSteeringAlign(const std_msgs::Bool& msg)
{
  steering_aligned_ = msg.data;
}

void Base::handleManualCmdVel(const geometry_msgs::Twist& msg)
{
  manual_cmd_vel_ = msg;
  manual_cmd_vel_timeout_ = 0.0;
}

}  // namespace agv05

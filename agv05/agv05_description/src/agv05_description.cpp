/*
 * Copyright (c) 2019, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <agv05_description/DescriptionConfig.h>
#include <tf2/LinearMath/Matrix3x3.h>


void callbackBroadcast(tf2_ros::Buffer& tfBuffer, const std::string& preset, const std::string& frame_id,
                       double x_offset, double y_offset, double z_offset,
                       double roll_offset, double pitch_offset, double yaw_offset)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  try
  {
    geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
          "base", preset, ros::Time(0), ros::Duration(0.5));

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "base";
    transform.child_frame_id = frame_id;

    transform.transform.translation.x += x_offset;
    transform.transform.translation.y += y_offset;
    transform.transform.translation.z += z_offset;

    tf2::Quaternion quat(transform.transform.rotation.x, transform.transform.rotation.y,
                         transform.transform.rotation.z, transform.transform.rotation.w);

    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    quat.setRPY(roll + roll_offset * M_PI / 180.0, pitch + pitch_offset * M_PI / 180.0, yaw + yaw_offset * M_PI / 180.0);
    quat.normalize();

    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }
}

void callbackConfig(const agv05_description::DescriptionConfig& config, uint32_t level)
{
  ROS_INFO("agv05_description: config received.");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

#define CALLBACK_BROADCAST_LASER(laser) \
  callbackBroadcast(tfBuffer, #laser "_preset", #laser, \
                    config.laser##_x_offset, config.laser##_y_offset, 0.0, 0.0, 0.0, config.laser##_yaw_offset)

  CALLBACK_BROADCAST_LASER(laser);
  CALLBACK_BROADCAST_LASER(laser2);
  CALLBACK_BROADCAST_LASER(laser3);
  CALLBACK_BROADCAST_LASER(laser4);
  CALLBACK_BROADCAST_LASER(laser5);
  CALLBACK_BROADCAST_LASER(laser6);

#undef CALLBACK_BROADCAST_LASER

#define CALLBACK_BROADCAST(sensor) \
  callbackBroadcast(tfBuffer, #sensor "_preset", #sensor "_link", \
                    config.sensor##_x_offset, config.sensor##_y_offset, config.sensor##_z_offset, \
                    config.sensor##_roll_offset, config.sensor##_pitch_offset, config.sensor##_yaw_offset)

  CALLBACK_BROADCAST(camera1);
  CALLBACK_BROADCAST(camera2);
  CALLBACK_BROADCAST(camera3);
  CALLBACK_BROADCAST(camera4);
  CALLBACK_BROADCAST(camera5);
  CALLBACK_BROADCAST(imu);

#undef CALLBACK_BROADCAST
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_description");
  ROS_INFO("Node started.");

  // dynamic_reconfigure
  dynamic_reconfigure::Server<agv05_description::DescriptionConfig> server;
  server.setCallback(callbackConfig);

  ros::spin();
}

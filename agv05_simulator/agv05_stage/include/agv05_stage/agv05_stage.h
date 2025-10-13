/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_STAGE_AGV05_STAGE_H
#define AGV05_STAGE_AGV05_STAGE_H

#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <stage.hh>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>


namespace agv05
{

class StageNode
{
public:
  StageNode(int argc, char** argv);
  void subscribeModels();
  void spin()
  {
    world_->Start();
    Stg::World::Run();
  }

private:
  // ROS callbacks
  void cmdVelCallback(size_t r, const geometry_msgs::TwistConstPtr& msg);

  // Stage update callback
  void worldCallback();
  static int worldCb(Stg::World* world, void* node)
  {
    reinterpret_cast<StageNode*>(node)->worldCallback();
    // We return 0 to indicate that we want to be called again (an
    // odd convention, but that's the way that Stage works).
    return 0;
  }

  // A helper function that is executed for each stage model. We use it
  // to search for models of interest.
  void graphCallback(Stg::Model* model);
  static int graphCb(Stg::Model* model, void* node)
  {
    reinterpret_cast<StageNode*>(node)->graphCallback(model);
    return 0;
  }

  // Append the given robot ID to the given message name.
  const char* mapName(const char* name, size_t robot_id, Stg::Model* model) const;
  const char* mapName(const char* name, size_t robot_id, size_t device_id, Stg::Model* model) const;

private:
  ros::NodeHandle nh_;

  // A mutex to lock access to fields that are used in message callbacks
  typedef boost::mutex::scoped_lock Lock;
  boost::mutex mutex_;

  // The main simulator object
  Stg::World* world_;

  // A structure representing a robot in the simulator
  struct StageRobot
  {
    // Stage-related models
    Stg::ModelPosition* position_model;
    std::vector<Stg::ModelRanger*> laser_models;

    // ROS publishers
    ros::Publisher odom_pub;
    ros::Publisher ground_truth_pub;
    std::vector<ros::Publisher> laser_pubs;

    // ROS subscriber
    ros::Subscriber cmdvel_sub;

    // Last time that we received a velocity command
    ros::Time base_last_cmd_time;

    // Last published global pose
    Stg::Pose base_last_global_pose;
  };

  std::vector<StageRobot> robots_;

  // ROS TF broadcaster
  tf2_ros::TransformBroadcaster tf_;
  tf2_ros::StaticTransformBroadcaster stf_;

  // ROS publisher
  ros::Publisher clock_pub_;

  // Current simulation time
  ros::Time sim_time_;
  ros::Duration sim_time_offset_;

  // Misc options
  bool use_model_names_;
  ros::Duration base_watchdog_timeout_;

  // Last time we saved global position (for velocity calculation).
  ros::Time base_last_global_pose_time_;
};

}  // namespace agv05

#endif  // AGV05_STAGE_AGV05_STAGE_H

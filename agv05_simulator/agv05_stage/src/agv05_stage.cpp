/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_stage/agv05_stage.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <tf2/LinearMath/Quaternion.h>

// topic names
#define SCAN "scan"
#define ODOM "odom"
#define GROUND_TRUTH "ground_truth"
#define CMD_VEL "cmd_vel"
// frame ids
#define LASER "laser"
#define BASE "base"
#define ODOM "odom"
#define MAP "map"
#define GROUND_TRUTH "ground_truth"


namespace agv05
{

StageNode::StageNode(int argc, char** argv)
{
  // Read ROS parameters
  ros::NodeHandle private_nh("~");
  std::string world_file;
  if (!private_nh.getParam("world_file", world_file) || world_file.empty())
  {
    ROS_FATAL("The `world_file` parameter is not set.");
    exit(-1);
  }
  // We'll check the existence of the world file, because libstage(<=4.1.1)
  // doesn't expose its failure to open it.
  struct stat s;
  if (stat(world_file.c_str(), &s) != 0)
  {
    ROS_FATAL_STREAM("The world file \"" << world_file << "\" does not exist.");
    exit(-1);
  }

  double t;
  private_nh.param("base_watchdog_timeout", t, 0.2);
  base_watchdog_timeout_.fromSec(t);

  bool use_gui;
  private_nh.param("use_gui", use_gui, true);
  private_nh.param("use_model_names", use_model_names_, false);

  nh_.param("/last_clock", t, 0.0);
  sim_time_offset_.fromSec(t);

  // Setup simulation clock
  nh_.setParam("/use_sim_time", true);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

  // Initialize libstage
  Stg::Init(&argc, &argv);

  if (use_gui)
  {
    world_ = new Stg::WorldGui(600, 400, "Stage (ROS)");
  }
  else
  {
    world_ = new Stg::World();
  }
  world_->Load(world_file);

  world_->ForEachDescendant(graphCb, this);
  world_->AddUpdateCallback(worldCb, this);
}

// Subscribe to models of interest.
void StageNode::subscribeModels()
{
  for (size_t r = 0; r < robots_.size(); r++)
  {
    StageRobot& robot = robots_[r];
    Stg::ModelPosition* position_model = robot.position_model;

    position_model->Subscribe();
    ROS_INFO_STREAM("Subscribed to Stage position model \"" << position_model->Token() << "\"");

    robot.odom_pub =
      nh_.advertise<nav_msgs::Odometry>(
        mapName(ODOM, r, position_model), 10);
    robot.ground_truth_pub =
      nh_.advertise<nav_msgs::Odometry>(
        mapName(GROUND_TRUTH, r, position_model), 10);

    robot.cmdvel_sub =
      nh_.subscribe<geometry_msgs::Twist>(
        mapName(CMD_VEL, r, position_model), 10,
        boost::bind(&StageNode::cmdVelCallback, this, r, _1));

    for (size_t s = 0; s < robot.laser_models.size(); s++)
    {
      Stg::ModelRanger* laser_model = robot.laser_models[s];

      laser_model->Subscribe();
      ROS_INFO_STREAM("Subscribed to Stage ranger \"" << laser_model->Token() << "\"");

      geometry_msgs::TransformStamped ts;
      ts.header.frame_id = mapName(BASE, r, position_model);

      Stg::Pose lp = position_model->GlobalToLocal(laser_model->GetGlobalPose());
      ts.transform.translation.x = lp.x;
      ts.transform.translation.y = lp.y;
      ts.transform.translation.z = lp.z;
      tf2::Quaternion q;
      q.setRPY(0, 0, lp.a);
      ts.transform.rotation.x = q.x();
      ts.transform.rotation.y = q.y();
      ts.transform.rotation.z = q.z();
      ts.transform.rotation.w = q.w();

      if (robot.laser_models.size() == 1)
      {
        robot.laser_pubs.push_back(
          nh_.advertise<sensor_msgs::LaserScan>(
            mapName(SCAN, r, position_model), 10));

        ts.child_frame_id = mapName(LASER, r, position_model);
      }
      else
      {
        robot.laser_pubs.push_back(
          nh_.advertise<sensor_msgs::LaserScan>(
            mapName(SCAN, r, s, position_model), 10));

        ts.child_frame_id = mapName(LASER, r, s, position_model);
      }

      stf_.sendTransform(ts);
    }

    ROS_INFO_STREAM("Robot " << position_model->Token() << " provided " << robot.laser_models.size() << " rangers.");
  }
}

// ROS callbacks
void StageNode::cmdVelCallback(size_t r, const geometry_msgs::TwistConstPtr& msg)
{
  Lock lock(mutex_);
  StageRobot& robot = robots_[r];
  robot.position_model->SetSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
  robot.base_last_cmd_time = sim_time_;
}

// Stage update callback
void StageNode::worldCallback()
{
  if (!ros::ok())
  {
    ROS_INFO("ros::ok() is false. Quitting.");
    world_->QuitAll();
    return;
  }

  Lock lock(mutex_);
  sim_time_.fromSec(world_->SimTimeNow() / 1e6);
  // We're not allowed to publish clock==0, because it is used as a special
  // value in parts of ROS, #4027.
  if (sim_time_.sec == 0 && sim_time_.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publish clock==0");
    return;
  }
  sim_time_ += sim_time_offset_;

  // loop on the robot models
  for (size_t r = 0; r < robots_.size(); r++)
  {
    StageRobot& robot = robots_[r];
    Stg::ModelPosition* position_model = robot.position_model;

    // stop robot if no cmd_vel received after watchdog timeout expired.
    if (base_watchdog_timeout_.toSec() > 0.0 &&
        sim_time_ - robot.base_last_cmd_time >= base_watchdog_timeout_)
    {
      position_model->SetSpeed(0.0, 0.0, 0.0);
    }

    // loop on the laser devices for the current robot
    for (size_t s = 0; s < robot.laser_models.size(); s++)
    {
      const Stg::ModelRanger* laser_model = robot.laser_models[s];
      const std::vector<Stg::ModelRanger::Sensor>& sensors = laser_model->GetSensors();

      if (sensors.size() > 1)
      {
        ROS_WARN_ONCE("ROS Stage currently supports rangers with 1 sensor only.");
      }

      // for now we access only the zeroth sensor of the ranger - good
      // enough for most laser models that have a single beam origin
      const Stg::ModelRanger::Sensor& sensor = sensors[0];

      if (sensor.ranges.size())
      {
        // Translate into ROS message format and publish
        sensor_msgs::LaserScan msg;
        msg.angle_min = -sensor.fov / 2.0;
        msg.angle_max = +sensor.fov / 2.0;
        msg.angle_increment = sensor.fov / (sensor.sample_count - 1);
        msg.range_min = sensor.range.min;
        msg.range_max = sensor.range.max;
        msg.ranges.resize(sensor.ranges.size());
        msg.intensities.resize(sensor.intensities.size());

        for (size_t i = 0; i < sensor.ranges.size(); i++)
        {
          msg.ranges[i] = sensor.ranges[i];
          msg.intensities[i] = sensor.intensities[i];
        }

        if (robot.laser_models.size() == 1)
        {
          msg.header.frame_id = mapName(LASER, r, position_model);
        }
        else
        {
          msg.header.frame_id = mapName(LASER, r, s, position_model);
        }

        msg.header.stamp = sim_time_;
        robot.laser_pubs[s].publish(msg);
      }
    }

    // Get latest odometry data
    // Translate into ROS message format and publish
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = sim_time_;
    odom_msg.header.frame_id = mapName(ODOM, r, position_model);
    odom_msg.child_frame_id = mapName(BASE, r, position_model);

    Stg::Pose p = position_model->est_pose;
    odom_msg.pose.pose.position.x = p.x;
    odom_msg.pose.pose.position.y = p.y;
    odom_msg.pose.pose.position.z = p.z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.a);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    Stg::Velocity v = position_model->GetVelocity();
    odom_msg.twist.twist.linear.x = v.x;
    odom_msg.twist.twist.linear.y = v.y;
    odom_msg.twist.twist.angular.z = v.a;

    robot.odom_pub.publish(odom_msg);

    // Broadcast odometry transform
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = sim_time_;
    ts.header.frame_id = odom_msg.header.frame_id;
    ts.child_frame_id = odom_msg.child_frame_id;
    ts.transform.translation.x = p.x;
    ts.transform.translation.y = p.y;
    ts.transform.translation.z = p.z;
    ts.transform.rotation = odom_msg.pose.pose.orientation;
    tf_.sendTransform(ts);

    // Also publish the ground truth odometry and transform
    odom_msg.header.frame_id = mapName(MAP, r, position_model);
    odom_msg.child_frame_id = mapName(GROUND_TRUTH, r, position_model);

    p = position_model->GetGlobalPose();
    odom_msg.pose.pose.position.x = p.x;
    odom_msg.pose.pose.position.y = p.y;
    odom_msg.pose.pose.position.z = p.z;
    q.setRPY(0.0, 0.0, p.a);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    v = Stg::Velocity();
    if (base_last_global_pose_time_.toSec() > 0.0)
    {
      Stg::Pose pp = robot.base_last_global_pose;
      double dt = (sim_time_ - base_last_global_pose_time_).toSec();
      if (dt > 0.0)
      {
        v = Stg::Velocity(
              (p.x - pp.x) / dt,
              (p.y - pp.y) / dt,
              (p.z - pp.z) / dt,
              Stg::normalize(p.a - pp.a) / dt);
      }
    }
    robot.base_last_global_pose = p;

    odom_msg.twist.twist.linear.x = v.x;
    odom_msg.twist.twist.linear.y = v.y;
    odom_msg.twist.twist.angular.z = v.a;

    robot.ground_truth_pub.publish(odom_msg);

    ts.header.frame_id = odom_msg.header.frame_id;
    ts.child_frame_id = odom_msg.child_frame_id;
    ts.transform.translation.x = p.x;
    ts.transform.translation.y = p.y;
    ts.transform.translation.z = p.z;
    ts.transform.rotation = odom_msg.pose.pose.orientation;
    tf_.sendTransform(ts);
  }

  base_last_global_pose_time_ = sim_time_;

  rosgraph_msgs::Clock clock_msg;
  clock_msg.clock = sim_time_;
  clock_pub_.publish(clock_msg);
  nh_.setParam("/last_clock", sim_time_.toSec());
}

// A helper function that is executed for each stage model. We use it
// to search for models of interest.
void StageNode::graphCallback(Stg::Model * model)
{
  if (dynamic_cast<Stg::ModelPosition*>(model))
  {
    size_t r = robots_.size();
    robots_.resize(r + 1);
    robots_[r].position_model = dynamic_cast<Stg::ModelPosition*>(model);
    return;
  }

  if (robots_.empty())
  {
    return;
  }
  StageRobot& robot = robots_.back();

  if (dynamic_cast<Stg::ModelRanger*>(model))
  {
    if (model->Parent() == robot.position_model)
    {
      robot.laser_models.push_back(dynamic_cast<Stg::ModelRanger*>(model));
    }
  }
}

// Append the given robot ID to the given message name.
const char* StageNode::mapName(const char* name, size_t robot_id, Stg::Model * model) const
{
  static char buf[100];
  if (robots_.size() > 1 || use_model_names_)
  {
    bool found = std::string(model->Token()).find(":") != std::string::npos;
    if (!found && use_model_names_)
    {
      snprintf(buf, sizeof(buf), "/%s/%s", model->Token(), name);
    }
    else
    {
      snprintf(buf, sizeof(buf), "/robot_%lu/%s", robot_id, name);
    }
    return buf;
  }
  else
  {
    return name;
  }
}

const char* StageNode::mapName(const char* name, size_t robot_id, size_t device_id, Stg::Model * model) const
{
  static char buf[100];
  if (robots_.size() > 1 || use_model_names_)
  {
    bool found = std::string(model->Token()).find(":") != std::string::npos;
    if (!found && use_model_names_)
    {
      snprintf(buf, sizeof(buf), "/%s/%s_%lu", model->Token(), name, device_id);
    }
    else
    {
      snprintf(buf, sizeof(buf), "/robot_%lu/%s_%lu", robot_id, name, device_id);
    }
  }
  else
  {
    snprintf(buf, sizeof(buf), "/%s_%lu", name, device_id);
  }
  return buf;
}

}  // namespace agv05


void mySigintHandler(int sig)
{
  delete Fl::first_window();
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_stage", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigintHandler);

  agv05::StageNode sn(argc, argv);
  sn.subscribeModels();

  boost::thread t = boost::thread(boost::bind(&ros::spin));
  sn.spin();
  t.join();
}

/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/components.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>


namespace agv05
{

BaseX::BaseX(ros::NodeHandle& nh) :
  Base(nh), tfl_(tf_),
  init_planner_(false),
  planner_thread_rate_(2.0), planner_thread_shutdown_(true),
  has_new_goal_(false), goal_active_(false),
  has_new_plan_(false), plan_active_(false)
{
  ros::NodeHandle private_nh("~");
  docking_plan_pub_ = private_nh.advertise<nav_msgs::Path>("docking/plan", 1, true);
  marker_type_pub_ = nh.advertise<std_msgs::String>("marker_type", 1, true);
  default_path_pub_ = nh.advertise<agv05_msgs::Path>("map/static_path", 1, true);

  tf_static_sub_ = nh.subscribe("tf_static", 1, &BaseX::handleTfStatic, this);

  char *dynplan = getenv("DYNAMIC_PATH_PLANNING");
  if (dynplan)
  {
    init_planner_ = !strcasecmp(dynplan, "true") || atoi(dynplan);
    planner_thread_shutdown_ = !init_planner_;
    ROS_INFO("DYNAMIC_PATH_PLANNING: %s (%d)", dynplan, init_planner_);
  }
}

double BaseX::getRobotRadius()
{
  double robot_radius = -1.0;
  if (costmap_)
  {
    geometry_msgs::Polygon footprint = costmap_->getRobotFootprintPolygon();
    double r2 = 0.0;
    for (const auto& point : footprint.points)
    {
      r2 = std::max<double>(r2, point.x * point.x + point.y * point.y);
    }
    robot_radius = std::sqrt(r2);
  }
  return robot_radius;
}

bool BaseX::getPose(geometry_msgs::Pose2D& pose)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_.lookupTransform("map", "base", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Localization error: " << ex.what());
    return false;
  }
  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  pose.theta = tf2::getYaw(transform.transform.rotation);

  if (init_planner_)
  {
    init_planner_ = false;

    dyn_obs_sensor_.setObstacleSensors("costmap");
    costmap_ = boost::make_shared<costmap_2d::Costmap2DROS>("costmap", boost::ref(tf_), true);
    costmap_->stop();

    planner_ = boost::make_shared<global_planner::GlobalPlanner>();
    planner_->initialize("planner", costmap_.get());

    planner_thread_ = boost::thread(&BaseX::plannerThread, this);
  }

  return pose_timeout_ > transform.header.stamp;
}

bool BaseX::getPoseInTarget(geometry_msgs::Pose2D& pose, bool ensureRecent)
{
  geometry_msgs::TransformStamped odom;
  try
  {
    odom = tf_.lookupTransform("odom", "base", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Odom error: " << ex.what());
    return false;
  }

  geometry_msgs::TransformStamped target;
  try
  {
    target = tf_.lookupTransform("target", "odom", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Target error: " << ex.what());
    return false;
  }

  if (ensureRecent)
  {
    if ((ros::Time::now() - target.header.stamp) > ros::Duration(0.5))
    {
      ROS_ERROR_STREAM_THROTTLE(5, "Target error: No recent transform found.");
      return false;
    }
  }

  geometry_msgs::TransformStamped transform;
  tf2::doTransform(odom, transform, target);

  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  pose.theta = tf2::getYaw(transform.transform.rotation);
  return true;
}

void BaseX::sendMarkerTypeAndTarget(const std::string& marker_type, const geometry_msgs::Pose2D& target)
{
  std_msgs::String msg;
  msg.data = marker_type;
  marker_type_pub_.publish(msg);

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "marker";
  transform.header.stamp = ros::Time::now();
  transform.child_frame_id = "target";
  transform.transform.translation.x = target.x;
  transform.transform.translation.y = target.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, target.theta);
  transform.transform.rotation = tf2::toMsg(q);
  stfb_.sendTransform(transform);
}

void BaseX::clearMarkerType()
{
  std_msgs::String msg;
  marker_type_pub_.publish(msg);
}

void BaseX::publishDockingPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = "target";
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = plan;
  docking_plan_pub_.publish(gui_path);
}

bool BaseX::getPlan(std::vector<geometry_msgs::PoseStamped>& plan)
{
  // skip acquiring mutex
  if (!has_new_plan_)
  {
    return false;
  }

  Lock lock(plan_mutex_);
  if (has_new_plan_)
  {
    plan.swap(latest_plan_);
    has_new_plan_ = false;
    return true;
  }
  return false;
}

void BaseX::startPlanner(const geometry_msgs::PoseStamped& goal,
                         const agv05_msgs::Path& default_path,
                         double tolerance)
{
  default_path_pub_.publish(default_path);
  ros::spinOnce();

  {
    Lock lock(goal_mutex_);
    latest_goal_.goal = goal;
    latest_goal_.tolerance = tolerance;
    has_new_goal_ = true;

    // notify planner thread
    goal_cv_.notify_one();
  }

  if (!plan_active_)
  {
    Lock lock(plan_mutex_);
    // wait for 1st plan (timeout after 2 cycles of planner thread)
    plan_cv_.timed_wait(lock, (planner_thread_rate_.expectedCycleTime() * 2.0).toBoost(), [this]() -> bool
    {
      return has_new_plan_;
    });
    plan_active_ = true;
  }
}

void BaseX::stopPlanner(bool plan_active, std::vector<geometry_msgs::PoseStamped>& plan)
{
  {
    Lock lock(goal_mutex_);
    goal_active_ = false;
    goal_stop_after_ = ros::Time::now() + ros::Duration(3.0);
  }

  {
    Lock lock(plan_mutex_);
    plan_active_ = plan_active;
    if (plan_active_ && !has_new_plan_ && !plan.empty())
    {
      latest_plan_.swap(plan);
      has_new_plan_ = true;
    }
  }
}

void BaseX::handleTfStatic(const tf2_msgs::TFMessage& msg)
{
  for (const auto& transform : msg.transforms)
  {
    if (transform.child_frame_id == "odom")
    {
      pose_timeout_ = transform.header.stamp + ros::Duration(SAFETY_HEARTBEAT_TIMEOUT);
    }
  }
}

void BaseX::plannerThread()
{
  bool costmap_stopped = true;

  while (ros::ok() && !planner_thread_shutdown_)
  {
    {
      Lock lock(goal_mutex_);

      if (!has_new_goal_)
      {
        // sleep until next cycle or goal received
        goal_cv_.timed_wait(lock, planner_thread_rate_.nextCycle().toBoost(), [this]() -> bool
        {
          return has_new_goal_;
        });
      }

      if (has_new_goal_)
      {
        planner_goal_ = latest_goal_;
        has_new_goal_ = false;
        goal_active_ = true;

        if (costmap_stopped)
        {
          clear_obstacle_ = false;
          costmap_->start();  // reactivate topic subscriptions
          ros::spinOnce();  // update with latest topic messages
          costmap_stopped = false;
        }
      }
    }

    if (goal_active_)
    {
      ros::Time t = ros::Time::now();
      if (clear_obstacle_)
      {
        dyn_obs_sensor_.clearObstacle();
        clear_obstacle_ = false;
      }
      costmap_->updateMap();
      costmap_->publishMap();
      costmap_sec_ += (ros::Time::now() - t).toSec();

      t = ros::Time::now();
      makePlan();
      make_plan_sec_ += (ros::Time::now() - t).toSec();
    }
    else if (ros::Time::now() < goal_stop_after_)
    {
      ros::Time t = ros::Time::now();
      // update costmap in anticipation of next motion
      costmap_->updateMap();
      costmap_->publishMap();
      costmap_sec_ += (ros::Time::now() - t).toSec();
      if (!plan_active_)
      {
        clearPlan();
      }
    }
    else if (!costmap_stopped)
    {
      costmap_->stop();
      clearPlan();
      costmap_stopped = true;
    }
  }

  clearPlan();
}

void BaseX::makePlan()
{
  geometry_msgs::Pose2D pose;
  if (!getPose(pose))
  {
    return;
  }

  geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position.x = pose.x;
  start.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  start.pose.orientation = tf2::toMsg(q);

  planner_plan_.clear();
  if (planner_goal_.tolerance < 0.0 ?
      planner_->makePlan(start, planner_goal_.goal, planner_plan_) :
      planner_->makePlan(start, planner_goal_.goal, planner_goal_.tolerance, planner_plan_))
  {
    ROS_DEBUG("Make plan successful (%ld).", planner_plan_.size());
  }
  else
  {
    ROS_DEBUG("Make plan failed.");
  }

  Lock lock(plan_mutex_);
  latest_plan_.swap(planner_plan_);
  has_new_plan_ = true;
  if (goal_active_ && !plan_active_)
  {
    plan_cv_.notify_one();
  }
}

void BaseX::clearPlan()
{
  planner_plan_.clear();
  planner_->publishPlan(planner_plan_);  // publish empty plan to clear UI

  Lock lock(plan_mutex_);
  latest_plan_.clear();
  has_new_plan_ = false;
  plan_active_ = false;
}

};  // namespace agv05

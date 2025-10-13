/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAVX_COMPONENTS_H
#define AGV05_NAVX_COMPONENTS_H

#include <agv05_nav/components.h>
#include <agv05_navx/NavConfig.h>
#include <agv05_navx/utils.h>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <agv05_msgs/Path.h>
#include <agv05_msgs/PolygonArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


namespace agv05
{

class DynamicObstacleSensor
{
public:
  DynamicObstacleSensor();
  void setObstacleSensors(const std::string& ns);
  void clearObstacle();

private:
  bool enumServerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  std::string ns_;
  std::vector<ros::ServiceServer> srv_;
};

class BaseX: public Base
{
public:
  explicit BaseX(ros::NodeHandle& nh);

  virtual ~BaseX()
  {
    planner_thread_shutdown_ = true;
    planner_thread_.join();
  }

  void handleConfig(agv05_navx::NavConfig& config, uint32_t level)
  {
    if (std::abs(planner_thread_rate_.expectedCycleTime().toSec() -
        (1.0 / config.dynplan_planner_frequency)) >
        (1.0 / config.__getMax__().dynplan_planner_frequency))
    {
      planner_thread_rate_.reset(config.dynplan_planner_frequency);
    }
  }

  double getRobotRadius();

  bool getPose(geometry_msgs::Pose2D& pose);
  bool getPoseInTarget(geometry_msgs::Pose2D& pose, bool ensureRecent = false);

  void sendMarkerTypeAndTarget(const std::string& marker_type, const geometry_msgs::Pose2D& target);
  void clearMarkerType();
  void publishDockingPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  bool getPlan(std::vector<geometry_msgs::PoseStamped>& plan);

  void startPlanner(const geometry_msgs::PoseStamped& goal,
                    const agv05_msgs::Path& default_path,
                    double tolerance);
  void stopPlanner(bool plan_active, std::vector<geometry_msgs::PoseStamped>& plan);

  void clearCostmapObstacle()
  {
    clear_obstacle_ = true;
  }

  bool diagPlanner()
  {
    return !planner_thread_shutdown_;
  }
  double getCostmapSec()
  {
    return costmap_sec_;
  }
  double getMakePlanSec()
  {
    return make_plan_sec_;
  }

private:
  void handleTfStatic(const tf2_msgs::TFMessage& msg);

  void plannerThread();
  void makePlan();
  void clearPlan();

private:
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tfl_;
  tf2_ros::StaticTransformBroadcaster stfb_;

  ros::Publisher docking_plan_pub_;
  ros::Publisher marker_type_pub_;
  ros::Publisher default_path_pub_;

  ros::Subscriber tf_static_sub_;

  ros::Time pose_timeout_;

  DynamicObstacleSensor dyn_obs_sensor_;
  bool clear_obstacle_;
  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  boost::shared_ptr<global_planner::GlobalPlanner> planner_;
  bool init_planner_;

  boost::thread planner_thread_;
  Rate planner_thread_rate_;
  bool planner_thread_shutdown_;

  typedef boost::mutex::scoped_lock Lock;

  struct
  {
    geometry_msgs::PoseStamped goal;
    double tolerance;
  } latest_goal_, planner_goal_;
  ros::Time goal_stop_after_;  // delay stopping costmap
  bool has_new_goal_;
  bool goal_active_;
  boost::mutex goal_mutex_;
  boost::condition_variable goal_cv_;

  std::vector<geometry_msgs::PoseStamped> planner_plan_;
  std::vector<geometry_msgs::PoseStamped> latest_plan_;
  bool has_new_plan_;
  bool plan_active_;
  boost::mutex plan_mutex_;
  boost::condition_variable plan_cv_;

  class MeanFilter
  {
  public:
    MeanFilter() : n_(0), sum_(0.0) {}
    operator double()
    {
      Lock lock(mutex_);
      if (n_)
      {
        sum_ /= n_;
        n_ = 0;
      }
      return sum_;
    }
    void operator += (double value)
    {
      Lock lock(mutex_);
      sum_ = n_++ ? sum_ + value : value;
    }
  private:
    boost::mutex mutex_;
    size_t n_;
    double sum_;
  } costmap_sec_, make_plan_sec_;
};

};  // namespace agv05

#endif  // AGV05_NAVX_COMPONENTS_H

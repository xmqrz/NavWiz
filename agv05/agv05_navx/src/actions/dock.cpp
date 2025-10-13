/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"

#include <tf2/utils.h>


namespace agv05
{

void ActionDock::initialize()
{
  uint8_t nav = goal_.nav;
  geometry_msgs::Pose2D target_2d = goal_.path_end;

  if (nav == Goal::NAV_FORWARD_DOCK || nav == Goal::NAV_REVERSE_DOCK || nav == Goal::NAV_OMNI_DOCK)
  {
    undock_ = false;
  }
  else if (nav == Goal::NAV_FORWARD_UNDOCK || nav == Goal::NAV_REVERSE_UNDOCK || nav == Goal::NAV_OMNI_UNDOCK)
  {
    undock_ = true;

    if (nav == Goal::NAV_OMNI_UNDOCK && goal_.path_cp2.theta > 0.0)
    {
      target_2d.theta = goal_.path_cp2.theta;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, target_2d.theta);
    tf2::Transform target(q, tf2::Vector3(target_2d.x, target_2d.y, 0));

    q.setRPY(0, 0, goal_.path_start.theta);
    tf2::Transform undock_end(q, tf2::Vector3(goal_.path_start.x, goal_.path_start.y, 0));
    undock_end = target.inverseTimes(undock_end);

    undock_end_.x = undock_end.getOrigin().x();
    undock_end_.y = undock_end.getOrigin().y();
    undock_end_.theta = tf2::getYaw(undock_end.getRotation());

    ROS_DEBUG("Undock End: %f %f %f", undock_end_.x, undock_end_.y, undock_end_.theta);
  }

  base_.sendMarkerTypeAndTarget(goal_.marker_type, target_2d);
}

void ActionDock::planning(float frequency)
{
  ROS_ASSERT(plan_.empty());

  speed_.setSpeed(0.0f);
  speed_.addSpeedLimit(0.0f);
  base_.stop();

  if (isAborted() || planning_retries_ <= 0)
  {
    completeNav(Result::RESULT_FAIL);
    return;
  }

  if (planning_throttle_.update(true, 1.0f / frequency))
  {
    planning_throttle_.setState(false);
    if (generatePlan())
    {
      state_ = RUNNING;
    }
    --planning_retries_;
  }
}

bool ActionDock::generatePlan()
{
  if (!base_.getPoseInTarget(pose_, true)) return false;

  // clear the plan, just in case
  plan_.clear();

  // from coordinate
  geometry_msgs::Pose2D from = undock_ ? undock_end_ : pose_;

  // Vasiljevic, 2016
  // An S-shaped docking path where the start and end orientations are the same
  double offset = std::abs(goal_.path_cp2.x);
  double xr = offset - from.x;
  double yr = -from.y;
  double tr = -from.theta;
  ROS_DEBUG("xr: %f, yr: %f, tr: %f", xr, yr, tr);

  double a, b;
  if (std::abs(yr) < 1e-6)
  {
    // straight line path
    a = b = 0;
  }
  else
  {
    a = yr / 2.0;
    if (xr == 0)  // prevent division by zero
    {
      b = M_PI / 0.001;
    }
    else
    {
      b = M_PI / xr;
    }
  }

  double sign = xr / std::abs(xr);
  double x_inc = 0.02 * sign;

  // special case: generate fully straight plan
  if (offset == 0)
  {
    offset = from.x;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "target";
  pose.pose.orientation.w = 1.0;

  for (double x = from.x; (x - offset) * sign < 0; x = x + x_inc)
  {
    double y = -a * (1.0 - cos(b * (x - offset)));

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    plan_.push_back(pose);
  }

  pose.pose.position.y = 0;
  for (double x = offset; x > 0; x -= 0.02)
  {
    pose.pose.position.x = x;
    plan_.push_back(pose);
  }

  pose.pose.position.x = 0;
  plan_.push_back(pose);

  if (undock_) std::reverse(plan_.begin(), plan_.end());
  base_.publishDockingPlan(plan_);

  // Accumulate target distance from the end
  size_t i = plan_.size();
  plan_distances_.clear();
  plan_distances_.resize(i);

  complex p = planComplex(--i);
  double target_distance = 0.0;
  plan_distances_[i] = target_distance;

  while (i > 0)
  {
    complex next_p = planComplex(--i);
    target_distance += std::abs(p - next_p);
    p = next_p;
    plan_distances_[i] = target_distance;
  }

  return true;
}

bool ActionDock::advancePlanCursor0()
{
  if (plan_.empty()) return true;

  if (!base_.getPoseInTarget(pose_)) return false;

  DigitalInputFilter<int> termination_filter(false, 0, 10);
  const complex pose = poseComplex();

  double target_squared_distance = std::numeric_limits<double>::max();
  for (size_t i = plan_cur_ + 1; i < plan_.size(); ++i)
  {
    complex pose_v = pose - planComplex(i);
    double squared_distance = std::norm(pose_v);

    if (squared_distance <= target_squared_distance)
    {
      target_squared_distance = squared_distance;
      plan_cur_ = i - 1;
      termination_filter.update(false, 1);
    }
    else
    {
      termination_filter.update(true, 1);
      if (termination_filter.isStalled())
      {
        break;
      }
    }
  }

  // Advance cursor (+1) if pose is ahead of cursor pose.
  if (plan_cur_ < plan_.size() - 2)
  {
    const complex pp0 = planComplex(plan_cur_);
    const complex pp = planComplex(plan_cur_ + 1);
    const complex v = pp - pp0;
    const complex pose_v = pose - pp;
    // float distance = -dot(v, pose_v) / std::abs(v);
    float direction = -dot(v, pose_v);
    if (direction <= 0)
    {
      // ROS_INFO("ADVANCE CURSOR");
      ++plan_cur_;
    }
  }

  return true;
}

float ActionDock::getTargetDistance0()
{
  if (io_trigger_) return ActionStraight::getTargetDistance0();
  if (plan_.empty()) return 1000.0f;

  ROS_ASSERT(plan_cur_ <= plan_.size() - 2);
  const complex pp0 = planComplex(plan_cur_);
  const complex pp = planComplex(plan_cur_ + 1);
  const complex v = pp - pp0;
  const complex pose_v = poseComplex() - pp;
  float distance = -dot(v, pose_v) / std::abs(v) + plan_distances_[plan_cur_ + 1];

  // ROS_INFO("Target Distance: %f, %ld", distance, plan_.size());
  if (undock_) return distance;
  return adjustTargetDistanceByLineSensing0(distance);
}

float ActionDock::getLinearError0()
{
  if (plan_.empty()) return 0.0f;

  ROS_ASSERT(plan_cur_ <= plan_.size() - 2);
  const complex v = planComplex(std::min(plan_cur_ + 5, plan_.size() - 1)) - planComplex(plan_cur_);
  const complex pose_v = poseComplex() - planComplex(plan_cur_ + 1);
  linear_error_ = cross(v, pose_v) / std::abs(v);  // update protected variable
  return linear_error_;
}

float ActionDock::getHeadingError0()
{
  if (plan_.empty()) return 0.0f;

  ROS_ASSERT(plan_cur_ <= plan_.size() - 2);
  const complex v = planComplex(std::min(plan_cur_ + 5, plan_.size() - 1)) - planComplex(plan_cur_);
  float heading_error = pose_.theta - std::arg(v);
  // flip heading by 180 deg if reverse direction
  if (!forward_dir_)
  {
    heading_error += M_PI;
  }

  // normalize
  heading_error = fmod(heading_error, 2 * M_PI);
  if (heading_error > M_PI)
  {
    heading_error -= 2 * M_PI;
  }
  else if (heading_error <= -M_PI)
  {
    heading_error += 2 * M_PI;
  }
  heading_error_ = heading_error;  // update protected variable
  return heading_error;
}

}  // namespace agv05

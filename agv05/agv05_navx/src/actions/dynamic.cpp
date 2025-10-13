/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"

#include <tf2/utils.h>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/create_straight_skeleton_2.h>

#define STOP_THRESHOLD_SPEED 0.001f


namespace agv05
{

void ActionDynamic::initialize()
{
  agv05_msgs::Path default_path = goal_.paths;

  junction_mid_ = default_path.paths.size() > 1;
  target_tolerance_ = std::abs(goal_.goal_tolerance - Goal::TOLERANCE_GOAL) < 0.5f ? config_.dynplan_tolerance_goal :
                      std::abs(goal_.goal_tolerance - Goal::TOLERANCE_MID) < 0.5f ? config_.dynplan_tolerance_mid :
                      goal_.goal_tolerance;
  target_dist_max_ = ActionStraight::reaching_.distance(path_speed_, config_.getProfile().straight_normal_acc) +
                     config_.dynplan_tolerance_path;

  switch (goal_.nav)
  {
  case Goal::NAV_DYNAMIC_FORWARD:
  case Goal::NAV_DYNAMIC_REVERSE:
  case Goal::NAV_DYNAMIC_OMNI:
    default_path.paths.resize(0);
    // default_path.area.points.resize(0);

    path_distance_ = path_dist_max_ = -1.0f;
    junction_mid_ = false;  // goal_.next_motion must be Goal::MOTION_IDLE
    break;

  case Goal::NAV_DYNAMIC_LINE_FORWARD:
  case Goal::NAV_DYNAMIC_LINE_REVERSE:
  case Goal::NAV_DYNAMIC_LINE_OMNI:
  case Goal::NAV_DYNAMIC_BEZIER_FORWARD:
  case Goal::NAV_DYNAMIC_BEZIER_REVERSE:
  case Goal::NAV_DYNAMIC_BEZIER_OMNI:
    if (!default_path.paths.size())
    {
      geometry_msgs::Point32 p;

      default_path.paths.resize(1);
      default_path.paths[0].points.resize(0);

      p.x = goal_.path_start.x;
      p.y = goal_.path_start.y;
      default_path.paths[0].points.push_back(p);

      if (goal_.nav == Goal::NAV_DYNAMIC_BEZIER_FORWARD ||
          goal_.nav == Goal::NAV_DYNAMIC_BEZIER_REVERSE ||
          goal_.nav == Goal::NAV_DYNAMIC_BEZIER_OMNI)
      {
        p.x = goal_.path_cp1.x;
        p.y = goal_.path_cp1.y;
        default_path.paths[0].points.push_back(p);

        p.x = goal_.path_cp2.x;
        p.y = goal_.path_cp2.y;
        default_path.paths[0].points.push_back(p);
      }

      p.x = goal_.path_end.x;
      p.y = goal_.path_end.y;
      default_path.paths[0].points.push_back(p);
    }
    if (config_.straight_out_of_path_distance > 0.0)
    {
      path_area_cur_ = getPathArea(default_path.paths[0]);
    }

    path_distance_ = goal_.next_distance > 0.0f ? goal_.next_distance : getBezierLength(default_path.paths[0]);
    path_dist_max_ = path_distance_ * config_.dynplan_distance_max * 0.01;
    if (target_tolerance_ > 0.0f)
    {
      path_dist_max_ += target_tolerance_;
    }
    break;

  default:
    ROS_ASSERT_MSG(0, "Invalid goal.");
    break;
  }

  if (default_path.area.points.size())
  {
    for (size_t i = 0; i < default_path.area.points.size(); i++)
    {
      path_area_goal_.push_back(ActionDynamic::Point_2(default_path.area.points[i].x, default_path.area.points[i].y));
    }
  }
  else if (default_path.paths.size() && (config_.straight_out_of_path_distance > 0.0))
  {
    std::list<CGAL::Polygon_with_holes_2<ActionDynamic::Kernel>> polies, polies_wh;

    for (size_t i = 0; i < default_path.paths.size(); i++)
    {
      ActionDynamic::Polygon_2 poly = getPathArea(default_path.paths[i]);
      if (poly.orientation() == CGAL::CLOCKWISE)
      {
        poly.reverse_orientation();
      }
      polies.push_back(CGAL::Polygon_with_holes_2<ActionDynamic::Kernel>(poly));
    }

    CGAL::join(polies.begin(), polies.end(), std::back_inserter(polies_wh));
    path_area_goal_ = polies_wh.begin()->outer_boundary();

    for (ActionDynamic::Polygon_2::Vertex_const_iterator vi = path_area_goal_.vertices_begin();
         vi != path_area_goal_.vertices_end();
         ++vi)
    {
      geometry_msgs::Point32 p;
      p.x = CGAL::to_double(vi->x());
      p.y = CGAL::to_double(vi->y());
      default_path.area.points.push_back(p);
    }
  }

  geometry_msgs::PoseStamped goal;
  goal.header.seq = ++plan_seq_;
  goal.header.frame_id = "map";

  if (default_path.paths.size())
  {
    size_t i = default_path.paths.size() - 1;
    size_t j = default_path.paths[i].points.size() - 1;
    goal.pose.position.x = default_path.paths[i].points[j].x;
    goal.pose.position.y = default_path.paths[i].points[j].y;
  }
  else
  {
    goal.pose.position.x = goal_.path_end.x;
    goal.pose.position.y = goal_.path_end.y;
  }

  double yaw = goal_.path_end.theta;
  tf2::Quaternion q;
  q.setRPY(0.0f, 0.0f, yaw);
  goal.pose.orientation = tf2::toMsg(q);

  base_.startPlanner(goal, default_path, target_tolerance_);

  plan_time_ = ros::Time::now();

  float robot_radius = base_.getRobotRadius();
  if (robot_radius > 0.0)
  {
    path_curvature_ = std::min(path_curvature_, 1.0f / robot_radius);
  }
}

void ActionDynamic::handleNavControl(const NavControl& nav_control)
{
  if (nav_control.control == NavControl::CONTROL_SAFETY_RESUME && plan_.empty())
  {
    if ((plan_time_ + ros::Duration(config_.dynplan_retry_time)) < ros::Time::now())
    {
      plan_time_ = ros::Time::now();
      base_.clearCostmapObstacle();
    }
  }

  switch (state_)
  {
  case RUNNING: ActionStraight::handleNavControl(nav_control); break;
  case TURNING: ActionTurn::handleNavControl(nav_control); break;
  default: ActionProcessor::handleNavControl(nav_control); break;
  }
}

uint8_t ActionDynamic::getStatus()
{
  uint8_t status = ActionProcessor::getStatus();
  manual_resume_ |= (status == Feedback::STATUS_PLAN_EMPTY);
  return status;
}

void ActionDynamic::completeNav(uint8_t result)
{
  if ((result != Result::RESULT_SUCCESS) ||
      (state_ != RUNNING) ||
      (goal_.next_motion != Goal::MOTION_IDLE) ||
      (goal_.path_end.theta <= 0.0f))
  {
    state_ = STOP_EXIT;
    ActionProcessor::completeNav(result);
  }
  else
  {
    state_ = TURNING;
    ActionTurn::pose_ = ActionStraight::pose_;
    left_dir_ = fmod(goal_.path_end.theta - ActionStraight::pose_.theta + M_PI, 2 * M_PI) > M_PI;
    if (forward_dir_)
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_FORWARD);
    }
    else
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_REVERSE);
    }
  }
}

uint8_t ActionDynamic::checkSafetyStatus(bool enable_sensor, bool resume, bool trigger_navigation_failed, UseLineSensor use_line_sensor)
{
  bool plan_empty = plan_.empty();

  if (auto_resume_)
  {
    switch (safety_status_)
    {
    case Feedback::STATUS_NORMAL:
    case Feedback::STATUS_SAFETY_TRIGGERED:
    case Feedback::STATUS_PLAN_EMPTY:
      resume |= !plan_empty;
      break;
    default:
      auto_resume_ = false;
      break;
    }
  }

  if (plan_empty && !path_area_goal_.is_empty())
  {
    const complex pose = poseComplex();
    if (path_area_goal_.bounded_side(ActionDynamic::Point_2(pose.real(), pose.imag())) == CGAL::ON_UNBOUNDED_SIDE)
    {
      trigger_navigation_failed = true;
    }
  }

  uint8_t status = ActionProcessor::checkSafetyStatus(enable_sensor, resume, trigger_navigation_failed, use_line_sensor);

  switch (status)
  {
  case Feedback::STATUS_NORMAL:
    auto_resume_ = true;
    if (plan_empty)
    {
      ActionStraight::speed_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    }
    // fall through
  case Feedback::STATUS_SAFETY_TRIGGERED:
    if (plan_empty && (std::abs(getHeadingError0()) < config_.getProfile().path_follow_heading_error_max)
                   && (ActionStraight::speed_ < (config_.straight_reaching_min_speed + STOP_THRESHOLD_SPEED)))
    {
      manual_resume_ = false;
      status = Feedback::STATUS_PLAN_EMPTY;
      if (auto_resume_)
      {
        safety_status_ = Feedback::STATUS_PLAN_EMPTY;
      }
    }
    break;
  case Feedback::STATUS_PLAN_EMPTY:
    if (!plan_empty)
    {
      // allow manual resume if auto resume disabled/failed
      status = Feedback::STATUS_SAFETY_TRIGGERED;
    }
    else if (config_.dynplan_retry_time > 0.0f)
    {
      if ((plan_time_ + ros::Duration(config_.dynplan_retry_time)) < ros::Time::now())
      {
        if (manual_resume_)  // allow manual resume
        {
          status = Feedback::STATUS_SAFETY_TRIGGERED;
        }
      }
    }
    break;
  default:
    break;
  }

  return status;
}

void ActionDynamic::running(float frequency)
{
  ActionStraight::alignPosition(frequency, target_distance_ < target_dist_max_, false, alignPositionDone());
}

bool ActionDynamic::alignPositionDone()
{
  const complex pose = poseComplex();
  double tolerance = target_tolerance_ * target_tolerance_;

  if (junction_mid_)
  {
    if (goal_.paths.paths.size())
    {
      for (size_t i = 0; i < goal_.paths.paths.size(); i++)
      {
        if (goal_.paths.paths[i].points.size())
        {
          size_t end = goal_.paths.paths[i].points.size() - 1;
          complex path_end = complex(goal_.paths.paths[i].points[end].x, goal_.paths.paths[i].points[end].y);
          if (std::norm(path_end - pose) < tolerance)
          {
            return true;
          }
        }
      }
    }
    else if (std::norm(path_end_ - pose) < tolerance)
    {
      return true;
    }
  }
  else if (path_heading_ != goal_.path_end.theta && goal_.path_end.theta > 0.0)
  {
    if (std::norm(path_end_ - pose) < (reaching_distance_ * reaching_distance_))
    {
      path_heading_ = goal_.path_end.theta;
    }
  }

  size_t last = plan_.size();
  if (last--)
  {
    if (plan_[last].header.seq == plan_seq_)
    {
      if (junction_mid_)
      {
        ActionDynamic::Point_2 point(pose.real(), pose.imag());
        if (path_area_cur_.is_empty())
        {
          return false;
        }
        if (path_area_cur_.bounded_side(point) != CGAL::ON_UNBOUNDED_SIDE)
        {
          return false;
        }
        if (path_area_goal_.is_empty())
        {
          return true;
        }
        return path_area_goal_.bounded_side(point) != CGAL::ON_UNBOUNDED_SIDE;
      }
      else if ((last == plan_goal_) && (std::norm(path_end_ - pose) < tolerance))
      {
        return ActionStraight::speed_ < (config_.straight_reaching_min_speed + STOP_THRESHOLD_SPEED);
      }
    }
  }

  return false;
}

bool ActionDynamic::handlePoseUpdate()
{
  bool new_plan = base_.getPlan(plan_);
  if (new_plan)
  {
    plan_cur_ = 0;
    if ((plan_.size() > 1) && (path_dist_max_ > 0.0f) && path_area_goal_.is_empty())
    {
      complex points[2] =
      {
        complex(plan_[0].pose.position.x, plan_[0].pose.position.y),
      };
      path_distance_ = 0.0f;
      for (std::size_t i = 1; i < plan_.size(); i++)
      {
        points[i & 1] = complex(plan_[i].pose.position.x, plan_[i].pose.position.y);
        path_distance_ += std::abs(points[1] - points[0]);
      }
      if (path_distance_ > path_dist_max_)
      {
        ROS_DEBUG("Plan overlength (%f > %f)", path_distance_, path_dist_max_);
        plan_.clear();
      }
    }
    if (!plan_.empty())
    {
      plan_time_ = ros::Time::now();
    }
  }

  DigitalInputFilter<int> termination_filter(false, 0, 20);
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

  plan_goal_ = plan_.size() ? plan_.size() - 1 : 0;
  target_squared_distance = std::norm((plan_.size() ? planComplex(plan_goal_) : path_end_) - pose);

  std::vector<complex> pose_goals;
  complex sum_goals;
  float tolerance = config_.dynplan_tolerance_path * config_.dynplan_tolerance_path;
  float squared_distance_max = std::max<float>(target_dist_max_, config_.getProfile().path_follow_ahead_distance);
  squared_distance_max *= squared_distance_max;
  for (size_t i = plan_cur_ + 1; i < plan_.size(); ++i)
  {
    pose_goals.push_back(planComplex(i) - pose);
    sum_goals += pose_goals.back();

    bool update = false;
    float threshold = std::norm(sum_goals) * tolerance;
    for (const auto& pose_goal : pose_goals)
    {
      float r = cross(pose_goal, sum_goals);
      if ((r * r) > threshold)
      {
        update = true;
        break;
      }
    }
    if (update)
    {
      --i;
      pose_goals.pop_back();
    }

    float squared_distance = std::norm(pose_goals.back());
    if (squared_distance > squared_distance_max)
    {
      update = true;
    }

    if (update)
    {
      plan_goal_ = i;
      target_squared_distance = squared_distance;
      break;
    }
  }

  target_distance_ = std::sqrt(target_squared_distance);

  return true;
}

float ActionDynamic::getTargetDistance0()
{
  return target_distance_;
}

float ActionDynamic::getLinearError0()
{
  if ((plan_cur_ + 2) > plan_.size()) return 0.0f;

  const complex v = planComplex(plan_goal_) - planComplex(plan_cur_);
  const complex pose_v = poseComplex() - planComplex(plan_cur_ + 1);
  linear_error_ = cross(v, pose_v) / std::abs(v);  // update protected variable
  return linear_error_;
}

float ActionDynamic::getHeadingError0()
{
  if (plan_.empty())
  {
    // trigger rotate on-spot if path not found
    if ((plan_time_ + ros::Duration(config_.dynplan_search_time)) > ros::Time::now())
    {
      float rotate_angle = config_.getProfile().path_follow_heading_error_max + 0.001f;
      return config_.dynplan_search_direction ? rotate_angle : -rotate_angle;
    }
  }

  const complex v = (plan_.size() ? planComplex(plan_goal_) : path_end_) -
                    ((plan_cur_ + 2) > plan_.size() ? poseComplex() : planComplex(plan_cur_));
  float heading_error = ActionStraight::pose_.theta - std::arg(v);
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

float ActionDynamic::computeOutputLimit(float speed, float heading_error)
{
  float ret = ActionStraight::computeOutputLimit(speed, heading_error);
  if (ActionStraight::state_ == ActionStraight::RUNNING)
  {
    float error = std::abs(heading_error);
    // rotate on-spot
    if (error >= config_.getProfile().path_follow_heading_error_max)
    {
      ret = std::max(ret, std::min(ActionStraight::computeOutputLimit(path_speed_, heading_error),
                                   ActionTurn::reaching_.speed(error)));
    }
  }
  return ret;
}

ActionDynamic::Polygon_2 ActionDynamic::getPathArea(const geometry_msgs::Polygon& path)
{
  ROS_ASSERT(path.points.size() > 1);

  double offset = config_.straight_out_of_path_distance * config_.dynplan_distance_max * 0.01;
  if (target_tolerance_ > 0.0f)
  {
    offset += target_tolerance_;
  }

  std::vector<complex> vertices;
  vertices.push_back(complex(path.points[0].x, path.points[0].y));

  size_t last = path.points.size() - 1;
  complex u, v = complex(path.points[last].x, path.points[last].y) - vertices[0];

  for (size_t i = 1, j = 1; i < path.points.size(); i++)
  {
    u = complex(path.points[i].x, path.points[i].y);
    double c = cross(v, u - vertices[0]);
    if ((c != 0.0) || (i == last))
    {
      vertices.insert(vertices.begin() + j, u);
      if (c > 0.0) j++;
    }
  }

  if (vertices.size() == 2)
  {
    ActionDynamic::Polygon_2 ret;
    double a = std::arg(v) - M_PI * 0.25;
    offset *= std::sqrt(2.0);

    u = vertices[1] + std::polar(offset, a);
    ret.push_back(ActionDynamic::Point_2(u.real(), u.imag()));
    a += M_PI * 0.5;
    u = vertices[1] + std::polar(offset, a);
    ret.push_back(ActionDynamic::Point_2(u.real(), u.imag()));
    a += M_PI * 0.5;
    u = vertices[0] + std::polar(offset, a);
    ret.push_back(ActionDynamic::Point_2(u.real(), u.imag()));
    a += M_PI * 0.5;
    u = vertices[0] + std::polar(offset, a);
    ret.push_back(ActionDynamic::Point_2(u.real(), u.imag()));

    return ret;
  }

  // bug in CGAL 4.11, Boolean Set-Operations only works with Exact_predicates_exact_constructions_kernel
  // https://github.com/CGAL/cgal/issues/5218
  // Polygon Offsetting only works with Exact_predicates_inexact_constructions_kernel
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef CGAL::Polygon_2<K> Polygon_2;
  typedef CGAL::Straight_skeleton_2<K> Ss;

  Polygon_2 poly;

  double min_x = vertices[0].real() - offset;
  double max_x = vertices[0].real() + offset;
  double min_y = vertices[0].imag() - offset;
  double max_y = vertices[0].imag() + offset;

  last = vertices.size() - 1;
  for (size_t i = 0; i < vertices.size(); i++)
  {
    u = vertices[i] - vertices[i ? i - 1 : last];
    v = vertices[i < last ? i + 1 : 0] - vertices[i];
    if (cross(v, u) > 0.0)
    {
      poly.push_back(Point_2(vertices[i].real(), vertices[i].imag()));
      if (i)
      {
        min_x = std::min(min_x, vertices[i].real() - offset);
        max_x = std::max(max_x, vertices[i].real() + offset);
        min_y = std::min(min_y, vertices[i].imag() - offset);
        max_y = std::max(max_y, vertices[i].imag() + offset);
      }
    }
  }

  boost::shared_ptr<Ss> ss = CGAL::create_interior_straight_skeleton_2(poly);
  std::vector<boost::shared_ptr<Polygon_2>> offset_polygons = CGAL::create_offset_polygons_2<Polygon_2>(offset, *ss);
  std::list<CGAL::Polygon_with_holes_2<ActionDynamic::Kernel>> polies, polies_wh;
  CGAL::Polygon_with_holes_2<ActionDynamic::Kernel> polywh;

  poly = *offset_polygons[0];
  if (poly.orientation() == CGAL::CLOCKWISE)
  {
    poly.reverse_orientation();
  }
  for (Polygon_2::Vertex_const_iterator vi = poly.vertices_begin(); vi != poly.vertices_end(); ++vi)
  {
    polywh.outer_boundary().push_back(ActionDynamic::Point_2(vi->x(), vi->y()));
  }
  polies.push_back(polywh);

  polywh = CGAL::Polygon_with_holes_2<ActionDynamic::Kernel>();
  polywh.outer_boundary().push_back(ActionDynamic::Point_2(min_x, min_y));
  polywh.outer_boundary().push_back(ActionDynamic::Point_2(max_x, min_y));
  polywh.outer_boundary().push_back(ActionDynamic::Point_2(max_x, max_y));
  polywh.outer_boundary().push_back(ActionDynamic::Point_2(min_x, max_y));
  polies.push_back(polywh);

  CGAL::intersection(polies.begin(), polies.end(), std::back_inserter(polies_wh));
  return polies_wh.begin()->outer_boundary();
}

uint32_t ActionDynamic::plan_seq_ = 0;

}  // namespace agv05

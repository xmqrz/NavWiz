/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"

#include <queue>

#define BEZIER_LOOKAHEAD_COUNT 100
#define BEZIER_LOOKAHEAD_TIME  (0.5 * BEZIER_LOOKAHEAD_COUNT / LOOP_FREQUENCY)


namespace agv05
{

void ActionBezier::generateBezierSegments()
{
  double segment_distance;
  if (config_.bezier_segment_distance > 0.0)
  {
    segment_distance = config_.bezier_segment_distance;
  }
  else
  {
    const double period = 1.0 / LOOP_FREQUENCY;
    const double speed = std::min<double>(goal_.speed, config_.getProfile().bezier_max_speed);
    // segment distance to be less than half the distance travelled in each cycle
    segment_distance = speed * period / 2;
  }
  const double squared_segment_distance = segment_distance * segment_distance;

  // subdivide bezier curve into segments of equal length.
  const double t_scale = 1e8;
  std::map<int, complex> segments;  // map of t => point
  segments[0] = path_start_;
  segments[t_scale] = path_end_;

  std::queue<int> q;
  q.push(t_scale / 2);

  while (!q.empty() && segments.size() < 100000)
  {
    int t = q.front();
    q.pop();
    complex p = interpolateBezier0(t / t_scale);

    std::map<int, complex>::const_iterator it = segments.lower_bound(t);
    int next_t = it->first;
    complex next_p = it->second;
    --it;
    int prev_t = it->first;
    complex prev_p = it->second;

    if (t - prev_t >= 2 && std::norm(p - prev_p) > squared_segment_distance)
    {
      q.push((t + prev_t) / 2);
    }
    if (next_t - t >= 2 && std::norm(p - next_p) > squared_segment_distance)
    {
      q.push((t + next_t) / 2);
    }
    // insert middle point
    segments[t] = p;
  }

  // Accumulate target distance from the end and populate the path segments.
  size_t i = segments.size();
  bezier_points_.clear();
  bezier_points_.resize(i);
  bezier_target_distances_.clear();
  bezier_target_distances_.resize(i);
  bezier_headings_.clear();
  bezier_headings_.resize(i);

  std::map<int, complex>::const_reverse_iterator it = segments.rbegin(),
                                                 it_end = segments.rend();
  complex p = it->second;
  double target_distance = 0.0;
  float curvature_max = 0.0;
  float curvature_sign = 0.0;
  --i;
  bezier_points_[i] = p;
  bezier_target_distances_[i] = target_distance;
  bezier_headings_[i] = std::norm(path_end_ - path_cp2_) < squared_segment_distance ?
                        std::norm(path_end_ - path_cp1_) < squared_segment_distance ?
                        std::arg(path_end_ - path_start_) :
                        std::arg(path_end_ - path_cp1_) :
                        std::arg(path_end_ - path_cp2_);

  for (++it, --i; it != it_end; ++it, --i)
  {
    complex prime = derivativeBezier0(it->first / t_scale);
    complex prime2 = derivative2Bezier0(it->first / t_scale);
    double curvature = getCurvature(prime, prime2);

    complex next_p = it->second;
    target_distance += std::abs(p - next_p);
    p = next_p;

    bezier_points_[i] = p;
    bezier_target_distances_[i] = target_distance;
    bezier_headings_[i] = std::arg(prime);

    if (!bezier_inflection_)
    {
      if (curvature != 0.0)
      {
        if ((curvature_sign * curvature) < 0.0)
        {
          bezier_inflection_ = i;
        }
        curvature_sign = curvature;
      }
    }

    curvature = std::abs(curvature);
    if (curvature_max < curvature)
    {
      curvature_max = curvature;
    }
  }
  curvature_max *= config_.bezier_curvature_gain;

  pathCurvatureUpdate(curvature_max);
}

void ActionBezier::advanceBezierCursor0()
{
  DigitalInputFilter<int> termination_filter(false, 0, 200);
  const complex pose = complex(pose_.x, pose_.y);

  double target_squared_distance = std::numeric_limits<double>::max();
  for (size_t i = bezier_cur_ + 1; i < bezier_points_.size(); ++i)
  {
    complex pose_v = pose - bezier_points_[i];
    double squared_distance = std::norm(pose_v);

    if (squared_distance <= target_squared_distance)
    {
      target_squared_distance = squared_distance;
      bezier_cur_ = i - 1;
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

  size_t look_ahead_count = BEZIER_LOOKAHEAD_COUNT;
  if (config_.bezier_segment_distance > 0.0)
  {
    float speed, look_ahead_time;
    if (config_.bezier_look_ahead_time > 0.0)
    {
      speed = speed_;
      look_ahead_time = config_.bezier_look_ahead_time;
    }
    else
    {
      speed = std::min<double>(goal_.speed, config_.getProfile().bezier_max_speed);
      look_ahead_time = BEZIER_LOOKAHEAD_TIME;
    }
    look_ahead_count = speed * look_ahead_time / config_.bezier_segment_distance + 1;
  }
  bezier_goal_ = std::min(bezier_cur_ + look_ahead_count, bezier_points_.size() - 1);

  if (bezier_inflection_)
  {
    if (bezier_cur_ > bezier_inflection_)
    {
      bezier_inflection_ = 0;
      resetPathFollowVariable();
    }
  }
}

float ActionBezier::getTargetDistance0()
{
  ROS_ASSERT(bezier_cur_ <= bezier_points_.size() - 2);
  const complex v = bezier_points_[bezier_cur_ + 1] - bezier_points_[bezier_cur_];
  const complex pose_v = complex(pose_.x, pose_.y) - bezier_points_[bezier_cur_ + 1];
  float distance = -dot(v, pose_v) / std::abs(v) + bezier_target_distances_[bezier_cur_ + 1];
  return adjustTargetDistanceByLineSensing0(distance);
}

float ActionBezier::getLinearError0()
{
  ROS_ASSERT(bezier_cur_ <= bezier_points_.size() - 2);
  const complex v = std::polar<double>(1.0, bezier_headings_[bezier_goal_]);
  const complex pose_v = complex(pose_.x, pose_.y) - bezier_points_[bezier_goal_];
  linear_error_ = cross(v, pose_v);
  return linear_error_;
}

float ActionBezier::getHeadingError0()
{
  float heading_error = pose_.theta - bezier_headings_[bezier_goal_];
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

complex ActionBezier::interpolateBezier0(double t)
{
  double t2 = t * t;
  double t3 = t2 * t;
  double inv_t = 1.0 - t;
  double inv_t2 = inv_t * inv_t;
  double inv_t3 = inv_t2 * inv_t;

  return inv_t3 * path_start_ +
         3.0 * inv_t2 * t * path_cp1_ +
         3.0 * inv_t * t2 * path_cp2_ +
         t3 * path_end_;
}

complex ActionBezier::derivativeBezier0(double t)
{
  double t2 = t * t;
  double inv_t = 1.0 - t;
  double inv_t2 = inv_t * inv_t;

  return 3.0 * inv_t2 * (path_cp1_ - path_start_) +
         6.0 * inv_t * t * (path_cp2_ - path_cp1_) +
         3.0 * t2 * (path_end_ - path_cp2_);
}

complex ActionBezier::derivative2Bezier0(double t)
{
  return 6.0 * (t * (path_end_ - 2.0 * path_cp2_ + path_cp1_) +
         (1.0 - t) * (path_cp2_ - 2.0 * path_cp1_ + path_start_));
}

}  // namespace agv05

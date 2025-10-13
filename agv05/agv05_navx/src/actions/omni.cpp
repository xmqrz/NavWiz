/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include "agv05_navx/actions.h"


namespace agv05
{

void ActionOmni::initialize()
{
  if (goal_.nav == Goal::NAV_OMNI)
  {
    pathSpeedUpdate(goal_.path_end.theta, std::abs(path_end_ - path_start_));
    reachingDistanceUpdate(goal_.path_end.theta, path_heading_ > 0.0 ? path_heading_ :
                                                 forward_dir_ ? std::arg(path_end_ - path_start_) :
                                                                std::arg(path_start_ - path_end_));
  }

  speed_y_.setAccelerationLimits(config_.getProfile().straight_normal_acc,
                                 config_.getProfile().straight_normal_dec,
                                 config_.getProfile().straight_jerk_acc);
}

void ActionOmni::pathFollow(float frequency, float speed, float angular_error, float heading_error)
{
  float linear_error = getLinearError0();

  if (frequency <= 0.0f) return;

  // calculate p
  float error_p = heading_error * 0.5f;

  // calculate i
  float error_i = path_follow_error_i_ + (error_p / frequency);  // ∫ e dt

  // calculate d
  float error_d;
  if (config_.getProfile().path_follow_pid_traditional)
  {
    error_d = (error_p - path_follow_error_d_) * frequency;  // de/dt

    // update error
    path_follow_error_d_ = error_p;
  }
  else
  {
    // error = set point - process variable
    // de/dt = d(SP)/dt - d(PV)/dt
    // While proportional and integral terms are driven by the controller error,
    // the derivative computation in many commercial implementations are based on PV,
    // as when the set point changes, derivative on error results in an undesirable
    // control action called 'derivative kick'.
    // d(SP)/dt = 0 -> de/dt = -d(PV)/dt = -dθ/dt
    // Hence, (-dθ/dt) is being used instead of de/dt
    float dtheta = pose_.theta - path_follow_error_d_;  // -2π < Δθ < 2π
    dtheta = fmod(dtheta + 3 * M_PI, 2 * M_PI) - M_PI;  // normalize to -π < Δθ < π
    error_d = dtheta * frequency;  // -ve sign is dropped as output is in opposite sign of error
    path_follow_error_d_ = pose_.theta;
  }

  // calculate output
  float output =
    config_.getProfile().path_follow_p * error_p +
    config_.getProfile().path_follow_i * error_i +
    config_.getProfile().path_follow_d * error_d;

  // saturate output
  float output_limit = computeOutputLimit(speed, heading_error);

  if ((speed == 0.0f) || isCompleted())
  {
    output = 0.0f;
  }
  else if (output > output_limit)
  {
    output = output_limit;
  }
  else if (output < -output_limit)
  {
    output = -output_limit;
  }
  else
  {
    // Anti-reset windup is that the integral term does not accumulate
    // if the controller output is saturated at an upper or lower limit.
    path_follow_error_i_ = error_i;
  }

  float error_lateral = std::abs(linear_error);
  float error_threshold = std::min<float>(reaching_.distance(speed), config_.localization_tolerance_distance);
  float speed_lateral = error_lateral < error_threshold ? 0.0f : std::min(speed, reaching_.speed(error_lateral));

  if (speed < (0.5 * config_.straight_reaching_min_speed))
  {
    // reaccelerate from zero
    speed_y_.setSpeed(0.0f);
  }
  else if (speed_y_ > 0.0f && dir_y_ ^ linear_error < 0.0f)
  {
    speed_y_.update(frequency, 0.0f);
  }
  else
  {
    speed_y_.update(frequency, speed_lateral);
    dir_y_ = linear_error < 0.0f;
  }

  const complex v = std::polar<double>(forward_dir_ ? 1 : -1, -angular_error);
  const complex pose_v(speed, dir_y_ ? speed_y_ : -speed_y_);

  // write output to motor
  base_.setSpeed(dot(v, pose_v), -output, cross(v, pose_v));

  ROS_DEBUG("Speed x %f, Speed y %f, Linear error %f, Angular error %f, Heading error %f, Output %f",
            speed, pose_v.imag(), linear_error, angular_error, heading_error, -output);
}

void ActionOmni::pathSpeedUpdate(float end_heading, float path_distance)
{
  if (end_heading > 0.0f)
  {
    float heading_error = base_.getPose(pose_) ? end_heading - pose_.theta : M_PI;
    heading_error = fmod(heading_error + 3 * M_PI, 2 * M_PI) - M_PI;  // normalize to -π < Δθ < π
    pathCurvatureUpdate(std::abs(heading_error / path_distance));
  }
}

void ActionOmni::reachingDistanceUpdate(float heading_error)
{
  if (config_.getProfile().straight_jerk_dec > 0.0f)
  {
    float t = std::abs(heading_error) / computeOutputLimit(path_speed_, heading_error) -
              (1.0f + config_.getProfile().path_follow_d) / config_.getProfile().path_follow_p;

    if (t > 0)
    {
      reaching_distance_ = std::max(reaching_distance_, reaching_.distance(path_speed_) + path_speed_ * t);
    }
  }
}

void ActionOmni::reachingDistanceUpdate(float end_heading, float path_heading)
{
  if (end_heading > 0.0f)
  {
    float heading_error = end_heading - path_heading;
    heading_error = fmod(heading_error + 3 * M_PI, 2 * M_PI) - M_PI;  // normalize to -π < Δθ < π
    reachingDistanceUpdate(heading_error);
  }
}

float ActionOmni::getTargetDistance0()
{
  if (io_trigger_)
  {
    geometry_msgs::Pose2D pose;
    if (base_.Base::getPose(pose))
    {
      return goal_.next_distance - std::hypot(pose.x - io_trigger_pose_.x, pose.y - io_trigger_pose_.y);
    }
  }
  return ActionStraight::getTargetDistance0();
}

float ActionOmni::getHeadingError0(float heading_error)
{
  angular_error_ = -heading_error;  // update protected variable

  if (path_heading_ > 0.0)  // 0 < Δθ < 2π
  {
    heading_error = pose_.theta - path_heading_;  // -3π < Δθ < π
    if (heading_error < -M_PI) heading_error += 2 * M_PI;  // normalize to -π < Δθ < π
    heading_error_ = heading_error;  // update protected variable
  }

  return heading_error;
}

}  // namespace agv05

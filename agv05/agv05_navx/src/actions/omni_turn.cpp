/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include "agv05_navx/actions.h"

#define STOP_THRESHOLD_SPEED 0.001f


namespace agv05
{

void ActionTurnOmni::initialize()
{
  geometry_msgs::Twist cmd_vel;
  if (base_.getMotorCmdVel(cmd_vel, true))
  {
    speed_ratio_ = cmd_vel.linear.x / cmd_vel.angular.z;
    speed_x_max_ = std::min<float>(speed_x_max_, cmd_vel.linear.x);
    speed_x_max_ = std::min<float>(speed_x_max_, config_.straight_reaching_speed);
    speed_x_max_ = std::min<float>(speed_x_max_, config_.turn_reaching_speed * speed_ratio_);
    speed_x_max_ = std::max<float>(speed_x_max_, config_.straight_reaching_min_speed);
    speed_y_max_ = cmd_vel.linear.y > 0 ? speed_x_max_ : 0.0f;
    speed_z_max_ = std::max<float>(speed_x_max_ / speed_ratio_, config_.turn_reaching_speed);
  }
  else
  {
    completeNav(Result::RESULT_FAIL);
  }
}

void ActionTurnOmni::processAlign(float frequency)
{
  if (isCompleted()) return;

  bool enable_sensor = goal_.enable_sensor;

  // safety check
  bool trigger_navigation_failed = !handlePoseUpdate();
  uint8_t status = checkSafetyStatus(enable_sensor, false, trigger_navigation_failed);
  safety_.publishNavTrigger(status != Feedback::STATUS_NORMAL, safety_internal_message_);

  // set speed limits
  speed_x_.setAccelerationLimits(config_.getProfile().straight_normal_acc,
                                 config_.getProfile().straight_normal_dec,
                                 config_.getProfile().straight_jerk_acc,
                                 config_.getProfile().straight_jerk_dec);
  speed_y_.setAccelerationLimits(config_.getProfile().straight_normal_acc,
                                 config_.getProfile().straight_normal_dec,
                                 config_.getProfile().straight_jerk_acc,
                                 config_.getProfile().straight_jerk_dec);
  speed_z_.setAccelerationLimits(config_.getProfile().turn_normal_acc,
                                 config_.getProfile().turn_normal_dec,
                                 config_.getProfile().turn_jerk_acc,
                                 config_.getProfile().turn_jerk_dec);

  speed_x_.clearSpeedLimit();
  speed_y_.clearSpeedLimit();
  speed_z_.clearSpeedLimit();

  speed_x_.addSpeedLimit(speed_x_max_);
  speed_y_.addSpeedLimit(speed_y_max_);
  speed_z_.addSpeedLimit(speed_z_max_);

  if (status != Feedback::STATUS_NORMAL)
  {
    speed_x_.addSpeedLimit(0.0f);
    speed_y_.addSpeedLimit(0.0f);
    speed_z_.addSpeedLimit(0.0f);
    ActionTurn::position_align_filter_.resetStalled();
  }
  else if (isPaused())
  {
    speed_x_.addSpeedLimit(0.0f);
    speed_y_.addSpeedLimit(0.0f);
    speed_z_.addSpeedLimit(0.0f);
    status = Feedback::STATUS_PAUSED;
    ActionTurn::position_align_filter_.resetStalled();
  }
  else if (!base_.isNavigationEnabled())
  {
    // No safety triggered but hardware is not ready. Use small speed to wake up hardware.
    speed_x_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    speed_y_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    speed_z_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    ActionTurn::position_align_filter_.resetStalled();
  }
  else if (!base_.isSteeringAligned())
  {
    // reaccelerate from zero to publish a small value as hint to motor
    speed_x_.setSpeed(0.0f);
    speed_y_.setSpeed(0.0f);
    speed_z_.setSpeed(0.0f);
    ActionTurn::position_align_filter_.resetStalled();
  }
  else if (laser_sensor_.getActivation().near_blocked)
  {
    speed_x_.addSpeedLimit(config_.straight_near_block_speed);
    speed_y_.addSpeedLimit(config_.straight_near_block_speed);
    speed_z_.addSpeedLimit(config_.turn_near_block_speed);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }
  else if (laser_sensor_.getActivation().middle_blocked)
  {
    speed_x_.addSpeedLimit(config_.straight_middle_block_speed);
    speed_y_.addSpeedLimit(config_.straight_middle_block_speed);
    speed_z_.addSpeedLimit(config_.straight_middle_block_speed / speed_ratio_);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }
  else if (laser_sensor_.getActivation().far_blocked)
  {
    speed_x_.addSpeedLimit(config_.straight_far_block_speed);
    speed_y_.addSpeedLimit(config_.straight_far_block_speed);
    speed_z_.addSpeedLimit(config_.straight_far_block_speed / speed_ratio_);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }

  // publish status
  publishStatus(status);

  // state machine
  switch (ActionStraight::state_)
  {
  case ActionStraight::PLANNING:
    planning(frequency);
    break;

  case ActionStraight::RUNNING:
  case ActionStraight::ALIGN_POSITION:
    alignPosition(frequency);
    break;

  case ActionStraight::STOP_EXIT:
  case ActionStraight::STOP_EXIT_REACHING:
  case ActionStraight::STOP_EXIT_IO_DETECTED:
  case ActionStraight::STOP_EXIT_ERROR_TRIGGERED:
    stopExit(frequency);
    break;

  default:
    ROS_ASSERT_MSG(0, "Unknown state.");
    break;
  }
}

void ActionTurnOmni::alignPosition(float frequency)
{
  // from NED to NWU
  float error_x = getTargetDistance0();
  float error_y = -getLinearError0();
  float error_z = -getHeadingError0();

  float center_error = std::max(config_.undershoot_stopping_distance,
                                config_.turn_alignment_center_error * speed_ratio_);
  center_error = std::max(center_error, goal_.goal_tolerance);

  bool aligned = (std::abs(error_x) < center_error);
  float error_a = aligned ? 0.0f : -getAngularError0();
  if (speed_y_max_ > 0.0f)
  {
    aligned &= (std::abs(error_y) < center_error);
  }
  aligned &= (std::abs(error_z) < config_.turn_alignment_center_error);
  ActionTurn::position_align_filter_.update(aligned, 1.0f / frequency);
  if (aligned && config_.straight_alignment_max_time > config_.turn_alignment_center_min_time)
  {
    ActionStraight::position_align_filter_.update(aligned, 1.0f / frequency);
  }

  if (isAborted())
  {
    ActionStraight::state_ = ActionStraight::STOP_EXIT;
  }
  else if (ActionStraight::position_align_filter_.getState())  // align time out
  {
    ActionStraight::state_ = ActionStraight::STOP_EXIT;
  }
  else if (ActionTurn::position_align_filter_.getState())
  {
    ActionStraight::state_ = ActionStraight::STOP_EXIT;
  }

  if (config_.getProfile().straight_jerk_dec > 0.0f)
  {
    omniAlign(frequency, error_x, speed_x_max_, speed_x_, dir_x_, ActionStraight::reaching_, center_error);
    if (speed_y_max_ > 0.0f)
    {
      omniAlign(frequency, error_y, speed_y_max_, speed_y_, dir_y_, ActionStraight::reaching_, center_error);
    }
  }
  else
  {
    omniAlign(frequency, error_x, speed_x_max_, speed_x_, dir_x_);
    if (speed_y_max_ > 0.0f)
    {
      omniAlign(frequency, error_y, speed_y_max_, speed_y_, dir_y_);
    }
  }

  if (config_.getProfile().turn_jerk_dec > 0.0f)
  {
    omniAlign(frequency, error_z + error_a, speed_z_max_, speed_z_, dir_z_,
              ActionTurn::reaching_, config_.turn_alignment_center_error);
  }
  else
  {
    omniAlign(frequency, error_z + error_a, speed_z_max_, speed_z_, dir_z_);
  }

  moving0();
}

void ActionTurnOmni::stopExit(float frequency)
{
  bool complete = true;
  if (speed_x_ < STOP_THRESHOLD_SPEED)
  {
    speed_x_.setSpeed(0.0f);
  }
  else
  {
    complete = false;
  }
  if (speed_y_ < STOP_THRESHOLD_SPEED)
  {
    speed_y_.setSpeed(0.0f);
  }
  else
  {
    complete = false;
  }
  if (speed_z_ < STOP_THRESHOLD_SPEED)
  {
    speed_z_.setSpeed(0.0f);
  }
  else
  {
    complete = false;
  }

  if (complete)
  {
    base_.stop();
    completeNav(Result::RESULT_SUCCESS);
  }
  else
  {
    speed_x_.update(frequency, 0.0f);
    speed_y_.update(frequency, 0.0f);
    speed_z_.update(frequency, 0.0f);
    moving0();
  }
}

void ActionTurnOmni::omniAlign(float frequency, float error, float speed_max, VelocitySmoother<float>& speed, bool& dir,
                               ReachingKinematic<float>& reaching, float center_error)
{
  float target_distance = dir ? error : -error;
  if (target_distance < -center_error)  // overshot
  {
    if (speed < STOP_THRESHOLD_SPEED)
    {
      speed.setSpeed(0.0f);
      target_distance = -target_distance;
      dir = !dir;
    }
    ActionTurn::position_align_filter_.resetStalled();
  }
  else if (target_distance < center_error)  // stopping zone
  {
    target_distance = 0.0f;
  }
  else
  {
    target_distance -= center_error;
  }

  float reaching_speed = std::max<float>(speed, speed_max);
  reaching_speed = std::min(reaching_speed, reaching.speed(target_distance));
  speed.update(frequency, reaching_speed);
}

void ActionTurnOmni::omniAlign(float frequency, float error, float speed_max, VelocitySmoother<float>& speed, bool& dir)
{
  if (frequency <= 0.0f) return;

  // calculate output
  float output = config_.turn_alignment_gain * error;

  // limit max output
  if (output > speed_max) output = speed_max;
  else if (output < -speed_max) output = -speed_max;

  // write output to motor
  speed.setSpeed(std::abs(output));
  dir = (output >= 0.0f);
}

void ActionTurnOmni::moving0()
{
  base_.setSpeed(dir_x_ ? speed_x_ : -speed_x_, dir_z_ ? speed_z_ : -speed_z_,
                 speed_y_max_ > 0.0f ? dir_y_ ? speed_y_ : -speed_y_ : 0.0f);
}

bool ActionTurnOmni::handlePoseUpdate()
{
  if (!getPose())
  {
    return ActionStraight::state_ == ActionStraight::PLANNING;
  }

  // update protected variable
  overshoot_ = ActionStraight::pose_.x;
  linear_error_ = ActionStraight::pose_.y;
  heading_error_ = ActionStraight::pose_.theta;

  if (speed_y_max_ > 0.0f)
  {
    angular_error_ = 0.0f;
  }
  else
  {
    float linear_error = linear_error_ < 0.0f ? std::max(linear_error_, -std::abs(overshoot_)) :
                                                std::min(linear_error_, std::abs(overshoot_));
    angular_error_ = ActionStraight::getAngularError0(overshoot_ > 0.0f ? -linear_error : linear_error);
  }

  return true;
}

}  // namespace agv05

/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"

#define STOP_THRESHOLD_SPEED 0.001f
#define ALIGN_TOLERANCE      0.15f


namespace agv05
{

void ActionTurn::initialize()
{
  uint8_t nav = goal_.nav;
  if (nav == Goal::NAV_ROTATE_LEFT)
  {
    left_dir_ = true;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_ROTATE_LEFT);
  }
  else if (nav == Goal::NAV_ROTATE_RIGHT)
  {
    left_dir_ = false;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_ROTATE_RIGHT);
  }
  else if (nav == Goal::NAV_OMNI_DOCK)
  {
    left_dir_ = true;
  }
}

void ActionTurn::process(float frequency)
{
  if (isCompleted()) return;

  if (isPausedManualControl())
  {
    return ActionManualControl::process(frequency);
  }

  bool enable_sensor = goal_.enable_sensor;

  // safety check
  bool trigger_navigation_failed = !base_.getPose(pose_);
  uint8_t status = checkSafetyStatus(enable_sensor, false, trigger_navigation_failed);
  safety_.publishNavTrigger(status != Feedback::STATUS_NORMAL, safety_internal_message_);

  // set speed limits
  speed_.setAccelerationLimits(config_.getProfile().turn_normal_acc,
                               config_.getProfile().turn_normal_dec,
                               config_.getProfile().turn_jerk_acc,
                               config_.getProfile().turn_jerk_dec);
  speed_.clearSpeedLimit();
  speed_.addSpeedLimit(config_.turn_normal_speed);
  speed_.addSpeedLimit(config_.getProfile().turn_max_speed);

  if (status != Feedback::STATUS_NORMAL)
  {
    speed_.addSpeedLimit(0.0f);
    position_align_filter_.resetStalled();
  }
  else if (isPaused())
  {
    speed_.addSpeedLimit(0.0f);
    status = Feedback::STATUS_PAUSED;
    position_align_filter_.resetStalled();
  }
  else if (!base_.isNavigationEnabled())
  {
    // No safety triggered but hardware is not ready. Use small speed to wake up hardware.
    speed_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    position_align_filter_.resetStalled();
  }
  else if (!base_.isSteeringAligned())
  {
    // reaccelerate from zero to publish a small value as hint to motor
    speed_.setSpeed(0.0f);
    position_align_filter_.resetStalled();
  }
  else if (laser_sensor_.getActivation().near_blocked)
  {
    speed_.addSpeedLimit(config_.turn_near_block_speed);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }
  else if (laser_sensor_.getActivation().middle_blocked)
  {
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }

  // publish status
  publishStatus(status, left_dir_ ? 1 : 2);

  // state machine
  switch (state_)
  {
  case TURNING:
    turning(frequency);
    break;

  case ALIGN_POSITION:
    alignPosition(frequency);
    break;

  case STOP_EXIT:
    stopExit(frequency);
    break;

  default:
    ROS_ASSERT_MSG(0, "Unknown state.");
    break;
  }
}

void ActionTurn::turning(float frequency)
{
  float angular_error = getAngularError0();
  speed_.update(frequency);
  turning0(speed_);

  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  else
  {
    float reaching_distance = config_.turn_reaching_distance;
    if (config_.getProfile().turn_jerk_dec > 0.0f)
    {
      reaching_distance = std::max(reaching_distance, reaching_.distance(speed_, speed_.getAccel()));
    }

    float error_abs = fabs(angular_error);
    if (error_abs > (M_PI * 2 - ALIGN_TOLERANCE))  // maximum rotate angle from skills is 350 deg
    {
      state_ = ALIGN_POSITION;
    }
    else if (error_abs < reaching_distance + ALIGN_TOLERANCE)
    {
      if (left_dir_)
      {
        if (angular_error > -reaching_distance)
        {
          state_ = ALIGN_POSITION;
        }
      }
      else
      {
        if (angular_error < reaching_distance)
        {
          state_ = ALIGN_POSITION;
        }
      }
    }
  }
}

void ActionTurn::alignPosition(float frequency)
{
  float angular_error = getAngularError0();

  position_align_filter_.update(fabs(angular_error) < config_.turn_alignment_center_error, 1.0f / frequency);

  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  else if (position_align_filter_.isStalled())  // align time out
  {
    state_ = STOP_EXIT;
  }
  else if (position_align_filter_.getState())
  {
    state_ = STOP_EXIT;
  }

  if (state_ == STOP_EXIT)
  {
    if ((config_.getProfile().turn_jerk_dec > 0.0f) || isAborted())
    {
      stopExit(frequency);
    }
    else
    {
      turningAlign(frequency, angular_error, config_.turn_reaching_speed);
      speed_.setSpeed(0.0f);
    }
  }
  else
  {
    if (config_.getProfile().turn_jerk_dec > 0.0f)
    {
      float target_distance = left_dir_ ? -angular_error : angular_error;
      if (target_distance < -config_.turn_alignment_center_error)  // overshot
      {
        if (speed_ < STOP_THRESHOLD_SPEED)
        {
          speed_.setSpeed(0.0f);
          target_distance = -target_distance;
          left_dir_ ^= true;
        }
        position_align_filter_.resetStalled();
      }
      else if (target_distance < config_.turn_alignment_center_error)  // stopping zone
      {
        target_distance = 0.0f;
      }
      else
      {
        target_distance -= config_.turn_alignment_center_error;
      }

      float reaching_speed = std::max<float>(speed_, config_.turn_reaching_speed);
      reaching_speed = std::min(reaching_speed, reaching_.speed(target_distance));
      speed_.update(frequency, reaching_speed);
      turning0(speed_);
    }
    else
    {
      turningAlign(frequency, angular_error, config_.turn_reaching_speed);
    }
  }
}

void ActionTurn::stopExit(float frequency)
{
  if (speed_ < STOP_THRESHOLD_SPEED)
  {
    speed_.setSpeed(0.0f);
    base_.stop();
    completeNav(Result::RESULT_SUCCESS);
  }
  else
  {
    speed_.update(frequency, 0.0f);
    turning0(speed_);
  }
}

void ActionTurn::turningAlign(float frequency, float angular_error, float max_angular_speed)
{
  if (frequency <= 0.0f) return;

  // calculate p
  float error_p = angular_error;

  // calculate output
  float output = config_.turn_alignment_gain * error_p;

  // limit max output
  if (output > max_angular_speed) output = max_angular_speed;
  else if (output < -max_angular_speed) output = -max_angular_speed;

  ROS_DEBUG("max angular speed %f, error %f, output %f", max_angular_speed, angular_error, -output);

  // write output to motor
  base_.setSpeed(0.0f, -output);
}

float ActionTurn::getAngularError0()
{
  // heading error
  float angular_error = pose_.theta - goal_.path_end.theta;

  // normalize
  angular_error = fmod(angular_error, 2 * M_PI);
  if (state_ == TURNING)
  {
    if (left_dir_)
    {
      if (angular_error > 0.0f)
      {
        angular_error -= 2 * M_PI;
      }
    }
    else if (angular_error < 0.0f)
    {
      angular_error += 2 * M_PI;
    }
  }
  else if (angular_error > M_PI)
  {
    angular_error -= 2 * M_PI;
  }
  else if (angular_error <= -M_PI)
  {
    angular_error += 2 * M_PI;
  }
  angular_error_ = angular_error;  // update protected variable
  return angular_error;
}

void ActionTurn::turning0(float speed)
{
  if (left_dir_)
  {
    base_.turnLeft(speed);
  }
  else
  {
    base_.turnRight(speed);
  }
}

}  // namespace agv05

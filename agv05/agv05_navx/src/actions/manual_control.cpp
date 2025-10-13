/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"

#define STOP_THRESHOLD_SPEED 0.001f


namespace agv05
{

ActionManualControl::ActionManualControl(Nav& nav, const Goal& goal) :
  ActionProcessor(nav, goal),
  area_(agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL)
{
  base_.resetManualCmdVel();
}

void ActionManualControl::process(float frequency)
{
  if (isCompleted()) return;

  if (laser_sensor_.getArea() != agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL)
  {
    area_ = laser_sensor_.getArea();
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL);
  }

  // Hack: Use the smallest positive denormal to signal no obstacle sensing
  geometry_msgs::Twist v = base_.getManualCmdVel();
  bool enable_sensor = *reinterpret_cast<uint64_t*>(&v.linear.z) != 1LL || base_.getManualCmdVelTimeout() >= 0.5;

  safety_.publishMuteBumper(!enable_sensor);

  // safety check
  uint8_t status = ActionProcessor::checkSafetyStatus(enable_sensor, true);
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
  speed_z_.setAccelerationLimits(config_.getProfile().straight_normal_acc,
                                 config_.getProfile().straight_normal_dec,
                                 config_.getProfile().straight_jerk_acc,
                                 config_.getProfile().straight_jerk_dec);
  speed_x_.clearSpeedLimit();
  speed_y_.clearSpeedLimit();
  speed_z_.clearSpeedLimit();

  if (status != Feedback::STATUS_NORMAL)
  {
    speed_x_.setSpeed(0.0f);
    speed_y_.setSpeed(0.0f);
    speed_z_.setSpeed(0.0f);
    speed_x_.addSpeedLimit(0.0f);
    speed_y_.addSpeedLimit(0.0f);
    speed_z_.addSpeedLimit(0.0f);
  }
  else if (!base_.isNavigationEnabled())
  {
    // No safety triggered but hardware is not ready. Use small speed to wake up hardware.
    speed_x_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    speed_y_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    speed_z_.addSpeedLimit(STOP_THRESHOLD_SPEED);
  }
  else if (!base_.isSteeringAligned())
  {
    // reaccelerate from zero to publish a small value as hint to motor
    speed_x_.setSpeed(0.0f);
    speed_y_.setSpeed(0.0f);
    speed_z_.setSpeed(0.0f);
  }
  else if (enable_sensor)
  {
    if (laser_sensor_.getActivation().middle_blocked)
    {
      speed_x_.addSpeedLimit(config_.straight_middle_block_speed);
      speed_y_.addSpeedLimit(config_.straight_middle_block_speed);
      speed_z_.addSpeedLimit(config_.straight_middle_block_speed * 2.0f);
      status = Feedback::STATUS_OBSTACLE_IN_RANGE;
    }
    else if (laser_sensor_.getActivation().far_blocked)
    {
      speed_x_.addSpeedLimit(config_.straight_far_block_speed);
      speed_y_.addSpeedLimit(config_.straight_far_block_speed);
      speed_z_.addSpeedLimit(config_.straight_far_block_speed * 2.0f);
      status = Feedback::STATUS_OBSTACLE_IN_RANGE;
    }
  }

  // publish status
  publishStatus(status);

  // apply speed
  if (!isAborted() && base_.getManualCmdVelTimeout() < 0.5)
  {
    base_.incrementManualCmdVelTimeout(1.0 / frequency);

    speed_x_.update(frequency, v.linear.x, STOP_THRESHOLD_SPEED);
    speed_y_.update(frequency, v.linear.y, STOP_THRESHOLD_SPEED);
    if (v.linear.x)
    {
      speed_z_.addSpeedLimit(std::abs(speed_x_ * v.angular.z / v.linear.x));
    }
    speed_z_.update(frequency, v.angular.z, STOP_THRESHOLD_SPEED);
  }
  else
  {
    bool stop = speed_x_.update(frequency, 0.0f, STOP_THRESHOLD_SPEED);
    stop &= speed_y_.update(frequency, 0.0f, STOP_THRESHOLD_SPEED);
    stop &= speed_z_.update(frequency, 0.0f, STOP_THRESHOLD_SPEED);

    if (stop)
    {
      speed_x_.setSpeed(0.0f);
      speed_y_.setSpeed(0.0f);
      speed_z_.setSpeed(0.0f);
      base_.stop();
      if (isAborted() || !isPaused())
      {
        safety_.publishMuteBumper(false);
        laser_sensor_.selectArea(area_);
      }
      return;
    }
  }
  base_.setSpeed(speed_x_, speed_z_, speed_y_);
}

}  // namespace agv05

/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/actions.h"

#define STOP_THRESHOLD_SPEED 0.001f


namespace agv05
{

void ActionTurn::initialize()
{
  uint8_t nav = goal_.nav;
  if (nav == Goal::NAV_ROTATE_LEFT || nav == Goal::NAV_UTURN_LEFT ||
      nav == Goal::NAV_ROTATE3Q_LEFT || nav == Goal::NAV_SEARCH_LINE_LEFT)
  {
    left_dir_ = true;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_ROTATE_LEFT);
  }
  else if (nav == Goal::NAV_ROTATE_RIGHT || nav == Goal::NAV_UTURN_RIGHT ||
           nav == Goal::NAV_ROTATE3Q_RIGHT || nav == Goal::NAV_SEARCH_LINE_RIGHT)
  {
    left_dir_ = false;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_ROTATE_RIGHT);
  }

  if (line_follow_type_ == Goal::LINE_FOLLOW_ODOM_2D)
  {
    float turn_distance = std::abs(goal_.distance);
    turn_distance_ = left_dir_ ? turn_distance : -turn_distance;
  }
  else
  {
    if (nav == Goal::NAV_ROTATE_LEFT || nav == Goal::NAV_ROTATE_RIGHT)
    {
      turn_distance_ = M_PI * 0.5;
    }
    else if (nav == Goal::NAV_UTURN_LEFT || nav == Goal::NAV_UTURN_RIGHT)
    {
      turn_distance_ = M_PI;
    }
    else if (nav == Goal::NAV_ROTATE3Q_LEFT || nav == Goal::NAV_ROTATE3Q_RIGHT)
    {
      turn_distance_ = M_PI * 1.5;
    }
    else
    {
      turn_distance_ = std::abs(goal_.distance);
    }
    turn_distance_ -= config_.turn_search_distance;
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
  agv05_msgs::LineSensor lsd = getLineSensorData0();
  uint8_t status = checkSafetyStatus(enable_sensor, false, false, getLineSensor());
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
    line_align_filter_.resetStalled();
  }
  else if (isPaused())
  {
    speed_.addSpeedLimit(0.0f);
    status = Feedback::STATUS_PAUSED;
    line_align_filter_.resetStalled();
  }
  else if (!base_.isNavigationEnabled())
  {
    // No safety triggered but hardware is not ready. Use small speed to wake up hardware.
    speed_.addSpeedLimit(STOP_THRESHOLD_SPEED);
    line_align_filter_.resetStalled();
  }
  else if (!base_.isSteeringAligned())
  {
    // reaccelerate from zero to publish a small value as hint to motor
    speed_.setSpeed(0.0f);
    line_align_filter_.resetStalled();
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

  case SEARCH_LINE:
    searchLine(frequency);
    break;

  case ALIGN_LINE:
    alignLine(frequency);
    break;

  case STOP_EXIT:
  case STOP_EXIT_NO_LINE:
    stopExit(frequency);
    break;

  default:
    ROS_ASSERT_MSG(0, "Unknown state.");
    break;
  }
}

void ActionTurn::turning(float frequency)
{
  speed_.update(frequency);
  turning0(speed_);

  // task aborted
  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  else if (line_follow_type_ == Goal::LINE_FOLLOW_ODOM_2D)
  {
    float angular_error = base_.getRotationalDistance() - initial_rd_ - turn_distance_;
    float target_distance = left_dir_ ? -angular_error : angular_error;
    float reaching_distance = reaching_.distance(speed_, speed_.getAccel());

    if (target_distance < reaching_distance)
    {
      state_ = ALIGN_LINE;
    }
  }
  else if (fabs(base_.getRotationalDistance() - initial_rd_) > turn_distance_)
  {
    state_ = SEARCH_LINE;
  }
}

void ActionTurn::searchLine(float frequency)
{
  agv05_msgs::LineSensor lsd = getLineSensorData0();
  line_found_filter_.update(!lsd.out_of_line, 1.0f / frequency);

  if (!lsd.out_of_line && (config_.getProfile().turn_jerk_dec > 0.0f))
  {
    float target_distance = left_dir_ ? -lsd.angular_error : lsd.angular_error;
    float reaching_speed = std::max<float>(speed_, config_.turn_search_line_speed);
    target_distance -= config_.turn_alignment_center_error;
    reaching_speed = std::min(reaching_speed, reaching_.speed(target_distance));
    speed_.update(frequency, reaching_speed);
  }
  else
  {
    // limit targeted speed to search line speed
    speed_.update(frequency, config_.turn_search_line_speed);
  }
  turning0(speed_);

  // task aborted
  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  // line found
  else if (line_found_filter_.getState())
  {
    state_ = ALIGN_LINE;
  }
  // search distance up
  else if (fabs(base_.getRotationalDistance() - initial_rd_) > (turn_distance_ + config_.turn_search_distance * 2.0))
  {
    state_ = STOP_EXIT_NO_LINE;
  }
}

void ActionTurn::alignLine(float frequency)
{
  // align line
  float angular_error = line_follow_type_ == Goal::LINE_FOLLOW_ODOM_2D ?
                        base_.getRotationalDistance() - initial_rd_ - turn_distance_ :
                        getLineSensorData0().angular_error;

  line_align_filter_.update(fabs(angular_error) < config_.turn_alignment_center_error, 1.0f / frequency);

  // task aborted
  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  // align time out
  else if (line_align_filter_.isStalled())
  {
    state_ = STOP_EXIT;
  }
  else if (line_align_filter_.getState())
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
      turningAlign(frequency, angular_error, config_.turn_search_line_speed);
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
        line_align_filter_.resetStalled();
      }
      else if (target_distance < config_.turn_alignment_center_error)  // stopping zone
      {
        target_distance = 0.0f;
      }
      else
      {
        target_distance -= config_.turn_alignment_center_error;
      }

      float reaching_speed = std::max<float>(speed_, config_.turn_search_line_speed);
      reaching_speed = std::min(reaching_speed, reaching_.speed(target_distance));
      speed_.update(frequency, reaching_speed);
      turning0(speed_);
    }
    else
    {
      turningAlign(frequency, angular_error, config_.turn_search_line_speed);
    }
  }
}

void ActionTurn::stopExit(float frequency)
{
  if (speed_ < STOP_THRESHOLD_SPEED)
  {
    speed_.setSpeed(0.0f);
    base_.stop();
    if (isAborted() || line_follow_type_ == Goal::LINE_FOLLOW_ODOM_2D)
    {
      completeNav(Result::RESULT_SUCCESS);
    }
    else
    {
      completeNav((state_ == STOP_EXIT_NO_LINE || getLineSensorData0().out_of_line) ?
                  Result::RESULT_FAIL : Result::RESULT_SUCCESS);
    }
  }
  else
  {
    speed_.update(frequency, 0.0f);
    turning0(speed_);
  }
}

void ActionTurn::turningAlign(float frequency, float line_angular_error, float max_angular_speed)
{
  if (frequency <= 0.0f) return;

  // calculate p
  float error_p = line_angular_error;

  // calculate output
  float output = config_.turn_alignment_gain * error_p;

  // limit max output
  if (output > max_angular_speed) output = max_angular_speed;
  else if (output < -max_angular_speed) output = -max_angular_speed;

  ROS_DEBUG("max angular speed %f, error %f, output %f", max_angular_speed, line_angular_error, -output);

  // write output to motor
  base_.setSpeed(0.0f, -output);
}

ActionProcessor::UseLineSensor ActionTurn::getLineSensor()
{
  return line_follow_type_ == Goal::LINE_FOLLOW_ODOM_2D ? LINE_SENSOR_DISABLE :
         goal_.rotate_align_sensor == Goal::ALIGN_SENSOR_FRONT ? LINE_SENSOR_FRONT : LINE_SENSOR_REAR;
}

agv05_msgs::LineSensor ActionTurn::getLineSensorData0()
{
  if (goal_.rotate_align_sensor == Goal::ALIGN_SENSOR_FRONT)
  {
    return line_sensor_.getFrontData();
  }
  else
  {
    return line_sensor_.getRearData();
  }
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

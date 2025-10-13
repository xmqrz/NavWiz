/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/actions.h"


namespace agv05
{

void ActionLineCalibrate::initialize()
{
  ActionTurn::state_ = ActionTurn::TURNING;
  initial_rd_ = base_.getRotationalDistance();
  line_found_filter_.setState(false);
  line_align_filter_.setState(false);

  switch (state_)
  {
  case TURN_LEFT_1:
    line_follow_type_ = Goal::LINE_FOLLOW_ODOM_2D;
    left_dir_ = true;
    turn_distance_ = config_.calibrate_turn_short_distance;
    break;
  case TURN_RIGHT_2:
    line_sensor_.activateCalibration(true);
    line_follow_type_ = Goal::LINE_FOLLOW_ODOM_2D;
    left_dir_ = false;
    turn_distance_ = -config_.calibrate_turn_long_distance;
    break;
  case CHECK_CALIBRATED_DATA:
    line_sensor_.activateCalibration(false);
    initial_cal_time_ = ros::Time::now().toSec();
    break;
  case TURN_LEFT_3:
    line_follow_type_ = Goal::LINE_FOLLOW_TRACK;
    left_dir_ = (final_rd_ > initial_rd_);
    turn_distance_ = std::abs(final_rd_ - initial_rd_) - config_.turn_search_distance;
    break;
  default:
    break;
  }
}

void ActionLineCalibrate::process(float frequency)
{
  if (isCompleted()) return;

  // state machine
  if (state_ == CHECK_CALIBRATED_DATA)
  {
    checkCalibratedData();
  }
  else
  {
    ActionTurn::process(frequency);
  }
}

void ActionLineCalibrate::checkCalibratedData()
{
  // wait for 1.5sec before check calibration data
  if ((ros::Time::now().toSec() - initial_cal_time_) > 1.5)
  {
    completeNav((line_sensor_.getFrontData().sensor_error || line_sensor_.getRearData().sensor_error) ?
                Result::RESULT_FAIL : Result::RESULT_SUCCESS);
  }
}

void ActionLineCalibrate::completeNav(uint8_t result)
{
  if ((state_ >= TURN_LEFT_3) || (result != Result::RESULT_SUCCESS))
  {
    ActionProcessor::completeNav(result);
    return;
  }

  state_ = static_cast<ActionLineCalibrate::State>(state_ + 1);
  initialize();
}

uint8_t ActionLineCalibrate::checkSafetyStatus(bool enable_sensor, bool resume, bool trigger_out_of_line, UseLineSensor use_line_sensor)
{
  return ActionProcessor::checkSafetyStatus(false, true);
}

}  // namespace agv05

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

void ActionStraight::initialize()
{
  resetLineFollowVariable();

  uint8_t nav = goal_.nav;
  float distance = goal_.distance;
  uint8_t io_trigger_port = goal_.io_trigger_port;
  uint8_t io_trigger_pin = goal_.io_trigger_pin;
  uint8_t error_io_trigger_port = goal_.error_io_trigger_port;
  uint8_t error_io_trigger_pin = goal_.error_io_trigger_pin;
  uint8_t next_motion = goal_.next_motion;

  if (nav == Goal::NAV_FORWARD || nav == Goal::NAV_BEZIER_FORWARD ||
      nav == Goal::NAV_FORWARD_LEFT || nav == Goal::NAV_BEZIER_FORWARD_LEFT ||
      nav == Goal::NAV_FORWARD_RIGHT || nav == Goal::NAV_BEZIER_FORWARD_RIGHT)
  {
    forward_dir_ = true;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_FORWARD);
  }
  else if (nav == Goal::NAV_REVERSE || nav == Goal::NAV_BEZIER_REVERSE ||
           nav == Goal::NAV_REVERSE_LEFT || nav == Goal::NAV_BEZIER_REVERSE_LEFT ||
           nav == Goal::NAV_REVERSE_RIGHT || nav == Goal::NAV_BEZIER_REVERSE_RIGHT)
  {
    forward_dir_ = false;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_REVERSE);
  }
  else if (nav == Goal::NAV_LINE_TUNE_PID)
  {
    forward_dir_ = (distance > 0.0f);
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DISABLE);

    distance = std::abs(distance);
  }

  if (nav == Goal::NAV_FORWARD || nav == Goal::NAV_REVERSE ||
      nav == Goal::NAV_BEZIER_FORWARD || nav == Goal::NAV_BEZIER_REVERSE)
  {
    led_side_ = 0;
    line_sensor_.setPreferredSide(0);
  }
  else if (nav == Goal::NAV_FORWARD_LEFT || nav == Goal::NAV_REVERSE_LEFT ||
           nav == Goal::NAV_BEZIER_FORWARD_LEFT || nav == Goal::NAV_BEZIER_REVERSE_LEFT)
  {
    led_side_ = 1;
    line_sensor_.setPreferredSide(1);
  }
  else if (nav == Goal::NAV_FORWARD_RIGHT || nav == Goal::NAV_REVERSE_RIGHT ||
           nav == Goal::NAV_BEZIER_FORWARD_RIGHT || nav == Goal::NAV_BEZIER_REVERSE_RIGHT)
  {
    led_side_ = 2;
    line_sensor_.setPreferredSide(2);
  }

  io_trigger_latch_.setState(io_.getInput(io_trigger_port, io_trigger_pin));
  error_io_trigger_latch_.setState(io_.getInput(error_io_trigger_port, error_io_trigger_pin));

  if (next_motion == Goal::MOTION_LEFT)
  {
    next_led_side_ = 1;
  }
  else if (next_motion == Goal::MOTION_RIGHT)
  {
    next_led_side_ = 2;
  }
  else
  {
    next_led_side_ = 0;
  }

  if (!old_forward_flag_)
  {
    lineForwardSpeedUpdate();
  }

  if (config_.straight_out_of_line_pre_check && speed_ == 0.0f)
  {
    // out of line checking before start moving
    out_of_line_filter_.setState(true);
  }

  junction_stopping_speed_ = config_.straight_junction_speed;
  if (config_.getProfile().straight_jerk_dec > 0.0f)
  {
    if (goal_.io_trigger_type && goal_.next_distance > 0.0f)
    {
      float speed = ReachingKinematic<float>(config_.straight_junction_min_speed,
                                             config_.io_trigger_stop_dec,
                                             config_.io_trigger_stop_jerk).speed(goal_.next_distance);
      junction_stopping_speed_ = std::min(junction_stopping_speed_, speed);
    }
    junction_stopping_speed_ = std::min(junction_stopping_speed_,
                                        reaching_.speed(getTargetJunctionDistance0()));
    junction_stopping_speed_ = std::max<float>(junction_stopping_speed_,
                                               config_.straight_junction_min_speed);
  }

  double jd = forward_dir_ ? config_.forward_junction_distance : config_.reverse_junction_distance;
  if (distance > 0)
  {
    state_ = BY_DISTANCE;
    junction_stopping_distance_ = getTargetJunctionDistance0();
  }
  else if (speed_ == 0.0f && goal_.next_distance < 0.0f && std::abs(goal_.next_distance) < jd)
  {
    if (forward_flag_)
    {
      completeNav(Result::RESULT_SUCCESS);
    }
    else
    {
      state_ = BY_DISTANCE;
      junction_stopping_distance_ = getTargetJunctionDistance0() - (jd + goal_.next_distance);

      if (junction_stopping_distance_ < 0)
      {
        ROS_ERROR("Overshot %f: %s junction (%f / %f)", junction_stopping_distance_,
                  forward_dir_ ? "forward" : "reverse", goal_.next_distance, jd);
        completeNav(Result::RESULT_FAIL);
      }
      else
      {
        ROS_WARN("Not possible to find %s junction (%f / %f)",
                 forward_dir_ ? "forward" : "reverse", goal_.next_distance, jd);
      }
    }
  }
  else if (!forward_flag_)
  {
    // limit maximum speed according to forward flag
    line_speed_ = std::min(line_speed_, junction_stopping_speed_);
  }

  switch (goal_.nav)
  {
  case Goal::NAV_BEZIER_FORWARD:
  case Goal::NAV_BEZIER_FORWARD_LEFT:
  case Goal::NAV_BEZIER_FORWARD_RIGHT:
  case Goal::NAV_BEZIER_REVERSE:
  case Goal::NAV_BEZIER_REVERSE_LEFT:
  case Goal::NAV_BEZIER_REVERSE_RIGHT:
    if (config_.getProfile().bezier_max_speed > 0.0f)
    {
      line_speed_ = std::min(line_speed_, config_.getProfile().bezier_max_speed);
    }
    // fall through
  case Goal::NAV_FORWARD:
  case Goal::NAV_FORWARD_LEFT:
  case Goal::NAV_FORWARD_RIGHT:
  case Goal::NAV_REVERSE:
  case Goal::NAV_REVERSE_LEFT:
  case Goal::NAV_REVERSE_RIGHT:
  case Goal::NAV_LINE_TUNE_PID:
  default:
    line_speed_ = std::min(line_speed_, config_.getProfile().straight_max_speed);
    break;
  }

  // limit maximum speed if personnel detection means in travel direction is permanently muted (ISO 3691-4)
  double detection_muted_speed = forward_dir_ ? config_.forward_detection_muted_speed :
                                                config_.reverse_detection_muted_speed;
  if (detection_muted_speed > 0.0)
  {
    line_speed_ = std::min<float>(line_speed_, detection_muted_speed);
  }

  line_speed_ = std::max<float>(line_speed_, config_.straight_junction_min_speed);

  resetPose();

  updateLineSensorData(true);
}

void ActionStraight::process(float frequency)
{
  if (isCompleted()) return;

  bool enable_sensor = goal_.enable_sensor;
  uint8_t io_trigger_port = goal_.io_trigger_port;
  uint8_t io_trigger_pin = goal_.io_trigger_pin;
  uint8_t error_io_trigger_port = goal_.error_io_trigger_port;
  uint8_t error_io_trigger_pin = goal_.error_io_trigger_pin;

  // process out of line trigger
  updateLineSensorData(false);
  out_of_line_filter_.update(lsd_.out_of_line, std::abs(base_.getStraightDistance() - initial_olsd_));
  if (!lsd_.out_of_line)
  {
    // clear immediately when line found
    out_of_line_filter_.setState(false);
    line_follow_theta_ = base_.getRotationalDistance();
  }
  else if (std::abs(base_.getRotationalDistance() - line_follow_theta_) > config_.straight_out_of_line_angle)
  {
    out_of_line_filter_.setState(true);
  }
  initial_olsd_ = base_.getStraightDistance();

  if (isPausedManualControl())
  {
    switch (goal_.nav)
    {
    case Goal::NAV_LINE_TUNE_PID:
      state_ = STOP_EXIT;
      break;
    default:
      return ActionManualControl::process(frequency);
    }
  }

  // update laser sensor area
  switch (goal_.nav)
  {
  case Goal::NAV_LINE_TUNE_PID:
    break;
  default:
    {
      uint8_t area_old = laser_sensor_.getArea();
      uint8_t area_new;

      if ((state_ == OVER_JUNCTION || state_ == STOP_EXIT_JUNCTION) && !forward_flag_)
      {
        // disable laser sensor after AGV crossed junction
        area_new = forward_dir_ ? agv05_msgs::ObstacleSensorArea::AREA_FORWARD_END :
                                  agv05_msgs::ObstacleSensorArea::AREA_REVERSE_END;
      }
      else if (speed_ < config_.straight_near_block_speed)
      {
        area_new = forward_dir_ ? agv05_msgs::ObstacleSensorArea::AREA_FORWARD_NEAR :
                                  agv05_msgs::ObstacleSensorArea::AREA_REVERSE_NEAR;
      }
      else if (speed_ < config_.straight_middle_block_speed)
      {
        area_new = forward_dir_ ? agv05_msgs::ObstacleSensorArea::AREA_FORWARD_MIDDLE :
                                  agv05_msgs::ObstacleSensorArea::AREA_REVERSE_MIDDLE;
      }
      else if (speed_ < config_.straight_far_block_speed)
      {
        area_new = forward_dir_ ? agv05_msgs::ObstacleSensorArea::AREA_FORWARD_FAR :
                                  agv05_msgs::ObstacleSensorArea::AREA_REVERSE_FAR;
      }
      else
      {
        area_new = forward_dir_ ? agv05_msgs::ObstacleSensorArea::AREA_FORWARD :
                                  agv05_msgs::ObstacleSensorArea::AREA_REVERSE;
      }

      if (area_new != area_old)
      {
        laser_sensor_.selectArea(area_new);
      }
    }
    break;
  }

  // safety check
  bool trigger_out_of_line = out_of_line_filter_.getState() &&
                             state_ != OVER_JUNCTION &&
                             state_ != STOP_EXIT &&
                             state_ != STOP_EXIT_JUNCTION &&
                             state_ != STOP_EXIT_IO_DETECTED &&
                             state_ != STOP_EXIT_ERROR_TRIGGERED;
  uint8_t status = checkSafetyStatus(enable_sensor, false, trigger_out_of_line, getLineSensor());
  safety_.publishNavTrigger(status != Feedback::STATUS_NORMAL, safety_internal_message_);

  // set speed limits
  if (io_trigger_ || state_ == STOP_EXIT_ERROR_TRIGGERED)
  {
    speed_.setAccelerationLimits(config_.getProfile().straight_normal_acc,
                                 config_.io_trigger_stop_dec,
                                 config_.io_trigger_stop_jerk,
                                 config_.io_trigger_stop_jerk);
  }
  else
  {
    speed_.setAccelerationLimits(config_.getProfile().straight_normal_acc,
                                 config_.getProfile().straight_normal_dec,
                                 config_.getProfile().straight_jerk_acc,
                                 config_.getProfile().straight_jerk_dec);
  }
  speed_.clearSpeedLimit();
  speed_.addSpeedLimit(line_speed_);

  if (status != Feedback::STATUS_NORMAL)
  {
    speed_.addSpeedLimit(0.0f);
  }
  else if (isPaused())
  {
    speed_.addSpeedLimit(0.0f);
    status = Feedback::STATUS_PAUSED;
  }
  else if (!base_.isNavigationEnabled())
  {
    // No safety triggered but hardware is not ready. Use small speed to wake up hardware.
    speed_.addSpeedLimit(STOP_THRESHOLD_SPEED);
  }
  else if (!base_.isSteeringAligned())
  {
    // reaccelerate from zero to publish a small value as hint to motor
    speed_.setSpeed(0.0f);
  }
  else if (laser_sensor_.getActivation().near_blocked)
  {
    speed_.addSpeedLimit(config_.straight_near_block_speed);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }
  else if (laser_sensor_.getActivation().middle_blocked)
  {
    speed_.addSpeedLimit(config_.straight_middle_block_speed);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }
  else if (laser_sensor_.getActivation().far_blocked)
  {
    speed_.addSpeedLimit(config_.straight_far_block_speed);
    status = Feedback::STATUS_OBSTACLE_IN_RANGE;
  }

  // process io trigger
  io_trigger_latch_.update(io_.getInput(io_trigger_port, io_trigger_pin), 1.0f / frequency);
  error_io_trigger_latch_.update(io_.getInput(error_io_trigger_port, error_io_trigger_pin), 1.0f / frequency);

  // publish status
  publishStatus(status, state_ < ON_JUNCTION ? led_side_ : next_led_side_);

  // state machine
  switch (state_)
  {
  case RUNNING:
  case BY_DISTANCE:
    running(frequency);
    break;

  case ON_JUNCTION:
  case OVER_JUNCTION:
  case OVER_IO_DETECTED:
  case OVER_DISTANCE:
    onOverJunction(frequency);
    break;

  case STOP_EXIT:
  case STOP_EXIT_JUNCTION:
  case STOP_EXIT_IO_DETECTED:
  case STOP_EXIT_ERROR_TRIGGERED:
    stopExit(frequency);
    break;

  default:
    ROS_ASSERT_MSG(0, "Unknown state.");
    break;
  }
}

void ActionStraight::handleNavControl(const NavControl& nav_control)
{
  ActionProcessor::handleNavControl(nav_control);
  if (nav_control.control == NavControl::CONTROL_CANCEL_FORWARD_FLAG)
  {
    forward_flag_ = false;
    line_speed_ = std::min(line_speed_, junction_stopping_speed_);

    // force immediate speed reduction if AGV has passed the junction
    if (state_ >= STOP_EXIT)
    {
      if (speed_ > config_.straight_junction_min_speed)
      {
        speed_.setSpeed(config_.straight_junction_min_speed);
      }
    }
    else if (state_ >= ON_JUNCTION)
    {
      float current_junction_distance = getStraightDistance0();
      float junction_speed = config_.getProfile().straight_jerk_dec > 0.0f ?
                             reaching_.speed(junction_stopping_distance_ - current_junction_distance) :
                             computeJunctionSpeed(current_junction_distance);
      if (speed_ > junction_speed)
      {
        speed_.setSpeed(junction_speed);
      }
    }
  }
}

void ActionStraight::running(float frequency)
{
  speed_.update(frequency, computeLinearErrorSpeed(inline_lsd_.linear_error));
  lineFollow(frequency, speed_, inline_lsd_.angular_error, inline_lsd_.heading_error);

  // abort initiated
  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  // io detected
  else if (io_trigger_latch_.getLatchState())
  {
    io_trigger_ = true;
    state_ = goal_.next_distance > 0.0f ? OVER_IO_DETECTED : STOP_EXIT_IO_DETECTED;
    initial_jsd_ = base_.getStraightDistance();
    junction_stopping_distance_ = goal_.next_distance;
  }
  // error triggered
  else if (error_io_trigger_latch_.getLatchState())
  {
    state_ = STOP_EXIT_ERROR_TRIGGERED;
  }
  // by distance
  else if (state_ == BY_DISTANCE)
  {
    float current_distance = getStraightDistance0();
    float target_distance = junction_stopping_distance_ - current_distance;
    if (target_distance < reaching_.distance(speed_, speed_.getAccel()))
    {
      state_ = OVER_DISTANCE;
    }
  }
  // junction detected
  else if (lsd_.on_junction && !safety_.getSafetyTrigger().emergency_button)
  {
    state_ = ON_JUNCTION;

    // lock distance value
    initial_jsd_ = base_.getStraightDistance();

    // calculate junction distance based on current junction speed
    if (config_.getProfile().straight_jerk_dec > 0.0f)
    {
      junction_stopping_distance_ = getTargetJunctionDistance0();
    }
    else
    {
      junction_stopping_distance_ = getTargetJunctionDistance0() - (speed_ - 0.3) * 0.27;
    }

    switch (goal_.next_motion)
    {
    case Goal::MOTION_NONSTOP_BEZIER:
      if (config_.getProfile().bezier_max_speed > 0.0f)
      {
        line_speed_ = std::min(line_speed_, config_.getProfile().bezier_max_speed);
      }
      // fall through
    case Goal::MOTION_NONSTOP:
      if (goal_.next_speed > 0.0f)
      {
        line_speed_ = std::min(line_speed_, goal_.next_speed);
      }
      line_speed_ = std::max<float>(line_speed_, config_.straight_junction_min_speed);
      break;
    default:  // stopping
      break;
    }
  }
}

void ActionStraight::onOverJunction(float frequency)
{
  // current running distance from junction
  const float current_junction_distance = getStraightDistance0();
  const float target_distance = junction_stopping_distance_ - current_junction_distance;

  if (speed_ > 0.0f)
  {
    position_align_filter_.update(target_distance < config_.undershoot_stopping_distance, 1.0f / frequency);
  }

  /*
  // get junction speed
  float junction_speed =
    fabs(getStraightDistance0()
    - (getTargetJunctionDistance0() - 0.05)) /
    (getTargetJunctionDistance0() - 0.05) * config_.straight_junction_speed + 0.07;
  // limit targeted speed to junction speed
  target_speed = std::min(target_speed, junction_speed);
  */

  // move normally if forward_flag is set
  if (forward_flag_)
  {
    speed_.update(frequency, computeLinearErrorSpeed(inline_lsd_.linear_error));
  }
  // junction stopping
  else
  {
    // calculate speed profile for junction
    float junction_speed;

    if (config_.getProfile().straight_jerk_dec > 0.0f)
    {
      junction_speed = reaching_.speed(target_distance);
      junction_speed = std::min<float>(junction_speed, config_.straight_junction_speed);
    }
    else
    {
      // ensure minimum speed limit is met
      junction_speed = std::max<float>(speed_, config_.straight_junction_min_speed);
      junction_speed = std::min(junction_speed, computeJunctionSpeed(current_junction_distance));
    }
    junction_speed = std::min(junction_speed, computeLinearErrorSpeed(inline_lsd_.linear_error));
    overshoot_ = reaching_.distance(speed_, speed_.getAccel()) - target_distance;
    speed_.update(frequency, junction_speed);

    ROS_DEBUG("calculated speed %f", junction_speed);
  }

  if (lsd_.on_junction && config_.persist_error_over_junction)
  {
    // use the line sensor error prior to entering junction
    lineFollow(frequency, speed_, initial_lsd_.angular_error, initial_lsd_.heading_error);
  }
  else
  {
    lineFollow(frequency, speed_, inline_lsd_.angular_error, inline_lsd_.heading_error);
  }

  // abort initiated
  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  // io detected
  else if (io_trigger_ && current_junction_distance >= junction_stopping_distance_)
  {
    state_ = STOP_EXIT_IO_DETECTED;
  }
  // error triggered
  else if (error_io_trigger_latch_.getLatchState())
  {
    state_ = STOP_EXIT_ERROR_TRIGGERED;
  }
  else if (!io_trigger_ && io_trigger_latch_.getLatchState())
  {
    io_trigger_ = true;
    state_ = goal_.next_distance > 0.0f ? OVER_IO_DETECTED : STOP_EXIT_IO_DETECTED;
    initial_jsd_ = base_.getStraightDistance();
    junction_stopping_distance_ = goal_.next_distance;
  }
  // junction reach detected (when forward_flag = 1)
  else if (forward_flag_ && state_ == OVER_JUNCTION &&
           current_junction_distance > std::min(junction_stopping_distance_, 0.1f))
  {
    completeNav(Result::RESULT_SUCCESS);
  }
  // junction reach detected (when forward_flag = 0)
  else if (current_junction_distance >= junction_stopping_distance_ || position_align_filter_.getState())
  {
    state_ = STOP_EXIT_JUNCTION;
  }
  // out junction detected
  else if (!lsd_.on_junction && state_ == ON_JUNCTION)
  {
    // lock distance value
    initial_jsd_ = (base_.getStraightDistance() + initial_jsd_) / 2.0;

    // if distance is negative, it is measured after the junction
    if (goal_.distance < 0)
    {
      state_ = OVER_DISTANCE;
    }
    else
    {
      state_ = OVER_JUNCTION;
    }
  }
}

void ActionStraight::stopExit(float frequency)
{
  // io detected
  if (io_trigger_)
  {
    if (getStraightDistance0() >= junction_stopping_distance_)
    {
      state_ = STOP_EXIT_IO_DETECTED;
    }
  }
  else if (io_trigger_latch_.getLatchState())
  {
    io_trigger_ = true;
    initial_jsd_ = base_.getStraightDistance();
    junction_stopping_distance_ = goal_.next_distance;
  }

  // speed zeroed
  float straight_min_speed = overshoot_ > config_.overshoot_stopping_distance ?
                             config_.straight_junction_min_speed : STOP_THRESHOLD_SPEED;
  if (speed_ < straight_min_speed)
  {
    speed_.setSpeed(0.0f);
    base_.stop();
    if (state_ == STOP_EXIT_IO_DETECTED)
    {
      completeNav(Result::RESULT_IO_DETECTED);
    }
    else if (state_ == STOP_EXIT_ERROR_TRIGGERED)
    {
      completeNav(Result::RESULT_ERROR_TRIGGERED);
    }
    else
    {
      completeNav(Result::RESULT_SUCCESS);
    }
  }
  else
  {
    overshoot_ = reaching_.distance(speed_, speed_.getAccel()) + getStraightDistance0() - junction_stopping_distance_;
    speed_.update(frequency, 0.0f);
    lineFollow(frequency, speed_, inline_lsd_.angular_error, inline_lsd_.heading_error);
  }
}

void ActionStraight::lineFollow(float frequency, float speed, float line_angular_error, float line_heading_error)
{
  if (frequency <= 0.0f) return;

  // calculate p
  float error_p = (line_angular_error + line_heading_error) * 0.5f;

  // calculate i
  float error_i = line_follow_error_i_ + (error_p / frequency);  // ∫ e dt

  // calculate d
  float error_d;
  if (config_.getProfile().line_follow_pid_traditional)
  {
    error_d = (error_p - line_follow_error_d_) * frequency;  // de/dt

    // update error
    line_follow_error_d_ = error_p;
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

    // -ve sign is dropped as output is in opposite sign of error
    error_d = (base_.getRotationalDistance() - line_follow_error_d_) * frequency;  // dθ/dt
    line_follow_error_d_ =  base_.getRotationalDistance();
  }

  // calculate output
  float output =
    config_.getProfile().line_follow_p * error_p +
    config_.getProfile().line_follow_i * error_i +
    config_.getProfile().line_follow_d * error_d;

  // saturate output
  float output_limit = config_.getProfile().turn_max_speed;
  // saturation is limited to a ratio of linear speed
  output_limit *= (std::max<float>(speed, config_.straight_junction_min_speed) / line_speed_);

  if (speed == 0.0f)
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
    line_follow_error_i_ = error_i;
  }

  // write output to motor
  if (forward_dir_)
  {
    base_.setSpeed(speed, -output);
  }
  else
  {
    base_.setSpeed(-speed, -output);
  }

  ROS_DEBUG("Speed %f, Angular error %f, Heading error %f, Output %f",
            forward_dir_ ? speed : -speed, line_angular_error, line_heading_error, -output);
}

void ActionStraight::resetLineFollowVariable()
{
  // clear all line_follow_error
  line_follow_error_i_ = 0.0f;
  line_follow_error_d_ = config_.getProfile().line_follow_pid_traditional ? 0.0f : base_.getRotationalDistance();
  line_follow_theta_ = base_.getRotationalDistance();
}

void ActionStraight::lineForwardSpeedUpdate()
{
  geometry_msgs::Twist cmd_vel;
  if (base_.getMotorCmdVel(cmd_vel))
  {
    float speed = forward_dir_ ? cmd_vel.linear.x : -cmd_vel.linear.x;

    // update forward_flag speed
    if (speed > 0.0f)
    {
      ROS_WARN("set forward_flag speed: %f, cmd_vel.linear.x: %f", speed, cmd_vel.linear.x);
      speed_.setSpeed(speed);
    }
    // clear expired forward_flag speed
    else if (speed_ > 0.0f)
    {
      ROS_WARN("clear forward_flag speed: %f, cmd_vel.linear.x: %f", (float)speed_, cmd_vel.linear.x);
      speed_.setSpeed(0.0f);
    }
  }
}

float ActionStraight::computeJunctionSpeed(float junction_distance)
{
  float junction_speed =
    config_.straight_junction_min_speed +
    (junction_stopping_distance_ - junction_distance) / junction_stopping_distance_ *
    (config_.straight_junction_speed - config_.straight_junction_min_speed);

  // ensure speed is within min-max range
  return std::min<float>(
           std::max<float>(junction_speed, config_.straight_junction_min_speed),
           config_.straight_junction_speed);
}

float ActionStraight::computeLinearErrorSpeed(float linear_error)
{
  linear_error = std::abs(linear_error);
  if (linear_error < config_.getProfile().line_follow_error_min)
  {
    return line_speed_;
  }
  linear_error -= config_.getProfile().line_follow_error_min;

  float d = config_.getProfile().line_follow_error_max - config_.getProfile().line_follow_error_min;
  if (d > 0.0f)
  {
    return std::max(line_speed_ * (1.0f - linear_error / d), STOP_THRESHOLD_SPEED);
  }

  return STOP_THRESHOLD_SPEED;
}

void ActionStraight::updateLineSensorData(bool init)
{
  updatePose();

  switch (goal_.line_follow_type)
  {
  case Goal::LINE_FOLLOW_ODOM_2D:
    if (!init)
    {
      lsd_.angular_error = getAngularError0();
      lsd_.heading_error = getHeadingError0();
      break;
    }
    // fall through
  case Goal::LINE_FOLLOW_ODOM_1D:
    if (init)
    {
      lsd_.linear_error = 0.0f;
      lsd_.angular_error = 0.0f;
      lsd_.heading_error = 0.0f;
      lsd_.out_of_line = false;
      lsd_.on_junction = false;
      lsd_.enable = true;
      lsd_.sensor_error = agv05_msgs::LineSensor::NORMAL;
      lsd_.junction_count = 0;
    }
    break;
  case Goal::LINE_FOLLOW_TRACK:
  default:
    if (init)
    {
      lsd_ = getLineSensorData0();
    }
    else
    {
      agv05_msgs::LineSensor lsd = getLineSensorData0();

      // process junction count (skip value zero which is due to msb reset)
      if (lsd.junction_count && lsd.junction_count != lsd_.junction_count)
      {
        lsd.on_junction = true;
      }

      if (lsd.on_junction && !lsd_.on_junction)
      {
        // preserve lsd data
        initial_lsd_ = lsd_;

        // preserve odom data
        if (!forward_flag_)
        {
          resetPose();
        }
      }
      else if (!forward_flag_ && (state_ == OVER_JUNCTION))
      {
        if (lsd.out_of_line)
        {
          inline_lsd_.angular_error = getAngularError0();
          inline_lsd_.heading_error = getHeadingError0();
        }
        else
        {
          line_vector_ = poseComplex();
        }
      }

      lsd_ = lsd;
    }
    break;
  }

  if (!lsd_.out_of_line)
  {
    inline_lsd_ = lsd_;
  }
}

float ActionStraight::getStraightDistance0()
{
  if (goal_.line_follow_type == Goal::LINE_FOLLOW_ODOM_2D)
  {
    return dot(line_vector_, poseComplex());
  }
  float distance = base_.getStraightDistance() - initial_jsd_;
  return forward_dir_ ? distance : -distance;
}

float ActionStraight::getLinearError0()
{
  return cross(line_vector_, poseComplex());
}

float ActionStraight::getAngularError0()
{
  return getAngularError0(getLinearError0());
}

float ActionStraight::getAngularError0(float linear_error)
{
  return atan2(linear_error, config_.getProfile().line_follow_ahead_distance);
}

float ActionStraight::getHeadingError0()
{
  return base_.getRotationalDistance() - line_start_.theta;
}

ActionProcessor::UseLineSensor ActionStraight::getLineSensor()
{
  return goal_.line_follow_type == Goal::LINE_FOLLOW_TRACK ? forward_dir_ ?
         LINE_SENSOR_FRONT : LINE_SENSOR_REAR : LINE_SENSOR_DISABLE;
}

agv05_msgs::LineSensor ActionStraight::getLineSensorData0()
{
  agv05_msgs::LineSensor lsd = forward_dir_ ? line_sensor_.getFrontData() : line_sensor_.getRearData();
  lsd.angular_error = getAngularError0(lsd.linear_error);
  return lsd;
}

float ActionStraight::getTargetJunctionDistance0()
{
  double distance;

  if (goal_.distance > 0)
  {
    distance = goal_.distance;
  }
  else if (goal_.distance < 0)
  {
    distance = -goal_.distance;
  }
  else if (goal_.io_trigger_type)
  {
    distance = config_.io_trigger_stop_distance;
  }
  else if (forward_dir_)
  {
    distance = config_.forward_junction_distance;
  }
  else
  {
    distance = config_.reverse_junction_distance;
  }

  if (forward_dir_)
  {
    return distance +
           (next_led_side_ == 1 ? config_.forward_stopping_offset_prior_turning_left :
            next_led_side_ == 2 ? config_.forward_stopping_offset_prior_turning_right :
                                  config_.forward_stopping_offset);
  }
  else
  {
    return distance +
           (next_led_side_ == 1 ? config_.reverse_stopping_offset_prior_turning_left :
            next_led_side_ == 2 ? config_.reverse_stopping_offset_prior_turning_right :
                                  config_.reverse_stopping_offset);
  }
}

/* Definition for static variables */
VelocitySmoother<float> ActionStraight::speed_;

}  // namespace agv05

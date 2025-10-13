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

void ActionStraight::initialize()
{
  resetPathFollowVariable();

  uint8_t nav = goal_.nav;
  uint8_t io_trigger_port = goal_.io_trigger_port;
  uint8_t io_trigger_pin = goal_.io_trigger_pin;
  uint8_t error_io_trigger_port = goal_.error_io_trigger_port;
  uint8_t error_io_trigger_pin = goal_.error_io_trigger_pin;
  uint8_t next_motion = goal_.next_motion;

  if (nav == Goal::NAV_FORWARD || nav == Goal::NAV_BEZIER_FORWARD ||
      nav == Goal::NAV_FORWARD_DOCK || nav == Goal::NAV_FORWARD_UNDOCK)
  {
    forward_dir_ = true;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_FORWARD);
  }
  else if (nav == Goal::NAV_REVERSE || nav == Goal::NAV_BEZIER_REVERSE ||
           nav == Goal::NAV_REVERSE_DOCK || nav == Goal::NAV_REVERSE_UNDOCK)
  {
    forward_dir_ = false;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_REVERSE);
  }
  else if (nav == Goal::NAV_DYNAMIC_FORWARD || nav == Goal::NAV_DYNAMIC_LINE_FORWARD ||
           nav == Goal::NAV_DYNAMIC_BEZIER_FORWARD)
  {
    forward_dir_ = true;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_FORWARD);
  }
  else if (nav == Goal::NAV_DYNAMIC_REVERSE || nav == Goal::NAV_DYNAMIC_LINE_REVERSE ||
           nav == Goal::NAV_DYNAMIC_BEZIER_REVERSE)
  {
    forward_dir_ = false;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_REVERSE);
  }
  else if (nav == Goal::NAV_ROTATE_LEFT || nav == Goal::NAV_ROTATE_RIGHT)
  {
    forward_dir_ = true;
  }
  else if (nav == Goal::NAV_OMNI_DOCK)
  {
    forward_dir_ = true;
    laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_OMNI);
  }
  else if (nav == Goal::NAV_OMNI || nav == Goal::NAV_BEZIER_OMNI || nav == Goal::NAV_OMNI_UNDOCK)
  {
    path_heading_ = nav == Goal::NAV_OMNI_UNDOCK ? goal_.path_cp2.theta : goal_.path_start.theta;
    forward_dir_ = path_heading_ >= 0.0;
    if (path_heading_ > 0.0)
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_OMNI);
    }
    else if (forward_dir_)
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_FORWARD);
    }
    else
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_REVERSE);
    }
  }
  else if (nav == Goal::NAV_DYNAMIC_OMNI || nav == Goal::NAV_DYNAMIC_LINE_OMNI ||
           nav == Goal::NAV_DYNAMIC_BEZIER_OMNI)
  {
    path_heading_ = goal_.path_start.theta;
    forward_dir_ = path_heading_ >= 0.0;
    if (path_heading_ > 0.0)
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_OMNI);
    }
    else if (forward_dir_)
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_FORWARD);
    }
    else
    {
      laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_REVERSE);
    }
  }

  io_trigger_latch_.setState(io_.getInput(io_trigger_port, io_trigger_pin));
  error_io_trigger_latch_.setState(io_.getInput(error_io_trigger_port, error_io_trigger_pin));

  led_side_ = 0;
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
    pathForwardSpeedUpdate();
  }

  if (goal_.io_trigger_type && goal_.next_distance > 0.0f)
  {
    float speed = ReachingKinematic<float>(config_.straight_reaching_min_speed,
                                           config_.io_trigger_stop_dec,
                                           config_.io_trigger_stop_jerk).speed(goal_.next_distance);
    path_speed_ = std::min(path_speed_, speed);
  }
  switch (goal_.nav)
  {
  default:
  case Goal::NAV_BEZIER_FORWARD:
  case Goal::NAV_BEZIER_REVERSE:
  case Goal::NAV_BEZIER_OMNI:
  case Goal::NAV_FORWARD_DOCK:
  case Goal::NAV_REVERSE_DOCK:
  case Goal::NAV_OMNI_DOCK:
  case Goal::NAV_FORWARD_UNDOCK:
  case Goal::NAV_REVERSE_UNDOCK:
  case Goal::NAV_OMNI_UNDOCK:
  case Goal::NAV_ROTATE_LEFT:
  case Goal::NAV_ROTATE_RIGHT:
  case Goal::NAV_DYNAMIC_BEZIER_FORWARD:
  case Goal::NAV_DYNAMIC_BEZIER_REVERSE:
  case Goal::NAV_DYNAMIC_BEZIER_OMNI:
    path_speed_ = std::min(path_speed_, config_.getProfile().bezier_max_speed);
    // fall through
  case Goal::NAV_FORWARD:
  case Goal::NAV_REVERSE:
  case Goal::NAV_OMNI:
  case Goal::NAV_DYNAMIC_FORWARD:
  case Goal::NAV_DYNAMIC_REVERSE:
  case Goal::NAV_DYNAMIC_OMNI:
  case Goal::NAV_DYNAMIC_LINE_FORWARD:
  case Goal::NAV_DYNAMIC_LINE_REVERSE:
  case Goal::NAV_DYNAMIC_LINE_OMNI:
    path_speed_ = std::min(path_speed_, config_.getProfile().straight_max_speed);
    break;
  }

  // limit maximum speed if personnel detection means in travel direction is permanently muted (ISO 3691-4)
  double detection_muted_speed = forward_dir_ ? config_.forward_detection_muted_speed :
                                                config_.reverse_detection_muted_speed;
  if (detection_muted_speed > 0.0)
  {
    path_speed_ = std::min<float>(path_speed_, detection_muted_speed);
  }

  path_speed_ = std::max<float>(path_speed_, config_.straight_reaching_min_speed);
  path_curvature_ = config_.getProfile().turn_max_speed / path_speed_;
  next_speed_ = path_speed_;

  reaching_distance_ = config_.straight_reaching_distance;
  if (config_.getProfile().straight_jerk_dec > 0.0f)
  {
    reaching_distance_ = std::max(reaching_distance_,
                                  reaching_.distance(path_speed_, config_.getProfile().straight_normal_acc));
  }
  reaching_distance_ += config_.localization_tolerance_distance;
}

void ActionStraight::straight(float frequency)
{
  if (isCompleted()) return;

  if (isPausedManualControl())
  {
    return ActionManualControl::process(frequency);
  }

  bool enable_sensor = goal_.enable_sensor;
  uint8_t io_trigger_port = goal_.io_trigger_port;
  uint8_t io_trigger_pin = goal_.io_trigger_pin;
  uint8_t error_io_trigger_port = goal_.error_io_trigger_port;
  uint8_t error_io_trigger_pin = goal_.error_io_trigger_pin;

  // update laser sensor area
  switch (goal_.nav)
  {
  case Goal::NAV_DYNAMIC_FORWARD:
  case Goal::NAV_DYNAMIC_REVERSE:
  case Goal::NAV_DYNAMIC_LINE_FORWARD:
  case Goal::NAV_DYNAMIC_LINE_REVERSE:
  case Goal::NAV_DYNAMIC_BEZIER_FORWARD:
  case Goal::NAV_DYNAMIC_BEZIER_REVERSE:
  case Goal::NAV_ROTATE_LEFT:
  case Goal::NAV_ROTATE_RIGHT:
  case Goal::NAV_OMNI_DOCK:
    break;
  case Goal::NAV_OMNI:
  case Goal::NAV_BEZIER_OMNI:
  case Goal::NAV_OMNI_UNDOCK:
  case Goal::NAV_DYNAMIC_OMNI:
  case Goal::NAV_DYNAMIC_LINE_OMNI:
  case Goal::NAV_DYNAMIC_BEZIER_OMNI:
    if (path_heading_ > 0.0)
    {
      uint8_t area_old = laser_sensor_.getArea();
      uint8_t area_new;

      if ((state_ == ALIGN_POSITION || state_ == STOP_EXIT_REACHING) && !forward_flag_)
      {
        // disable laser sensor after AGV crossed junction
        area_new = agv05_msgs::ObstacleSensorArea::AREA_OMNI_END;
      }
      else if (speed_ < config_.straight_near_block_speed)
      {
        area_new = agv05_msgs::ObstacleSensorArea::AREA_OMNI_NEAR;
      }
      else if (speed_ < config_.straight_middle_block_speed)
      {
        area_new = agv05_msgs::ObstacleSensorArea::AREA_OMNI_MIDDLE;
      }
      else if (speed_ < config_.straight_far_block_speed)
      {
        area_new = agv05_msgs::ObstacleSensorArea::AREA_OMNI_FAR;
      }
      else
      {
        area_new = agv05_msgs::ObstacleSensorArea::AREA_OMNI;
      }

      if (area_new != area_old)
      {
        laser_sensor_.selectArea(area_new);
      }
      break;
    }
    // fall through
  default:
    {
      uint8_t area_old = laser_sensor_.getArea();
      uint8_t area_new;

      if ((state_ == ALIGN_POSITION || state_ == STOP_EXIT_REACHING) && !forward_flag_)
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
  speed_.addSpeedLimit(path_speed_);

  // safety check
  bool trigger_navigation_failed = !base_.getPose(pose_) || !handlePoseUpdate() ||
                                   (config_.straight_out_of_path_distance > 0.0 &&
                                    config_.straight_out_of_path_distance < fabs(getLinearError0()));
  uint8_t status = checkSafetyStatus(enable_sensor, false, trigger_navigation_failed, getLineSensor());
  safety_.publishNavTrigger(status != Feedback::STATUS_NORMAL, safety_internal_message_);

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
  publishStatus(status, state_ < ALIGN_POSITION ? led_side_ : next_led_side_);

  // state machine
  switch (state_)
  {
  case PLANNING:
    planning(frequency);
    break;

  case RUNNING:
    running(frequency);
    break;

  case ALIGN_POSITION:
    alignPosition(frequency, !forward_flag_);
    break;

  case STOP_EXIT:
  case STOP_EXIT_REACHING:
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

    // force immediate speed reduction if AGV is approaching the end
    if (state_ >= STOP_EXIT)
    {
      if (speed_ > config_.straight_reaching_min_speed)
      {
        speed_.setSpeed(config_.straight_reaching_min_speed);  // jerk will be noticeable
      }
    }
    else if (state_ >= ALIGN_POSITION)
    {
      float reaching_distance = computeReachingDistance(getTargetDistance0());
      float reaching_speed = config_.getProfile().straight_jerk_dec > 0.0f ?
                             reaching_.speed(reaching_distance) :
                             computeReachingSpeed(reaching_distance);
      if (speed_ > reaching_speed)
      {
        speed_.setSpeed(reaching_speed);  // jerk will be noticeable
      }
    }
  }
}

void ActionStraight::running(float frequency)
{
  alignPosition(frequency, false, true);
}

void ActionStraight::alignPosition(float frequency, bool align, bool align_ready, bool force_exit)
{
  float target_distance = getTargetDistance0();
  float heading_error = getHeadingError0();
  float angular_error = getAngularError0();

  if ((align_ready || state_ >= ALIGN_POSITION) && speed_ > 0.0f)
  {
    force_exit |= position_align_filter_.update(target_distance < config_.undershoot_stopping_distance,
                                                1.0f / frequency);
  }

  // rotate on-spot
  if (std::fabs(heading_error) >= config_.getProfile().path_follow_heading_error_max)
  {
    speed_.update(frequency, (std::fabs(heading_error + angular_error) <
                  config_.getProfile().path_follow_heading_error_max || path_heading_ > 0.0) ?
                  config_.straight_reaching_min_speed : STOP_THRESHOLD_SPEED);
  }
  else if (align)
  {
    float reaching_speed;
    float reaching_distance = computeReachingDistance(target_distance);

    if (config_.getProfile().straight_jerk_dec > 0.0f)
    {
      reaching_speed = reaching_.speed(reaching_distance);
    }
    else
    {
      // ensure minimum speed limit is met
      reaching_speed = std::max<float>(speed_, config_.straight_reaching_min_speed);
      reaching_speed = std::min(reaching_speed, computeReachingSpeed(reaching_distance));
    }

    overshoot_ = reaching_.distance(speed_, speed_.getAccel()) - target_distance;
    speed_.update(frequency, reaching_speed);

    ROS_DEBUG("reaching_speed %f", reaching_speed);
  }
  else if (next_speed_ < path_speed_)  // forward_flag_
  {
    if (config_.getProfile().straight_jerk_dec > 0.0f)
    {
      speed_.update(frequency, std::max(reaching_.speed(target_distance), next_speed_));
    }
    else
    {
      speed_.update(frequency, next_speed_);
    }
  }
  else
  {
    speed_.update(frequency);
  }

  if (isAborted())
  {
    state_ = STOP_EXIT;
  }
  else if (io_trigger_ && target_distance <= 0.0f)
  {
    state_ = STOP_EXIT_IO_DETECTED;
  }
  else if (error_io_trigger_latch_.getLatchState())
  {
    state_ = STOP_EXIT_ERROR_TRIGGERED;
  }
  else if (!io_trigger_ && io_trigger_latch_.getLatchState())
  {
    if (base_.Base::getPose(io_trigger_pose_))
    {
      io_trigger_ = true;
      io_trigger_distance_ = base_.getStraightDistance();
      if (align_ready)
      {
        alignPositionInit();
      }
    }
  }
  else if (align_ready)
  {
    if (force_exit || target_distance < reaching_distance_)
    {
      alignPositionInit();
    }
  }
  else if (force_exit || target_distance <= 0.0f)
  {
    if (forward_flag_)
    {
      completeNav(Result::RESULT_SUCCESS);
    }
    else
    {
      state_ = STOP_EXIT_REACHING;
    }
  }

  pathFollow(frequency, speed_, angular_error, heading_error);
}

void ActionStraight::stopExit(float frequency)
{
  // io detected
  if (io_trigger_)
  {
    if (getTargetDistance0() <= 0.0f)
    {
      state_ = STOP_EXIT_IO_DETECTED;
    }
  }
  else if (io_trigger_latch_.getLatchState())
  {
    if (base_.Base::getPose(io_trigger_pose_))
    {
      io_trigger_ = true;
      io_trigger_distance_ = base_.getStraightDistance();
    }
  }

  // speed zeroed
  float straight_min_speed = overshoot_ > config_.localization_tolerance_distance ?
                             config_.straight_reaching_min_speed : STOP_THRESHOLD_SPEED;
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
    overshoot_ = reaching_.distance(speed_, speed_.getAccel()) - getTargetDistance0();
    speed_.update(frequency, 0.0f);
    pathFollow(frequency, speed_, getAngularError0(), getHeadingError0());
  }
}

void ActionStraight::pathFollow(float frequency, float speed, float angular_error, float heading_error)
{
  if (frequency <= 0.0f) return;

  // calculate p
  float error_p = (angular_error + heading_error) * 0.5f;

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
            forward_dir_ ? speed : -speed, angular_error, heading_error, -output);
}

void ActionStraight::resetPathFollowVariable()
{
  // clear all path_follow_error
  path_follow_error_i_ = 0.0f;
  path_follow_error_d_ = (!config_.getProfile().path_follow_pid_traditional && base_.getPose(pose_)) ?
                         pose_.theta : 0.0f;
}

void ActionStraight::pathForwardSpeedUpdate()
{
  geometry_msgs::Twist cmd_vel;
  if (base_.getMotorCmdVel(cmd_vel))
  {
    float speed = path_heading_ > 0.0 ? std::abs(complex(cmd_vel.linear.x, cmd_vel.linear.y)) :
                  forward_dir_ ? cmd_vel.linear.x : -cmd_vel.linear.x;

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

void ActionStraight::pathCurvatureUpdate(float curvature)
{
  if (path_curvature_ < curvature)
  {
    path_speed_ = std::min<float>(path_speed_, config_.getProfile().turn_max_speed / curvature);
    path_speed_ = std::max<float>(path_speed_, config_.straight_reaching_min_speed);
    path_curvature_ = config_.getProfile().turn_max_speed / path_speed_;
  }
}

float ActionStraight::computeOutputLimit(float speed, float heading_error)
{
  return std::max<float>(speed, config_.straight_reaching_min_speed) * path_curvature_;
}

float ActionStraight::computeReachingDistance(float target_distance)
{
  float reaching_distance = target_distance;
  if (reaching_distance > config_.localization_tolerance_distance)
  {
    reaching_distance -= config_.localization_tolerance_distance;
    if (reaching_distance < config_.localization_tolerance_distance)
    {
      reaching_distance = config_.localization_tolerance_distance;
    }
  }
  return reaching_distance;
}

float ActionStraight::computeReachingSpeed(float reaching_distance)
{
  float reaching_speed =
    config_.straight_reaching_min_speed +
    reaching_distance / config_.straight_reaching_distance *
    (config_.straight_reaching_speed - config_.straight_reaching_min_speed);

  // ensure speed is within min-max range
  return std::min<float>(
           std::max<float>(reaching_speed, config_.straight_reaching_min_speed),
           config_.straight_reaching_speed);
}

void ActionStraight::alignPositionInit()
{
  float& path_speed = forward_flag_ ? next_speed_ : path_speed_;
  state_ = ALIGN_POSITION;

  switch (goal_.next_motion)
  {
  case Goal::MOTION_NONSTOP_BEZIER:
    path_speed = std::min(path_speed, config_.getProfile().bezier_max_speed);
    // fall through
  case Goal::MOTION_NONSTOP:
    if (goal_.paths.paths.size() > 1)
    {
      double curvature = getCurvature(goal_.paths.paths[1]) * config_.bezier_curvature_gain;
      if (path_curvature_ < curvature)
      {
        path_speed = std::min<float>(path_speed, config_.getProfile().turn_max_speed / curvature);
      }
    }
    if (goal_.next_speed > 0.0f)
    {
      path_speed = std::min(path_speed, goal_.next_speed);
    }
    if (goal_.next_distance > 0.0f)
    {
      float reaching_distance = computeReachingDistance(goal_.next_distance);
      path_speed = std::min(path_speed, reaching_.speed(reaching_distance));
    }
    break;
  default:  // stopping
    path_speed = std::min<float>(path_speed, config_.straight_reaching_speed);
    break;
  }

  path_speed = std::max<float>(path_speed, config_.straight_reaching_min_speed);
  path_curvature_ = config_.getProfile().turn_max_speed / path_speed;

  switch (goal_.nav)
  {
  case Goal::NAV_OMNI:
  case Goal::NAV_BEZIER_OMNI:
  case Goal::NAV_DYNAMIC_OMNI:
  case Goal::NAV_DYNAMIC_LINE_OMNI:
  case Goal::NAV_DYNAMIC_BEZIER_OMNI:
    if (goal_.path_end.theta > 0.0)
    {
      path_heading_ = goal_.path_end.theta;
    }
  default:
    break;
  }
}

float ActionStraight::getTargetDistance0()
{
  if (io_trigger_)
  {
    return goal_.next_distance - std::abs(base_.getStraightDistance() - io_trigger_distance_);
  }

  const complex v = path_end_ - path_start_;
  const complex pose_v = complex(pose_.x, pose_.y) - path_end_;
  float distance = -dot(v, pose_v) / std::abs(v);
  return adjustTargetDistanceByLineSensing0(distance);
}

float ActionStraight::getLinearError0()
{
  const complex v = path_end_ - path_start_;
  const complex pose_v = complex(pose_.x, pose_.y) - path_end_;
  linear_error_ = cross(v, pose_v) / std::abs(v);  // update protected variable
  return linear_error_;
}

float ActionStraight::getAngularError0()
{
  angular_error_ = getAngularError0(getLinearError0());  // update protected variable
  return angular_error_;
}

float ActionStraight::getAngularError0(float linear_error)
{
  return atan2(linear_error, config_.getProfile().path_follow_ahead_distance);
}

float ActionStraight::getHeadingError0()
{
  const complex v = path_end_ - path_start_;
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

float ActionStraight::adjustTargetDistanceByLineSensing0(float distance)
{
  const float offset =
    forward_dir_ ?
    next_led_side_ == 1 ? config_.forward_stopping_offset_prior_turning_left :
    next_led_side_ == 2 ? config_.forward_stopping_offset_prior_turning_right :
                          config_.forward_stopping_offset :
    next_led_side_ == 1 ? config_.reverse_stopping_offset_prior_turning_left :
    next_led_side_ == 2 ? config_.reverse_stopping_offset_prior_turning_right :
                          config_.reverse_stopping_offset;

  if (sense_line_ > 0 && sense_line_ <= 4)
  {
    if (!sense_line_distance_)
    {
      agv05_msgs::LineSensor lsd = getLineSensorData0();
      if (lsd.enable && !lsd.out_of_line && !lsd.sensor_error)
      {
        if (sense_line_ <= 2  || lsd.on_junction)  // sense_line 3 or 4: detect full junction
        {
          // record distance at first line sensing
          sense_line_distance_ = distance;
        }
      }
    }
    if (sense_line_distance_)
    {
      float adjusted_distance;
      if (sense_line_ == 1 || sense_line_ == 3)
      {
        // stop after stopping_distance_line_sensed
        if (forward_dir_)
        {
          adjusted_distance = distance - sense_line_distance_ + config_.forward_stopping_distance_line_sensed;
        }
        else
        {
          adjusted_distance = distance - sense_line_distance_ + config_.reverse_stopping_distance_line_sensed;
        }
      }
      else  // sense_line 2 or 4
      {
        // stop immediately
        adjusted_distance = distance - sense_line_distance_;
        speed_.setSpeed(0);
      }
      return adjusted_distance + offset;
    }
  }
  return distance + offset;
}

ActionProcessor::UseLineSensor ActionStraight::getLineSensor()
{
  return sense_line_ ? forward_dir_ ?
    LINE_SENSOR_FRONT : LINE_SENSOR_REAR : LINE_SENSOR_DISABLE;
}

agv05_msgs::LineSensor ActionStraight::getLineSensorData0()
{
  if (forward_dir_)
  {
    return line_sensor_.getFrontData();
  }
  else
  {
    return line_sensor_.getRearData();
  }
}

/* Definition for static variables */
VelocitySmoother<float> ActionStraight::speed_;

}  // namespace agv05

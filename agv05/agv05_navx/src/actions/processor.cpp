/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"

#include <agv05_msgs/LedControl.h>
#include <agv05_msgs/ObstacleSensorArea.h>


namespace agv05
{

ActionProcessor::ActionProcessor(Nav& nav, const Goal& goal) :
  base_(nav.base_), io_(nav.io_), laser_sensor_(nav.laser_sensor_),
  line_sensor_(nav.line_sensor_), panel_(nav.panel_), safety_(nav.safety_),
  config_(nav.config_),
  goal_(goal), nav_(nav),
  old_forward_flag_(nav.laser_sensor_.stopDelay() != LaserSensor::STOP_DELAY_OFF),
  forward_flag_((goal.next_motion == Goal::MOTION_NONSTOP) ||
                (goal.next_motion == Goal::MOTION_NONSTOP_BEZIER)),
  aborted_(false), paused_(false),
  status_updated_(false), completed_(false),
  status_(Feedback::STATUS_NORMAL), result_(0),
  safety_status_(Feedback::STATUS_NORMAL)
{}

boost::shared_ptr<ActionProcessor> ActionProcessor::create(Nav& nav, const Goal& goal)
{
  // validate goal parameters
  if (goal.nav == Goal::NAV_IDLE || goal.nav >= Goal::NAV_MAX_NUMBER ||
      goal.sense_line > 4 ||
      goal.io_trigger_type >= FlipFlop<float>::NUM_TRIGGER_TYPES ||
      goal.io_trigger_port > Io::NUM_PORTS ||
      goal.io_trigger_pin >= Io::NUM_PINS ||
      goal.error_io_trigger_type >= FlipFlop<float>::NUM_TRIGGER_TYPES ||
      goal.error_io_trigger_port > Io::NUM_PORTS ||
      goal.error_io_trigger_pin >= Io::NUM_PINS ||
      goal.next_motion >= Goal::MOTION_MAX_NUMBER)
  {
    ActionStraight::resetSpeed();
    nav.base_.stop();
    return boost::shared_ptr<ActionProcessor>();
  }

  switch (goal.nav)
  {
  case Goal::NAV_FORWARD:
  case Goal::NAV_REVERSE:
    return boost::make_shared<ActionStraight>(boost::ref(nav), goal);

  case Goal::NAV_BEZIER_FORWARD:
  case Goal::NAV_BEZIER_REVERSE:
    return boost::make_shared<ActionBezier>(boost::ref(nav), goal);

  case Goal::NAV_FORWARD_DOCK:
  case Goal::NAV_REVERSE_DOCK:
  case Goal::NAV_FORWARD_UNDOCK:
  case Goal::NAV_REVERSE_UNDOCK:
    return boost::make_shared<ActionDock>(boost::ref(nav), goal);

  case Goal::NAV_ROTATE_LEFT:
  case Goal::NAV_ROTATE_RIGHT:
    return goal.goal_tolerance ? boost::make_shared<ActionTurnOmni>(boost::ref(nav), goal) :
                                 boost::make_shared<ActionTurn>(boost::ref(nav), goal);

  case Goal::NAV_SELECT_NAV_PROFILE1
      ... Goal::NAV_SELECT_NAV_PROFILE5:
    return boost::make_shared<ActionSelectNavProfile>(boost::ref(nav), goal);

  case Goal::NAV_SELECT_LASER_PROFILE1
      ... Goal::NAV_SELECT_LASER_PROFILE10:
    return boost::make_shared<ActionSelectLaserProfile>(boost::ref(nav), goal);

  case Goal::NAV_DYNAMIC_FORWARD:
  case Goal::NAV_DYNAMIC_REVERSE:
  case Goal::NAV_DYNAMIC_LINE_FORWARD:
  case Goal::NAV_DYNAMIC_LINE_REVERSE:
  case Goal::NAV_DYNAMIC_BEZIER_FORWARD:
  case Goal::NAV_DYNAMIC_BEZIER_REVERSE:
    return boost::make_shared<ActionDynamic>(boost::ref(nav), goal);

  case Goal::NAV_OMNI:
    return boost::make_shared<ActionOmni>(boost::ref(nav), goal);

  case Goal::NAV_BEZIER_OMNI:
    return boost::make_shared<ActionBezierOmni>(boost::ref(nav), goal);

  case Goal::NAV_OMNI_DOCK:
    return boost::make_shared<ActionDockOmni>(boost::ref(nav), goal);

  case Goal::NAV_OMNI_UNDOCK:
    return boost::make_shared<ActionUndockOmni>(boost::ref(nav), goal);

  case Goal::NAV_DYNAMIC_OMNI:
  case Goal::NAV_DYNAMIC_LINE_OMNI:
  case Goal::NAV_DYNAMIC_BEZIER_OMNI:
    return boost::make_shared<ActionDynamicOmni>(boost::ref(nav), goal);

  default:
    ROS_ASSERT_MSG(0, "Invalid goal.");
    break;
  }

  return boost::shared_ptr<ActionProcessor>();
}

void ActionProcessor::process(float frequency, Nav& nav)
{
  switch (nav.laser_sensor_.stopDelay())
  {
  case LaserSensor::STOP_DELAY_OFF:
    break;
  case LaserSensor::STOP_DELAY_ON:
    {
      agv05_msgs::ObstacleSensor activation = nav.laser_sensor_.getActivation();
      nav.safety_.publishNavTrigger(activation.near_blocked, activation.hint);
      nav.panel_.update(activation.near_blocked ? Feedback::STATUS_OBSTACLE_BLOCKED : Feedback::STATUS_NORMAL);
    }
    break;
  case LaserSensor::STOP_DELAY_DONE:
    ROS_ERROR("Forward flag is not resumed by moving action.");
    completeNav(nav);
    break;
  default:
    break;
  }
}

void ActionProcessor::handleNavControl(const NavControl& nav_control)
{
  switch (nav_control.control)
  {
  case NavControl::CONTROL_PAUSE:
    paused_ = true;
    break;
  case NavControl::CONTROL_CONTINUE:
    paused_ = false;
    break;
  case NavControl::CONTROL_ABORT:
    aborted_ = true;
    break;
  case NavControl::CONTROL_SAFETY_RESUME:
    safety_status_ = Feedback::STATUS_NORMAL;
    break;
  }
}

// publish functions
//===============================================================================

void ActionProcessor::publishStatus(uint8_t status, uint8_t led_side)
{
  // publish to led blink according to status
  panel_.update(status, led_side);

  status_ = status;
  status_updated_ = true;
}

void ActionProcessor::completeNav(Nav& nav)
{
  // off safety nav trigger
  nav.safety_.publishNavTrigger(false);
  // off led
  nav.panel_.setLed(agv05_msgs::LedControl::OFF);
  // off alarm
  nav.panel_.setAlarm(false);
  // disable laser sensor
  nav.laser_sensor_.selectArea(agv05_msgs::ObstacleSensorArea::AREA_DISABLE);
}

void ActionProcessor::completeNav(uint8_t result)
{
  if (forward_flag_)
  {
    // 2s is the maximum value of agv05_motor velocity_timeout
    laser_sensor_.stopDelay(2.0);
  }
  else
  {
    completeNav(nav_);
  }

  completeAction(result);
}

void ActionProcessor::completeAction(uint8_t result)
{
  result_ = result;
  completed_ = true;
}

// helper functions
//===============================================================================

uint8_t ActionProcessor::checkSafetyStatus(bool enable_sensor, bool resume, bool trigger_navigation_failed, UseLineSensor use_line_sensor)
{
  agv05_msgs::SafetyTriggers st = safety_.getSafetyTrigger();
  agv05_msgs::LineSensor lsf = line_sensor_.getFrontData();
  agv05_msgs::LineSensor lsr = line_sensor_.getRearData();
  agv05_msgs::ObstacleSensor activation = laser_sensor_.getActivation();

  safety_internal_message_ = "";

  if (st.emergency_button)
  {
    safety_status_ = Feedback::STATUS_EMERGENCY_BUTTON_PRESSED;
  }
  else if (st.bumper_front || st.bumper_rear)
  {
    safety_internal_message_ = st.bumper_front && st.bumper_rear ? "Front & Rear" :
                               st.bumper_front ? "Front" : "Rear";
    safety_status_ = Feedback::STATUS_BUMPER_BLOCKED;
  }
  else if (st.safety_in_1 || st.safety_in_2)
  {
    if (st.safety_in_1 && !config_.safety_in_1_auto_resume ||
        st.safety_in_2 && !config_.safety_in_2_auto_resume)
    {
      safety_status_ = Feedback::STATUS_EXTERNAL_SAFETY_TRIGGER;
    }
    else
    {
      return Feedback::STATUS_EXTERNAL_SAFETY_TRIGGER;
    }
  }
  else if (activation.malfunction)
  {
    safety_internal_message_ = activation.hint;
    safety_status_ = Feedback::STATUS_LASER_MALFUNCTION;
  }
  else if (enable_sensor && activation.near_blocked &&
           (!config_.safety_triggered_disable_obstacle || safety_status_ != Feedback::STATUS_SAFETY_TRIGGERED))
  {
    safety_internal_message_ = activation.hint;

    // temporary trigger so don't modify safety_status_
    return Feedback::STATUS_OBSTACLE_BLOCKED;
  }
  else if (((!lsf.enable || lsf.sensor_error) && (use_line_sensor == LINE_SENSOR_FRONT)) ||
           ((!lsr.enable || lsr.sensor_error) && (use_line_sensor == LINE_SENSOR_REAR)))
  {
    agv05_msgs::LineSensor& ls = use_line_sensor == LINE_SENSOR_FRONT ? lsf : lsr;
    safety_internal_message_ = use_line_sensor == LINE_SENSOR_FRONT ? "Front " : "Rear ";
    if (!ls.enable)
    {
      safety_internal_message_ += "Sensor Disabled";
    }
    else
    {
      switch (ls.sensor_error)
      {
      case agv05_msgs::LineSensor::COMMUNICATION_ERROR:
        safety_internal_message_ += "Communication Error";
        break;
      case agv05_msgs::LineSensor::CALIBRATION_ERROR:
        safety_internal_message_ += "Calibration Error";
        break;
      default:
        safety_internal_message_ += "Unknown Error: ";
        safety_internal_message_ += std::to_string(ls.sensor_error);
        break;
      }
    }
    safety_status_ = Feedback::STATUS_LINE_SENSOR_ERROR;
  }
  else if (trigger_navigation_failed)
  {
    safety_status_ = Feedback::STATUS_NAVIGATION_FAILED;
  }
  else if (st.motor_fault)
  {
    safety_internal_message_ = safety_.getMotorFaultHint();
    safety_status_ = Feedback::STATUS_MOTOR_FAULT;
  }
  else if (st.wheel_slippage)
  {
    safety_status_ = Feedback::STATUS_WHEEL_SLIPPAGE;
  }
  else if (st.charger_connected)
  {
    safety_status_ = Feedback::STATUS_CHARGER_CONNECTED;
  }
  else if (st.system_error)
  {
    safety_internal_message_ = safety_.getSafetySystemHint();
    safety_status_ = Feedback::STATUS_SYSTEM_ERROR;
  }
  else if (safety_status_ != Feedback::STATUS_NORMAL && !resume)
  {
    switch (safety_status_)
    {
    case Feedback::STATUS_PLAN_EMPTY:
      break;
    default:
      if (!isStatusUpdated())
      {
        // change to 'safety triggered' only after the action server has
        // acknowledged and published the existing status.
        safety_status_ = Feedback::STATUS_SAFETY_TRIGGERED;
      }
      break;
    }
  }
  else
  {
    safety_status_ = Feedback::STATUS_NORMAL;
  }
  return safety_status_;
}

}  // namespace agv05

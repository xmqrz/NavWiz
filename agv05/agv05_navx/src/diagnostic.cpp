/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Authors: Patrick Chin
 */

#include "agv05_navx/nav.h"


namespace agv05
{

Diagnostic::Diagnostic(ros::NodeHandle& nh, BaseX& base, NavConfig& config):
  expected_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_,
                        &expected_frequency_, 0.1, 1), "Process Frequency"),
  base_(base),
  config_(config),
  nav_(Goal::NAV_IDLE),
  status_(Feedback::STATUS_NORMAL),
  overshoot_(0.0)
{
  updater_.setHardwareID("AGV05");
  updater_.add(diagnostic_frequency_);
  updater_.add("Status", this, &Diagnostic::statusDiagnostic);
}

void Diagnostic::statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::string nav_text;
  std::ostringstream oss;
  switch (nav_)
  {
  case Goal::NAV_IDLE:
    nav_text = "Idle";
    break;
  case Goal::NAV_FORWARD:
    nav_text = "Forward";
    break;
  case Goal::NAV_REVERSE:
    nav_text = "Reverse";
    break;
  case Goal::NAV_BEZIER_FORWARD:
    nav_text = "Bezier Forward";
    break;
  case Goal::NAV_BEZIER_REVERSE:
    nav_text = "Bezier Reverse";
    break;
  case Goal::NAV_FORWARD_DOCK:
    nav_text = "Forward Dock";
    break;
  case Goal::NAV_REVERSE_DOCK:
    nav_text = "Reverse Dock";
    break;
  case Goal::NAV_FORWARD_UNDOCK:
    nav_text = "Forward Undock";
    break;
  case Goal::NAV_REVERSE_UNDOCK:
    nav_text = "Reverse Undock";
    break;
  case Goal::NAV_ROTATE_LEFT:
    nav_text = "Rotate left";
    break;
  case Goal::NAV_ROTATE_RIGHT:
    nav_text = "Rotate right";
    break;
  case Goal::NAV_SELECT_NAV_PROFILE1
      ... Goal::NAV_SELECT_NAV_PROFILE5:
    oss << "Select nav profile " << nav_ - Goal::NAV_SELECT_NAV_PROFILE1 + 1;
    nav_text = oss.str();
    break;
  case Goal::NAV_SELECT_LASER_PROFILE1
      ... Goal::NAV_SELECT_LASER_PROFILE10:
    oss << "Select laser profile " << nav_ - Goal::NAV_SELECT_LASER_PROFILE1 + 1;
    nav_text = oss.str();
    break;
  case Goal::NAV_DYNAMIC_FORWARD:
    nav_text = "Dynamic Forward";
    break;
  case Goal::NAV_DYNAMIC_REVERSE:
    nav_text = "Dynamic Reverse";
    break;
  case Goal::NAV_DYNAMIC_LINE_FORWARD:
    nav_text = "Dynamic Line Forward";
    break;
  case Goal::NAV_DYNAMIC_LINE_REVERSE:
    nav_text = "Dynamic Line Reverse";
    break;
  case Goal::NAV_DYNAMIC_BEZIER_FORWARD:
    nav_text = "Dynamic Bezier Forward";
    break;
  case Goal::NAV_DYNAMIC_BEZIER_REVERSE:
    nav_text = "Dynamic Bezier Reverse";
    break;
  case Goal::NAV_OMNI:
    nav_text = "Omni";
    break;
  case Goal::NAV_BEZIER_OMNI:
    nav_text = "Bezier Omni";
    break;
  case Goal::NAV_OMNI_DOCK:
    nav_text = "Omni Dock";
    break;
  case Goal::NAV_OMNI_UNDOCK:
    nav_text = "Omni Undock";
    break;
  case Goal::NAV_DYNAMIC_OMNI:
    nav_text = "Dynamic Omni";
    break;
  case Goal::NAV_DYNAMIC_LINE_OMNI:
    nav_text = "Dynamic Line Omni";
    break;
  case Goal::NAV_DYNAMIC_BEZIER_OMNI:
    nav_text = "Dynamic Bezier Omni";
    break;
  default:
    oss << "Unknown (" << nav_ << ")";
    nav_text = oss.str();
    break;
  }

  std::string status_text;
  switch (status_)
  {
  case Feedback::STATUS_NORMAL:
    status_text = "Normal";
    break;
  case Feedback::STATUS_PAUSED:
    status_text = "Paused";
    break;
  case Feedback::STATUS_OUT_OF_LINE:
    status_text = "Out of line";
    break;
  case Feedback::STATUS_BUMPER_BLOCKED:
    status_text = "Bumper blocked";
    break;
  case Feedback::STATUS_EXTERNAL_SAFETY_TRIGGER:
    status_text = "External safety triggered";
    break;
  case Feedback::STATUS_EMERGENCY_BUTTON_PRESSED:
    status_text = "Emergency button pressed";
    break;
  case Feedback::STATUS_CHARGER_CONNECTED:
    status_text = "Charger connected";
    break;
  case Feedback::STATUS_LASER_MALFUNCTION:
    status_text = "Laser malfunction";
    break;
  case Feedback::STATUS_LINE_SENSOR_ERROR:
    status_text = "Line sensor error";
    break;
  case Feedback::STATUS_DRIVE_OVERLIMIT_ERROR:
    status_text = "Drive overlimit error";
    break;
  case Feedback::STATUS_MOTOR_FAULT:
    status_text = "Motor fault";
    break;
  case Feedback::STATUS_WHEEL_SLIPPAGE:
    status_text = "Wheel slippage";
    break;
  case Feedback::STATUS_SYSTEM_ERROR:
    status_text = "System error";
    break;
  case Feedback::STATUS_NAVIGATION_FAILED:
    status_text = "Navigation failed";
    break;
  case Feedback::STATUS_OBSTACLE_BLOCKED:
    status_text = "Obstacle blocked";
    break;
  case Feedback::STATUS_OBSTACLE_IN_RANGE:
    status_text = "Obstacle in range";
    break;
  case Feedback::STATUS_PLAN_EMPTY:
    status_text = "Path blocked";
    break;
  case Feedback::STATUS_SAFETY_TRIGGERED:
    status_text = "Safety triggered";
    break;
  default:
    oss.str("");
    oss << "Unknown (" << status_ << ")";
    status_text = oss.str();
    break;
  }

  stat.add("NavigationX", nav_text);
  stat.add("NavigationX Status", status_text);
  stat.add("NavigationX Profile", 1 + config_.getProfileIndex());
  stat.add("X", pose_.x);
  stat.add("Y", pose_.y);
  stat.add("Theta", pose_.theta);
  stat.add("Overshoot", overshoot_);
  stat.add("Linear Error", linear_error_);
  stat.add("Angular Error", angular_error_);
  stat.add("Heading Error", heading_error_);

  if (base_.diagPlanner())
  {
    stat.add("Costmap Duration", base_.getCostmapSec());
    stat.add("Make Plan Duration", base_.getMakePlanSec());
  }

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status OK");
}

}  // namespace agv05

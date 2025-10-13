/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/nav.h"


namespace agv05
{

Diagnostic::Diagnostic(ros::NodeHandle& nh, NavConfig& config):
  expected_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_,
                        &expected_frequency_, 0.1, 1), "Process Frequency"),
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
  case Goal::NAV_FORWARD_LEFT:
    nav_text = "Forward left";
    break;
  case Goal::NAV_FORWARD_RIGHT:
    nav_text = "Forward right";
    break;
  case Goal::NAV_REVERSE:
    nav_text = "Reverse";
    break;
  case Goal::NAV_REVERSE_LEFT:
    nav_text = "Reverse left";
    break;
  case Goal::NAV_REVERSE_RIGHT:
    nav_text = "Reverse right";
    break;
  case Goal::NAV_BEZIER_FORWARD:
    nav_text = "Bezier Forward";
    break;
  case Goal::NAV_BEZIER_FORWARD_LEFT:
    nav_text = "Bezier Forward left";
    break;
  case Goal::NAV_BEZIER_FORWARD_RIGHT:
    nav_text = "Bezier Forward right";
    break;
  case Goal::NAV_BEZIER_REVERSE:
    nav_text = "Bezier Reverse";
    break;
  case Goal::NAV_BEZIER_REVERSE_LEFT:
    nav_text = "Bezier Reverse left";
    break;
  case Goal::NAV_BEZIER_REVERSE_RIGHT:
    nav_text = "Bezier Reverse right";
    break;
  case Goal::NAV_ROTATE_LEFT:
    nav_text = "Rotate left";
    break;
  case Goal::NAV_ROTATE_RIGHT:
    nav_text = "Rotate right";
    break;
  case Goal::NAV_UTURN_LEFT:
    nav_text = "U-turn left";
    break;
  case Goal::NAV_UTURN_RIGHT:
    nav_text = "U-turn right";
    break;
  case Goal::NAV_ROTATE3Q_LEFT:
    nav_text = "Rotate three-quarter left";
    break;
  case Goal::NAV_ROTATE3Q_RIGHT:
    nav_text = "Rotate three-quarter right";
    break;
  case Goal::NAV_SEARCH_LINE_LEFT:
    nav_text = "Search line left";
    break;
  case Goal::NAV_SEARCH_LINE_RIGHT:
    nav_text = "Search line right";
    break;
  case Goal::NAV_MANUAL_CONTROL:
    nav_text = "Manual control";
    break;
  case Goal::NAV_LINE_CALIBRATE:
    nav_text = "Line calibration";
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
  case Goal::NAV_SCAN_LASER_AREA1
      ... Goal::NAV_SCAN_LASER_AREA31:
    oss << "Scan laser area " << nav_ - Goal::NAV_SCAN_LASER_AREA1 + 1;
    nav_text = oss.str();
    break;
  case Goal::NAV_FREE_MOTOR:
    nav_text = "Free motor";
    break;
  case Goal::NAV_WAIT_TRAFFIC:
    nav_text = "Wait traffic";
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
  case Feedback::STATUS_OBSTACLE_BLOCKED:
    status_text = "Obstacle blocked";
    break;
  case Feedback::STATUS_OBSTACLE_IN_RANGE:
    status_text = "Obstacle in range";
    break;
  case Feedback::STATUS_WAIT_TRAFFIC:
    status_text = "Wait traffic";
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

  stat.add("Navigation", nav_text);
  stat.add("Navigation Status", status_text);
  stat.add("Navigation Profile", 1 + config_.getProfileIndex());
  stat.add("Overshoot", overshoot_);

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status OK");
}

}  // namespace agv05

/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_MOTOR_CONTROLLERS_TRICYCLE_DRIVE_H
#define AGV05_MOTOR_CONTROLLERS_TRICYCLE_DRIVE_H

#include <agv05_motor/controller.h>

#include <agv05_msgs/SteeringControl.h>
#include <agv05_msgs/SteeringFeedback.h>


namespace agv05
{

class TricycleDrive: public Controller
{
public:
  TricycleDrive();

  // dynamic reconfigure
  void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update = false);

protected:
  // timer for speed smoothening
  void ctrlCallback(bool enable, float period = 0.0f, const std::vector< std::complex<double> >& speed = {});

  // drive feedback
  void callbackFeedback(const agv05_msgs::SteeringFeedback& msg);

  // latest output
  std::vector< std::complex<double> > getOutput(bool max)
  {
    if (max)
    {
      return {max_speed_};
    }
    return {std::polar<double>(output_.drive_speed * config_.drive_motor_calibration, output_.steering_angle)};
  }

  // diagnostic
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

protected:
  /* ROS publishers */
  ros::Publisher output_pub_;

  /* ROS subscribers */
  ros::Subscriber feedback_sub_;

  /* Data inputs */
  agv05_msgs::SteeringFeedback feedback_;

  float max_steer_;

  /* Data outputs */
  agv05_msgs::SteeringControl output_;

  std_msgs::Bool steering_align_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_CONTROLLERS_TRICYCLE_DRIVE_H

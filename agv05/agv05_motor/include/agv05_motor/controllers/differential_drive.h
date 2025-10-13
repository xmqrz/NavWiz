/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_MOTOR_CONTROLLERS_DIFFERENTIAL_DRIVE_H
#define AGV05_MOTOR_CONTROLLERS_DIFFERENTIAL_DRIVE_H

#include <agv05_motor/controller.h>

#include <agv05_msgs/MotorControl.h>
#include <agv05_msgs/MotorFeedback.h>


namespace agv05
{

class DifferentialDrive: public Controller
{
public:
  DifferentialDrive();

  // dynamic reconfigure
  void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update = false);

protected:
  // timer for speed smoothening
  void ctrlCallback(bool enable, float period = 0.0f, const std::vector< std::complex<double> >& speed = {});

  // drive feedback
  void callbackFeedback(const agv05_msgs::MotorFeedback& msg);

  // latest output
  std::vector< std::complex<double> > getOutput(bool max)
  {
    if (max)
    {
      return {max_speed_, max_speed_};
    }
    return {
      output_.left_speed * config_.left_motor_calibration,
      output_.right_speed * config_.right_motor_calibration
    };
  }

  // diagnostic
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

protected:
  /* ROS publishers */
  ros::Publisher output_pub_;

  /* ROS subscribers */
  ros::Subscriber feedback_sub_;

  /* Data inputs */
  Feedback<agv05_msgs::MotorFeedback> feedback_;

  /* Data outputs */
  agv05_msgs::MotorControl output_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_CONTROLLERS_DIFFERENTIAL_DRIVE_H

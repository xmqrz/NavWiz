/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef AGV05_MOTOR_CONTROLLERS_MECANUM_DRIVE_H
#define AGV05_MOTOR_CONTROLLERS_MECANUM_DRIVE_H

#include <agv05_motor/controller.h>

#include <agv05_msgs/MecanumControl.h>
#include <agv05_msgs/MecanumFeedback.h>


namespace agv05
{

class MecanumDrive: public Controller
{
public:
  MecanumDrive();

  // dynamic reconfigure
  void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update = false);

protected:
  // timer for speed smoothening
  void ctrlCallback(bool enable, float period = 0.0f, const std::vector< std::complex<double> >& speed = {});

  // drive feedback
  void callbackFeedback(const agv05_msgs::MecanumFeedback& msg);

  // latest output
  std::vector< std::complex<double> > getOutput(bool max)
  {
    if (max)
    {
      return {
        std::complex<double>(max_speed_, max_speed_),
        std::complex<double>(max_speed_, max_speed_)
      };
    }

    double left_front_speed = output_.left_front_speed * config_.left_front_motor_calibration;
    double left_rear_speed = output_.left_rear_speed * config_.left_rear_motor_calibration;
    double right_rear_speed = output_.right_rear_speed * config_.right_rear_motor_calibration;
    double right_front_speed = output_.right_front_speed * config_.right_front_motor_calibration;
    return {
      std::complex<double>(left_rear_speed + left_front_speed, left_rear_speed - left_front_speed) * 0.5,
      std::complex<double>(right_front_speed + right_rear_speed, right_front_speed - right_rear_speed) * 0.5,
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
  Feedback<agv05_msgs::MecanumFeedback> feedback_;

  /* Data outputs */
  agv05_msgs::MecanumControl output_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_CONTROLLERS_MECANUM_DRIVE_H

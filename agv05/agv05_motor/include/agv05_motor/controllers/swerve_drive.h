/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef AGV05_MOTOR_CONTROLLERS_SWERVE_DRIVE_H
#define AGV05_MOTOR_CONTROLLERS_SWERVE_DRIVE_H

#include <agv05_motor/controller.h>

#include <agv05_msgs/SwerveControl.h>
#include <agv05_msgs/SwerveFeedback.h>


namespace agv05
{

class SwerveDrive: public Controller
{
public:
  SwerveDrive();

  // dynamic reconfigure
  void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update = false);

protected:
  // timer for speed smoothening
  float alignSteer(float front_angle, float rear_angle);
  bool ctrlSteer(float& angle, float angle0, float min_steer, float max_steer);
  void ctrlCallback(bool enable, float period = 0.0f, const std::vector< std::complex<double> >& speed = {});

  // drive feedback
  void callbackFeedback(const agv05_msgs::SwerveFeedback& msg);

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

    return {
      std::polar<double>(output_.front_speed * config_.front_motor_calibration, output_.front_angle),
      std::polar<double>(output_.rear_speed * config_.rear_motor_calibration, output_.rear_angle),
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
  agv05_msgs::SwerveFeedback feedback_;

  float front_min_steer_;
  float front_max_steer_;
  float rear_min_steer_;
  float rear_max_steer_;

  /* Data outputs */
  agv05_msgs::SwerveControl output_;

  std_msgs::Bool steering_align_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_CONTROLLERS_SWERVE_DRIVE_H

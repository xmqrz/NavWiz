/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <agv05_motor/controllers/differential_drive.h>
#include <angles/angles.h>


namespace agv05
{

DifferentialDrive::DifferentialDrive() :
  Controller(2)
{
  ros::NodeHandle nh_motor("agv05/motor");

  // ROS publishers
  output_pub_ = nh_motor.advertise<agv05_msgs::MotorControl>("control", 1, true);

  // ROS subscribers
  feedback_sub_ = nh_motor.subscribe("feedback", 10, &DifferentialDrive::callbackFeedback, this);

  // Initial publish
  std_msgs::Bool msg;
  msg.data = true;
  steering_align_pub_.publish(msg);
}

void DifferentialDrive::callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update)
{
  if (force_update || config.track_width != config_.track_width)
  {
    std::complex<double> l(0.0, config.track_width * 0.5);
    Lock lock(mutex_);
    kinematic_.setup({l, -l});
    angular_speed_ratio_ = 1.0 / std::abs(kinematic_.control(0.0, 1.0)[0].real());
  }
  Controller::callbackConfig(config, level, force_update);
}

void DifferentialDrive::ctrlCallback(bool enable, float period, const std::vector< std::complex<double> >& speed)
{
  output_.enable = enable;

  // safety trigger
  if (speed.size() != wheels_.size())
  {
    output_.left_speed = 0.0f;
    output_.right_speed = 0.0f;
  }
  // navigation enabled
  else
  {
    // compute left and right speed
    float left_speed = speed[0].real() / config_.left_motor_calibration;
    float right_speed = speed[1].real() / config_.right_motor_calibration;

    // limit motor to max speed
    float d_max = std::max(left_speed, right_speed) - max_speed_;
    if (d_max > 0)
    {
      if (left_speed > d_max) left_speed -= d_max;
      else if (left_speed < -d_max) left_speed += d_max;
      else left_speed = 0;

      if (right_speed > d_max) right_speed -= d_max;
      else if (right_speed < -d_max) right_speed += d_max;
      else right_speed = 0;
    }
    float d_min = std::min(left_speed, right_speed) + max_speed_;
    if (d_min < 0)
    {
      if (left_speed < d_min) left_speed -= d_min;
      else if (left_speed > -d_min) left_speed += d_min;
      else left_speed = 0;

      if (right_speed < d_min) right_speed -= d_min;
      else if (right_speed > -d_min) right_speed += d_min;
      else right_speed = 0;
    }

    // accelerate or decelerate
    output_.left_speed = wheels_[0].control(left_speed, period);
    output_.right_speed = wheels_[1].control(right_speed, period);
  }

  static agv05_msgs::MotorControl output;
  ROS_DEBUG_COND(
    (output.left_speed != output_.left_speed ||
     output.right_speed != output_.right_speed ||
     output.enable != output_.enable) && (output = output_, true),
    "Control: %f %f %d", output_.left_speed, output_.right_speed, output_.enable);

  // publish outputs
  output_pub_.publish(output_);
}

void DifferentialDrive::callbackFeedback(const agv05_msgs::MotorFeedback& msg)
{
  if (msg.stamp.isZero())
  {
    agv05_msgs::MotorFeedback m = msg;
    m.stamp = ros::Time::now();
    if (!m.stamp.isZero())
    {
      callbackFeedback(m);
    }
    return;
  }

  const ros::Time& now = msg.stamp;

  if (feedback_.setup(msg))
  {
    return;
  }

  const agv05_msgs::MotorFeedback& feedback = feedback_(config_.motor_feedback_window);
  double rate = 0.0;
  if (now > feedback.stamp)
  {
    rate = std::min(1.0 / (now - feedback.stamp).toSec(), LOOP_FREQUENCY);
  }

  // compute odom
  double left_diff = (msg.left_distance - feedback.left_distance) * config_.left_motor_calibration;
  double right_diff = (msg.right_distance - feedback.right_distance) * config_.right_motor_calibration;
  double left_speed = left_diff * rate;
  double right_speed = right_diff * rate;

  if (config_.motor_feedback_window > 1)
  {
    left_diff /= config_.motor_feedback_window;
    right_diff /= config_.motor_feedback_window;
  }

  double dist = std::abs(left_diff) + std::abs(right_diff);
  if (dist > 2)
  {
    ROS_ERROR("Huge odom difference: %f: %f %f, %f %f, %f %f", dist, left_diff, right_diff,
              msg.left_distance, msg.right_distance, feedback.left_distance, feedback.right_distance);
    feedback_ = msg;
    return;
  }

  if (config_.motor_feedback_window < (FEEDBACK_WINDOW_SIZE_MAX >> 1))
  {
    const agv05_msgs::MotorFeedback& feedback_speed = feedback_(FEEDBACK_WINDOW_SIZE_MAX >> 1);
    if (now > feedback_speed.stamp)
    {
      rate = std::min(1.0 / (now - feedback_speed.stamp).toSec(), LOOP_FREQUENCY);
      left_speed = (msg.left_distance - feedback_speed.left_distance) * config_.left_motor_calibration * rate;
      right_speed = (msg.right_distance - feedback_speed.right_distance) * config_.right_motor_calibration * rate;
    }
  }

  updateOdom(now, {left_speed, right_speed}, {left_diff, right_diff}, dist * 0.5);

  // motor fault detection
  std::ostringstream oss("", std::ostringstream::ate);
  oss << std::hex << std::setfill('0') << std::uppercase;
  if (msg.left_error)
  {
    oss << "Left (0x" << std::setw(sizeof(msg.left_error) << 1) << msg.left_error << ") ";
  }
  if (msg.right_error)
  {
    oss << "Right (0x" << std::setw(sizeof(msg.right_error) << 1) << msg.right_error << ") ";
  }

  wheels_[0].feedback(now, left_speed);
  wheels_[1].feedback(now, right_speed);
  Controller::callbackFeedback(now, msg.enabled, oss.str());

  // store feedback
  feedback_ = msg;
  diagnostic_frequency_.tick();
}

void DifferentialDrive::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  const agv05_msgs::MotorFeedback& feedback = feedback_();

  stat.add("Drive Type", "Differential Drive");
  stat.add("Motor Enable", static_cast<bool>(output_.enable));

  stat.addf("Motor Left Command Speed (m/s)", "%.3f", output_.left_speed);
  stat.addf("Motor Left Actual Speed (m/s)", "%.3f", wheels_[0].feedback_);
  stat.addf("Motor Left Speed Index", "%.3f", wheels_[0].index_);
  stat.addf("Motor Left Speed Error", "%.3f", wheels_[0].error_);
  stat.addf("Motor Left Distance (m)", "%.3f", feedback.left_distance);
  stat.addf("Motor Left Current (A)", "%.3f", feedback.left_current);
  stat.addf("Motor Left Temperature (°C)", "%.2f", feedback.left_temperature);
  stat.add("Motor Left Error Code", feedback.left_error);

  stat.addf("Motor Right Command Speed (m/s)", "%.3f", output_.right_speed);
  stat.addf("Motor Right Actual Speed (m/s)", "%.3f", wheels_[1].feedback_);
  stat.addf("Motor Right Speed Index", "%.3f", wheels_[1].index_);
  stat.addf("Motor Right Speed Error", "%.3f", wheels_[1].error_);
  stat.addf("Motor Right Distance (m)", "%.3f", feedback.right_distance);
  stat.addf("Motor Right Current (A)", "%.3f", feedback.right_current);
  stat.addf("Motor Right Temperature (°C)", "%.2f", feedback.right_temperature);
  stat.add("Motor Right Error Code", feedback.right_error);

  Controller::diagnosticStatus(stat);
}

}  // namespace agv05

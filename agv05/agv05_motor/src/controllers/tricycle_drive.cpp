/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <agv05_motor/controllers/tricycle_drive.h>
#include <angles/angles.h>


namespace agv05
{

TricycleDrive::TricycleDrive() :
  Controller(1)
{
  ros::NodeHandle nh_steering("agv05/motor/steering");

  // ROS publishers
  output_pub_ = nh_steering.advertise<agv05_msgs::SteeringControl>("control", 1, true);

  // ROS subscribers
  feedback_sub_ = nh_steering.subscribe("feedback", 10, &TricycleDrive::callbackFeedback, this);
}

void TricycleDrive::callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update)
{
  if (force_update || config.wheelbase != config_.wheelbase ||
      config.drive_center_offset != config_.drive_center_offset ||
      config.steer_angle_turn != config_.steer_angle_turn)
  {
    double offset = config.drive_center_offset;
    if (config.steer_angle_turn && (std::abs(config.steer_angle_turn) < 90.0))
    {
      offset -= config.wheelbase / std::tan(config.steer_angle_turn * M_PI / 180.0);
    }

    Lock lock(mutex_);
    kinematic_.setup({std::complex<double>(config.wheelbase, offset)});
    angular_speed_ratio_ = 1.0 / std::abs(kinematic_.control(0.0, 1.0)[0]);
    max_steer_ = std::atan2(std::abs(config.wheelbase), std::abs(offset));
    if (config.steer_angle_turn)
    {
      max_steer_ = std::min<float>(max_steer_, std::abs(config.steer_angle_turn));
    }
  }
  Controller::callbackConfig(config, level, force_update);
}

void TricycleDrive::ctrlCallback(bool enable, float period, const std::vector< std::complex<double> >& speed)
{
  output_.enable = enable;

  // safety trigger
  if (speed.size() != wheels_.size())
  {
    output_.drive_speed = 0.0f;

    steering_align_.data = false;
  }
  // navigation enabled
  else
  {
    // compute drive speed and steering angle
    float drive_speed = 0.0f;
    float steering_angle = 0.0f;

    const std::complex<double>& v = speed[0];
    // steering_angle = std::arg(v);
    // drive_speed = std::abs(v);
    if (v.real())
    {
      steering_angle = std::atan(v.imag() / v.real());  // -M_PI_2 < steering_angle < M_PI_2
      drive_speed = v.real() / std::cos(steering_angle);  // std::cos(steering_angle) > 0

      if (drive_speed * velocity_.linear.x < 0.0)
      {
        // prioritize angular speed
        steering_angle = -steering_angle;
        drive_speed = -drive_speed;
      }
    }
    else if (v.imag())
    {
      steering_angle = M_PI_2;
      drive_speed = v.imag();

      if (config_.steer_angle_turn > 0)
      {
        // use the given steer angle to turn
      }
      else if (config_.steer_angle_turn < 0 || feedback_.steering_angle < 0)
      {
        steering_angle = -steering_angle;
        drive_speed = -drive_speed;
      }
    }

    // hysteresis for steering align state
    if (drive_speed)
    {
      float threshold = (velocity_.linear.x == 0) ? 1 * M_PI / 180 :
                        steering_align_.data ? max_steer_ * 2 :
                        std::max(config_.steer_angle_align > 0 ? config_.steer_angle_align * M_PI / 180 :
                                 std::abs(config_.steer_angle_align * steering_angle), 1 * M_PI / 180);
      float error = std::abs(steering_angle - feedback_.steering_angle);
      steering_align_.data = error <= threshold;
      error = std::min(error, max_steer_);
      drive_speed = steering_align_.data ? drive_speed * std::cos(error) / config_.drive_motor_calibration : 0.0f;
      if (!steering_angle && !drive_speed)
      {
        steering_angle = FLT_MIN;
      }
    }
    else  // both linear and angular are zero
    {
      steering_align_.data = false;
    }

    // limit motor to max speed
    if (drive_speed > max_speed_)
    {
      drive_speed = max_speed_;
    }
    else if (drive_speed < -max_speed_)
    {
      drive_speed = -max_speed_;
    }
    // limit steering angle
    if (steering_angle > M_PI_2)
    {
      steering_angle = M_PI_2;
    }
    else if (steering_angle < -M_PI_2)
    {
      steering_angle = -M_PI_2;
    }

    // accelerate or decelerate
    output_.drive_speed = wheels_[0].control(drive_speed, period);
    output_.steering_angle = steering_angle;
  }

  static agv05_msgs::SteeringControl output;
  ROS_DEBUG_COND(
    (output.drive_speed != output_.drive_speed ||
     output.steering_angle != output_.steering_angle ||
     output.enable != output_.enable) && (output = output_, true),
    "Control: %f %f %d", output_.drive_speed, output_.steering_angle, output_.enable);

  // publish outputs
  output_pub_.publish(output_);
  steering_align_pub_.publish(steering_align_);
}

void TricycleDrive::callbackFeedback(const agv05_msgs::SteeringFeedback& msg)
{
  if (msg.stamp.isZero())
  {
    agv05_msgs::SteeringFeedback m = msg;
    m.stamp = ros::Time::now();
    if (!m.stamp.isZero())
    {
      callbackFeedback(m);
    }
    return;
  }

  const ros::Time& now = msg.stamp;

  if (feedback_.stamp.isZero())
  {
    feedback_ = msg;
    return;
  }

  // compute odom
  double period = (now > feedback_.stamp) ? (now - feedback_.stamp).toSec() : 0.0;
  double drive_speed = msg.drive_speed * config_.drive_motor_calibration;
  double drive_diff = drive_speed * period;

  double dist = std::abs(drive_diff);
  if (dist > 2)
  {
    ROS_ERROR("Huge odom difference: %f: %f %f", dist, drive_diff, msg.steering_angle);
    feedback_ = msg;
    return;
  }

  updateOdom(now, {std::polar(drive_speed, msg.steering_angle)}, {std::polar(drive_diff, msg.steering_angle)}, dist);

  // motor fault detection
  std::ostringstream oss("", std::ostringstream::ate);
  oss << std::hex << std::setfill('0') << std::uppercase;
  if (msg.motor_error)
  {
    oss << "0x" << std::setw(sizeof(msg.motor_error) << 1) << msg.motor_error;
  }

  wheels_[0].feedback(now, drive_speed);
  Controller::callbackFeedback(now, msg.enabled, oss.str());

  // store feedback
  feedback_ = msg;
  diagnostic_frequency_.tick();
}

void TricycleDrive::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Drive Type", "Tricycle Steering Drive");
  stat.add("Motor Enable", static_cast<bool>(output_.enable));
  stat.add("Steering Align", static_cast<bool>(steering_align_.data));

  stat.addf("Drive Motor Command Speed (m/s)", "%.3f", output_.drive_speed);
  stat.addf("Drive Motor Actual Speed (m/s)", "%.3f", wheels_[0].feedback_);
  stat.addf("Drive Motor Speed Index", "%.3f", wheels_[0].index_);
  stat.addf("Drive Motor Speed Error", "%.3f", wheels_[0].error_);
  stat.addf("Drive Steering Command Angle (°)", "%.3f", angles::to_degrees(output_.steering_angle));
  stat.addf("Drive Steering Actual Angle (°)", "%.3f", angles::to_degrees(feedback_.steering_angle));
  stat.addf("Drive Motor Current (A)", "%.3f", feedback_.motor_current);
  stat.addf("Drive Motor Temperature (°C)", "%.2f", feedback_.motor_temperature);
  stat.add("Drive Motor Error Code", feedback_.motor_error);

  Controller::diagnosticStatus(stat);
}

}  // namespace agv05

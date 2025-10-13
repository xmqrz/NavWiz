/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <agv05_motor/controllers/mecanum_drive.h>
#include <angles/angles.h>


namespace agv05
{

MecanumDrive::MecanumDrive() :
  Controller(4)
{
  ros::NodeHandle nh_motor("agv05/motor/mecanum");

  // ROS publishers
  output_pub_ = nh_motor.advertise<agv05_msgs::MecanumControl>("control", 1, true);

  // ROS subscribers
  feedback_sub_ = nh_motor.subscribe("feedback", 10, &MecanumDrive::callbackFeedback, this);

  // Initial publish
  std_msgs::Bool msg;
  msg.data = true;
  steering_align_pub_.publish(msg);
}

void MecanumDrive::callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update)
{
  if (force_update || config.mecanum_wheelbase != config_.mecanum_wheelbase ||
      config.mecanum_track_width != config_.mecanum_track_width)
  {
    std::complex<double> l(0.0, (config.mecanum_wheelbase + config.mecanum_track_width) * 0.5);
    Lock lock(mutex_);
    kinematic_.setup({l, -l});
    angular_speed_ratio_ = 1.0 / std::abs(kinematic_.control(0.0, 1.0)[0].real());
  }
  Controller::callbackConfig(config, level, force_update);
}

void MecanumDrive::ctrlCallback(bool enable, float period, const std::vector< std::complex<double> >& speed)
{
  output_.enable = enable;

  // safety trigger
  if (speed.size() == 0)
  {
    output_.left_front_speed = 0.0f;
    output_.left_rear_speed = 0.0f;
    output_.right_rear_speed = 0.0f;
    output_.right_front_speed = 0.0f;
  }
  // navigation enabled
  else
  {
    // compute wheels speed
    float left_front_speed = (speed[0].real() - speed[0].imag()) / config_.left_front_motor_calibration;
    float left_rear_speed = (speed[0].real() + speed[0].imag()) / config_.left_rear_motor_calibration;
    float right_rear_speed = (speed[1].real() - speed[1].imag()) / config_.right_rear_motor_calibration;
    float right_front_speed = (speed[1].real() + speed[1].imag()) / config_.right_front_motor_calibration;

    // limit motor to max speed
    float max_speed = std::max(std::max(std::abs(left_front_speed), std::abs(left_rear_speed)),
                               std::max(std::abs(right_front_speed), std::abs(right_rear_speed)));
    if (max_speed_ < max_speed)
    {
      float normalize = max_speed_ / max_speed;
      left_front_speed *= normalize;
      left_rear_speed *= normalize;
      right_rear_speed *= normalize;
      right_front_speed *= normalize;
    }

    // accelerate or decelerate
    output_.left_front_speed = wheels_[0].control(left_front_speed, period);
    output_.left_rear_speed = wheels_[1].control(left_rear_speed, period);
    output_.right_rear_speed = wheels_[2].control(right_rear_speed, period);
    output_.right_front_speed = wheels_[3].control(right_front_speed, period);
  }

  static agv05_msgs::MecanumControl output;
  ROS_DEBUG_COND(
    (output.left_front_speed != output_.left_front_speed ||
     output.left_rear_speed != output_.left_rear_speed ||
     output.right_rear_speed != output_.right_rear_speed ||
     output.right_front_speed != output_.right_front_speed ||
     output.enable != output_.enable) && (output = output_, true),
    "Control: %f %f %f %f %d", output_.left_front_speed, output_.left_rear_speed,
                               output_.right_rear_speed, output_.right_front_speed, output_.enable);

  // publish outputs
  output_pub_.publish(output_);
}

void MecanumDrive::callbackFeedback(const agv05_msgs::MecanumFeedback& msg)
{
  if (msg.stamp.isZero())
  {
    agv05_msgs::MecanumFeedback m = msg;
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

  const agv05_msgs::MecanumFeedback& feedback = feedback_();

  // compute odom
  double left_front_diff = (msg.left_front.distance - feedback.left_front.distance) * config_.left_front_motor_calibration;
  double left_rear_diff = (msg.left_rear.distance - feedback.left_rear.distance) * config_.left_rear_motor_calibration;
  double right_rear_diff = (msg.right_rear.distance - feedback.right_rear.distance) * config_.right_rear_motor_calibration;
  double right_front_diff = (msg.right_front.distance - feedback.right_front.distance) * config_.right_front_motor_calibration;
  double left_front_speed = msg.left_front.speed * config_.left_front_motor_calibration;
  double left_rear_speed = msg.left_rear.speed * config_.left_rear_motor_calibration;
  double right_rear_speed = msg.right_rear.speed * config_.right_rear_motor_calibration;
  double right_front_speed = msg.right_front.speed * config_.right_front_motor_calibration;

  double dist = std::abs(left_front_diff) + std::abs(left_rear_diff) + std::abs(right_rear_diff) + std::abs(right_front_diff);
  if (dist > 2)
  {
    ROS_ERROR("Huge odom difference: %f: %f %f %f %f",
              dist, left_front_diff, left_rear_diff, right_rear_diff, right_front_diff);
    ROS_ERROR("%f %f, %f %f, %f %f, %f %f",
              msg.left_front.distance, feedback.left_front.distance, msg.left_rear.distance, feedback.left_rear.distance,
              msg.right_rear.distance, feedback.right_rear.distance, msg.right_front.distance, feedback.right_front.distance);
    feedback_ = msg;
    return;
  }

  if (!(msg.left_front.speed || msg.left_rear.speed || msg.right_rear.speed || msg.right_front.speed))
  {
    const agv05_msgs::MecanumFeedback& feedback_speed = feedback_(FEEDBACK_WINDOW_SIZE_MAX >> 1);
    if (now > feedback_speed.stamp)
    {
      double rate = std::min(1.0 / (now - feedback_speed.stamp).toSec(), LOOP_FREQUENCY);
      left_front_speed = (msg.left_front.distance - feedback_speed.left_front.distance) * config_.left_front_motor_calibration * rate;
      left_rear_speed = (msg.left_rear.distance - feedback_speed.left_rear.distance) * config_.left_rear_motor_calibration * rate;
      right_rear_speed = (msg.right_rear.distance - feedback_speed.right_rear.distance) * config_.right_rear_motor_calibration * rate;
      right_front_speed = (msg.right_front.distance - feedback_speed.right_front.distance) * config_.right_front_motor_calibration * rate;
    }
  }

  updateOdom(now, {std::complex<double>(left_rear_speed + left_front_speed, left_rear_speed - left_front_speed) * 0.5,
                   std::complex<double>(right_front_speed + right_rear_speed, right_front_speed - right_rear_speed) * 0.5},
                  {std::complex<double>(left_rear_diff + left_front_diff, left_rear_diff - left_front_diff) * 0.5,
                   std::complex<double>(right_front_diff + right_rear_diff, right_front_diff - right_rear_diff) * 0.5}, dist * 0.25);

  // motor fault detection
  std::ostringstream oss("", std::ostringstream::ate);
  oss << std::hex << std::setfill('0') << std::uppercase;
  if (msg.left_front.error)
  {
    oss << "Left Front (0x" << std::setw(sizeof(msg.left_front.error) << 1) << msg.left_front.error << ") ";
  }
  if (msg.left_rear.error)
  {
    oss << "Left Rear (0x" << std::setw(sizeof(msg.left_rear.error) << 1) << msg.left_rear.error << ") ";
  }
  if (msg.right_rear.error)
  {
    oss << "Right Rear (0x" << std::setw(sizeof(msg.right_rear.error) << 1) << msg.right_rear.error << ") ";
  }
  if (msg.right_front.error)
  {
    oss << "Right Front (0x" << std::setw(sizeof(msg.right_front.error) << 1) << msg.right_front.error << ") ";
  }

  wheels_[0].feedback(now, left_front_speed);
  wheels_[1].feedback(now, left_rear_speed);
  wheels_[2].feedback(now, right_rear_speed);
  wheels_[3].feedback(now, right_front_speed);
  Controller::callbackFeedback(now, msg.enabled, oss.str());

  // store feedback
  feedback_ = msg;
  diagnostic_frequency_.tick();
}

void MecanumDrive::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  const agv05_msgs::MecanumFeedback& feedback = feedback_();

  stat.add("Drive Type", "Mecanum Drive");
  stat.add("Motor Enable", static_cast<bool>(output_.enable));

  stat.addf("Motor Left Front Command Speed (m/s)", "%.3f", output_.left_front_speed);
  stat.addf("Motor Left Front Actual Speed (m/s)", "%.3f", wheels_[0].feedback_);
  stat.addf("Motor Left Front Speed Index", "%.3f", wheels_[0].index_);
  stat.addf("Motor Left Front Speed Error", "%.3f", wheels_[0].error_);
  stat.addf("Motor Left Front Distance (m)", "%.3f", feedback.left_front.distance);
  stat.addf("Motor Left Front Current (A)", "%.3f", feedback.left_front.current);
  stat.addf("Motor Left Front Temperature (°C)", "%.2f", feedback.left_front.temperature);
  stat.add("Motor Left Front Error Code", feedback.left_front.error);

  stat.addf("Motor Left Rear Command Speed (m/s)", "%.3f", output_.left_rear_speed);
  stat.addf("Motor Left Rear Actual Speed (m/s)", "%.3f", wheels_[1].feedback_);
  stat.addf("Motor Left Rear Speed Index", "%.3f", wheels_[1].index_);
  stat.addf("Motor Left Rear Speed Error", "%.3f", wheels_[1].error_);
  stat.addf("Motor Left Rear Distance (m)", "%.3f", feedback.left_rear.distance);
  stat.addf("Motor Left Rear Current (A)", "%.3f", feedback.left_rear.current);
  stat.addf("Motor Left Rear Temperature (°C)", "%.2f", feedback.left_rear.temperature);
  stat.add("Motor Left Rear Error Code", feedback.left_rear.error);

  stat.addf("Motor Right Rear Command Speed (m/s)", "%.3f", output_.right_rear_speed);
  stat.addf("Motor Right Rear Actual Speed (m/s)", "%.3f", wheels_[2].feedback_);
  stat.addf("Motor Right Rear Speed Index", "%.3f", wheels_[2].index_);
  stat.addf("Motor Right Rear Speed Error", "%.3f", wheels_[2].error_);
  stat.addf("Motor Right Rear Distance (m)", "%.3f", feedback.right_rear.distance);
  stat.addf("Motor Right Rear Current (A)", "%.3f", feedback.right_rear.current);
  stat.addf("Motor Right Rear Temperature (°C)", "%.2f", feedback.right_rear.temperature);
  stat.add("Motor Right Rear Error Code", feedback.right_rear.error);

  stat.addf("Motor Right Front Command Speed (m/s)", "%.3f", output_.right_front_speed);
  stat.addf("Motor Right Front Actual Speed (m/s)", "%.3f", wheels_[3].feedback_);
  stat.addf("Motor Right Front Speed Index", "%.3f", wheels_[3].index_);
  stat.addf("Motor Right Front Speed Error", "%.3f", wheels_[3].error_);
  stat.addf("Motor Right Front Distance (m)", "%.3f", feedback.right_front.distance);
  stat.addf("Motor Right Front Current (A)", "%.3f", feedback.right_front.current);
  stat.addf("Motor Right Front Temperature (°C)", "%.2f", feedback.right_front.temperature);
  stat.add("Motor Right Front Error Code", feedback.right_front.error);

  Controller::diagnosticStatus(stat);
}

}  // namespace agv05

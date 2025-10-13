/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <agv05_motor/controllers/swerve_drive.h>
#include <angles/angles.h>


namespace agv05
{

SwerveDrive::SwerveDrive() :
  Controller(2)
{
  ros::NodeHandle nh_motor("agv05/motor/swerve");

  // ROS publishers
  output_pub_ = nh_motor.advertise<agv05_msgs::SwerveControl>("control", 1, true);

  // ROS subscribers
  feedback_sub_ = nh_motor.subscribe("feedback", 10, &SwerveDrive::callbackFeedback, this);
}

void SwerveDrive::callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update)
{
  if (force_update || config.swerve_wheelbase != config_.swerve_wheelbase ||
      config.swerve_center_offset != config_.swerve_center_offset)
  {
    std::complex<double> l(config.swerve_wheelbase * 0.5, config.swerve_center_offset);
    Lock lock(mutex_);
    kinematic_.setup({l, -l});
    angular_speed_ratio_ = 1.0 / std::abs(kinematic_.control(0.0, 1.0)[0]);
  }

  if (config.swerve_front_steer_angle_min < config.swerve_front_steer_angle_max)
  {
    front_min_steer_ = config.swerve_front_steer_angle_min * M_PI / 180.0;
    front_max_steer_ = config.swerve_front_steer_angle_max * M_PI / 180.0;
  }
  else
  {
    front_min_steer_ = front_max_steer_ = 0.0f;
  }

  if (config.swerve_rear_steer_angle_min < config.swerve_rear_steer_angle_max)
  {
    rear_min_steer_ = config.swerve_rear_steer_angle_min * M_PI / 180.0;
    rear_max_steer_ = config.swerve_rear_steer_angle_max * M_PI / 180.0;
  }
  else
  {
    rear_min_steer_ = rear_max_steer_ = 0.0f;
  }

  Controller::callbackConfig(config, level, force_update);
}

float SwerveDrive::alignSteer(float front_angle, float rear_angle)
{
  double ret = std::abs(angles::normalize_angle(front_angle - rear_angle));
  return std::max(ret > M_PI_2 ? M_PI - ret : ret, 1 * M_PI / 180);
}

bool SwerveDrive::ctrlSteer(float& angle, float angle0, float min_steer, float max_steer)
{
  bool ret = false;

  float delta = angles::normalize_angle(angle - angle0);
  angle = angle0 + delta;
  if (delta < -M_PI_2)
  {
    angle += M_PI;
    ret ^= true;
  }
  else if (delta > M_PI_2)
  {
    angle -= M_PI;
    ret ^= true;
  }

  if (max_steer > 0.0)
  {
    if (!steering_align_.data)
    {
      min_steer = (min_steer + max_steer) * 0.5 - M_PI_2;
      max_steer = min_steer + M_PI;
    }

    while (angle < min_steer)
    {
      angle += M_PI;
      ret ^= true;
    }

    while (angle > max_steer)
    {
      angle -= M_PI;
      ret ^= true;
    }
  }

  return ret;
}

void SwerveDrive::ctrlCallback(bool enable, float period, const std::vector< std::complex<double> >& speed)
{
  output_.enable = enable;

  // safety trigger
  if (speed.size() != wheels_.size())
  {
    output_.front_speed = 0.0f;
    output_.rear_speed = 0.0f;

    steering_align_.data = false;
  }
  // navigation enabled
  else
  {
    // compute drive speed and steering angle
    float front_speed = std::abs(speed[0]);
    float front_angle = std::arg(speed[0]);
    float rear_speed = std::abs(speed[1]);
    float rear_angle = std::arg(speed[1]);

    if (front_speed)
    {
      if (ctrlSteer(front_angle, output_.front_angle, front_min_steer_, front_max_steer_))
      {
        front_speed = -front_speed;
      }
    }
    else
    {
      front_angle = rear_speed ? output_.front_angle : 0.0f;
      ctrlSteer(front_angle, output_.front_angle, front_min_steer_, front_max_steer_);
    }

    if (rear_speed)
    {
      if (ctrlSteer(rear_angle, output_.rear_angle, rear_min_steer_, rear_max_steer_))
      {
        rear_speed = -rear_speed;
      }
    }
    else
    {
      rear_angle = front_speed ? output_.rear_angle : 0.0f;
      ctrlSteer(rear_angle, output_.rear_angle, rear_min_steer_, rear_max_steer_);
    }

    // hysteresis for steering align state
    if (front_speed || rear_speed)
    {
      float threshold = (velocity_.linear.x == 0 && velocity_.linear.y == 0) ? 1 * M_PI / 180 :
                        steering_align_.data ? 120 * M_PI / 180 : alignSteer(front_angle, rear_angle);
      float error = std::max(std::abs(angles::normalize_angle(front_angle - feedback_.front.steering_angle)),
                             std::abs(angles::normalize_angle(rear_angle - feedback_.rear.steering_angle)));
      steering_align_.data = error <= threshold;
      if (steering_align_.data)
      {
        error = std::cos(std::min<float>(error, M_PI_2));
        front_speed *= (error / config_.front_motor_calibration);
        rear_speed *= (error / config_.rear_motor_calibration);
      }
      else
      {
        front_speed = rear_speed = 0.0f;
        if (!front_angle) front_angle = FLT_MIN;
        if (!rear_angle) rear_angle = FLT_MIN;
      }
    }
    else  // both linear and angular are zero
    {
      steering_align_.data = false;
    }

    // limit motor to max speed
    float max_speed = std::max(std::abs(front_speed), std::abs(rear_speed));
    if (max_speed_ < max_speed)
    {
      float normalize = max_speed_ / max_speed;
      front_speed *= normalize;
      rear_speed *= normalize;
    }

    // accelerate or decelerate
    output_.front_speed = wheels_[0].control(front_speed, period);
    output_.front_angle = front_angle;
    output_.rear_speed = wheels_[1].control(rear_speed, period);
    output_.rear_angle = rear_angle;
  }

  static agv05_msgs::SwerveControl output;
  ROS_DEBUG_COND(
    (output.front_speed != output_.front_speed ||
     output.front_angle != output_.front_angle ||
     output.rear_speed != output_.rear_speed ||
     output.rear_angle != output_.rear_angle ||
     output.enable != output_.enable) && (output = output_, true),
    "Control: %f %f %f %f %d", output_.front_speed, output_.front_angle,
                               output_.rear_speed, output_.rear_angle, output_.enable);

  // publish outputs
  output_pub_.publish(output_);
  steering_align_pub_.publish(steering_align_);
}

void SwerveDrive::callbackFeedback(const agv05_msgs::SwerveFeedback& msg)
{
  if (msg.stamp.isZero())
  {
    agv05_msgs::SwerveFeedback m = msg;
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
  double front_diff = (msg.front.distance - feedback_.front.distance) * config_.front_motor_calibration;
  double rear_diff = (msg.rear.distance - feedback_.rear.distance) * config_.rear_motor_calibration;
  double front_speed = msg.front.speed * config_.front_motor_calibration;
  double rear_speed = msg.rear.speed * config_.rear_motor_calibration;

  double dist = std::abs(front_diff) + std::abs(rear_diff);
  if (dist > 2)
  {
    ROS_ERROR("Huge odom difference: %f: %f %f %f %f",
              dist, front_diff, msg.front.steering_angle, rear_diff, msg.rear.steering_angle);
    ROS_ERROR("%f %f, %f %f",
              msg.front.distance, feedback_.front.distance, msg.rear.distance, feedback_.rear.distance);
    feedback_ = msg;
    return;
  }

  updateOdom(now,
             {std::polar(front_speed, msg.front.steering_angle), std::polar(rear_speed, msg.rear.steering_angle)},
             {std::polar(front_diff, msg.front.steering_angle), std::polar(rear_diff, msg.rear.steering_angle)},
             dist * 0.5);

  // motor fault detection
  std::ostringstream oss("", std::ostringstream::ate);
  oss << std::hex << std::setfill('0') << std::uppercase;
  if (msg.front.error)
  {
    oss << "Front (0x" << std::setw(sizeof(msg.front.error) << 1) << msg.front.error << ") ";
  }
  if (msg.rear.error)
  {
    oss << "Rear (0x" << std::setw(sizeof(msg.rear.error) << 1) << msg.rear.error << ") ";
  }

  wheels_[0].feedback(now, front_speed);
  wheels_[1].feedback(now, rear_speed);
  Controller::callbackFeedback(now, msg.enabled, oss.str());

  // store feedback
  feedback_ = msg;
  diagnostic_frequency_.tick();
}

void SwerveDrive::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Drive Type", "Swerve Drive");
  stat.add("Motor Enable", static_cast<bool>(output_.enable));
  stat.add("Steering Align", static_cast<bool>(steering_align_.data));

  stat.addf("Front Motor Command Speed (m/s)", "%.3f", output_.front_speed);
  stat.addf("Front Motor Actual Speed (m/s)", "%.3f", wheels_[0].feedback_);
  stat.addf("Front Motor Speed Index", "%.3f", wheels_[0].index_);
  stat.addf("Front Motor Speed Error", "%.3f", wheels_[0].error_);
  stat.addf("Front Steering Command Angle (°)", "%.3f", angles::to_degrees(output_.front_angle));
  stat.addf("Front Steering Actual Angle (°)", "%.3f", angles::to_degrees(feedback_.front.steering_angle));
  stat.addf("Front Motor Distance (m)", "%.3f", feedback_.front.distance);
  stat.addf("Front Motor Current (A)", "%.3f", feedback_.front.current);
  stat.addf("Front Motor Temperature (°C)", "%.2f", feedback_.front.temperature);
  stat.add("Front Motor Error Code", feedback_.front.error);

  stat.addf("Rear Motor Command Speed (m/s)", "%.3f", output_.rear_speed);
  stat.addf("Rear Motor Actual Speed (m/s)", "%.3f", wheels_[1].feedback_);
  stat.addf("Rear Motor Speed Index", "%.3f", wheels_[1].index_);
  stat.addf("Rear Motor Speed Error", "%.3f", wheels_[1].error_);
  stat.addf("Rear Steering Command Angle (°)", "%.3f", angles::to_degrees(output_.rear_angle));
  stat.addf("Rear Steering Actual Angle (°)", "%.3f", angles::to_degrees(feedback_.rear.steering_angle));
  stat.addf("Rear Motor Distance (m)", "%.3f", feedback_.rear.distance);
  stat.addf("Rear Motor Current (A)", "%.3f", feedback_.rear.current);
  stat.addf("Rear Motor Temperature (°C)", "%.2f", feedback_.rear.temperature);
  stat.add("Rear Motor Error Code", feedback_.rear.error);

  Controller::diagnosticStatus(stat);
}

}  // namespace agv05

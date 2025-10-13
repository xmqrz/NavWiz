/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <agv05_motor/controller.h>
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define VARIABLE_SAVE_PERIOD 10.0


namespace agv05
{

Controller::Controller(size_t wheel_num) :
  tfl_(tf_),
  expected_process_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_process_frequency_,
                        &expected_process_frequency_, 0.1, 1), "Motor Feedback Frequency"),
  mileage_spinner_(1, &mileage_callback_queue_),
  max_speed_(1.0f),
  safety_trigger_(false), button_unbrake_(false),
  last_imu_time_(ros::Time::now()),
  imu_yaw_prev_(-1.0f), imu_yaw_(-1.0f),
  mileage_(0.0), base_("base"), wheel_("wheel"),
  wheels_(wheel_num, Wheel(config_))
{
  // read mileage
  std::string value;
  double mileage;
  if (variable_storage_.getVariable("mileage", value))
  {
    std::istringstream iss(value);
    iss >> mileage;
  }
  mileage_ = mileage;

  ros::NodeHandle nh;
  ros::NodeHandle nh_motor("agv05/motor");

  // ROS publishers
  navigation_enable_pub_ = nh_motor.advertise<std_msgs::Bool>("navigation_enable", 1, true);
  steering_align_pub_ = nh_motor.advertise<std_msgs::Bool>("steering_align", 1, true);
  motor_fault_pub_ = nh.advertise<std_msgs::Bool>("agv05/safety/motor_fault", 1, true);
  motor_fault_hint_pub_ = nh_motor.advertise<std_msgs::String>("fault_hint", 1, true);
  safety_heartbeat_pub_ = nh.advertise<std_msgs::UInt8>("agv05/safety/heartbeat/" + ros::this_node::getName(), 1);

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1, true);
  odom_wheel_pub_ = nh.advertise<nav_msgs::Odometry>("odom/wheel", 1, true);
  straight_pub_ = nh_motor.advertise<std_msgs::Float64>("straight_distance", 1, true);
  rotational_pub_ = nh_motor.advertise<std_msgs::Float64>("rotational_distance", 1, true);
  mileage_pub_ = nh_motor.advertise<std_msgs::Float64>("mileage", 1, true);

  // ROS subscribers
  feedback_rate_sub_ = nh_motor.subscribe("feedback_rate", 1, &Controller::callbackFeedbackRate, this);
  max_speed_sub_ = nh_motor.subscribe("max_speed", 1, &Controller::callbackMaxSpeed, this);

  cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &Controller::callbackCmdVel, this);
  safety_trigger_sub_ = nh.subscribe("agv05/safety/safety_trigger", 1, &Controller::callbackSafetyTrigger, this);
  button_unbrake_sub_ = nh.subscribe("agv05/panel/control/button_unbrake", 1, &Controller::callbackButtonUnbrake, this);

  imu_sub_ = nh.subscribe("imu/data", 1, &Controller::callbackImu, this);

  // ROS services
  get_twist_srv_ = nh_motor.advertiseService("get_twist", &Controller::onGetTwist, this);
  reset_odom_srv_ = nh.advertiseService("reset_odom", &Controller::onResetOdom, this);

  ros::NodeHandle nh_mileage;
  nh_mileage.setCallbackQueue(&mileage_callback_queue_);
  reset_mileage_srv_ = nh_mileage.advertiseService("reset_mileage", &Controller::onResetMileage, this);

  // diagnostic updater
  diagnostic_updater_.setHardwareID("AGV05");
  diagnostic_updater_.add(diagnostic_frequency_);
  diagnostic_updater_.add("Status", this, &Controller::diagnosticStatus);

  // ROS timer for speed smoothening
  timer_ = nh.createTimer(ros::Duration(1.0 / LOOP_FREQUENCY), &Controller::timerCallback, this);
  timer2_ = nh.createTimer(ros::Duration(0.1), &Controller::timer2Callback, this);

  // ROS timer for writing mileage
  mileage_timer_ = nh_mileage.createTimer(ros::Duration(VARIABLE_SAVE_PERIOD), &Controller::mileageTimerCallback, this);

  // Initialize
  std_msgs::Float64 msg;
  msg.data = mileage;
  mileage_pub_.publish(msg);
  mileage_spinner_.start();
}

void Controller::timerCallback(const ros::TimerEvent& event)
{
  const ros::Time& now = event.current_real;

  // Hack: Use the smallest positive denormal to signal free motor
  bool free_motor = *reinterpret_cast<uint64_t*>(&velocity_.linear.z) == 1LL || button_unbrake_;

  bool complete_stop = true;
  if (config_.decelerate_on_safety_trigger && !free_motor)
  {
    for (auto& wheel : wheels_)
    {
      if (wheel.speed_)
      {
        complete_stop = false;
        break;
      }
    }
  }

  // safety trigger
  if (safety_trigger_ && complete_stop)
  {
    for (auto& wheel : wheels_)
    {
      wheel.speed_ = 0.0f;
    }

    ctrlCallback(!free_motor);
    navigation_enable_.data = false;
  }
  // navigation enabled
  else
  {
    const ros::Time& last = last_velocity_ < last_safety_trigger_ ? last_velocity_ : last_safety_trigger_;
    if (safety_trigger_ || (now - last).toSec() > config_.velocity_timeout)
    {
      velocity_.linear.x = 0.0;
      velocity_.linear.y = 0.0;
      velocity_.angular.z = 0.0;
    }

    std::vector< std::complex<double> > speed;
    {
      Lock lock(mutex_);
      speed = kinematic_.control(std::complex<double>(velocity_.linear.x, velocity_.linear.y), velocity_.angular.z);
    }

    float period = (event.current_real - event.last_real).toSec();
    period = std::max(0.0f, std::min<float>(2.0f / expected_process_frequency_, period));  // limit period in the event of time jump.
    ctrlCallback(true, period, speed);
  }

  // publish outputs
  navigation_enable_pub_.publish(navigation_enable_);

  // trigger error if no motor feedback within timeout
  if ((now - last_no_motor_fault_).toSec() > config_.motor_fault_trigger_timeout)
  {
    if (!motor_fault_.data)
    {
      motor_fault_.data = true;
      motor_fault_pub_.publish(motor_fault_);

      motor_fault_hint_.data = std::to_string(config_.motor_fault_trigger_timeout) + "s Feedback Timeout";
      motor_fault_hint_pub_.publish(motor_fault_hint_);
    }
    resetOdom(now);
  }
}

void Controller::callbackFeedback(const ros::Time& now, bool enable, const std::string& motor_fault_hint)
{
  navigation_enable_.data = enable;

  float speed_index = 1.0f;
  float speed_error = 0.0f;
  if (config_.motor_fault_trigger_sensitivity > 0.0)
  {
    for (auto& wheel : wheels_)
    {
      speed_index = std::min(speed_index, wheel.index_);
      speed_error = std::max(speed_error, std::abs(wheel.error_));
    }
    speed_error = config_.motor_fault_trigger_fir ? speed_error * LOOP_FREQUENCY : 0.0f;
  }

  // motor fault detection
  if (motor_fault_hint != "")
  {
    motor_fault_.data = true;
    motor_fault_hint_.data = motor_fault_hint;
    motor_fault_hint_pub_.publish(motor_fault_hint_);
  }
  else if (speed_index < config_.motor_fault_trigger_sensitivity)
  {
    if ((now - last_no_motor_fault_).toSec() > config_.motor_fault_trigger_timeout)
    {
      motor_fault_.data = true;
      motor_fault_hint_.data = "Speed Index " + std::to_string(speed_index);
      motor_fault_hint_pub_.publish(motor_fault_hint_);
      ROS_ERROR_STREAM_THROTTLE(config_.motor_fault_trigger_timeout, "Motor Fault Speed Index: " << speed_index);
    }
  }
  else if (speed_error > config_.max_acceleration * config_.motor_fault_trigger_timeout)
  {
    motor_fault_.data = true;
    motor_fault_hint_.data = "Speed Error " + std::to_string(speed_error);
    motor_fault_hint_pub_.publish(motor_fault_hint_);
    ROS_ERROR_STREAM_THROTTLE(config_.motor_fault_trigger_timeout, "Motor Fault Speed Error: " << speed_error);
  }
  else
  {
    motor_fault_.data = false;
    last_no_motor_fault_ = now;

    if (motor_fault_hint_.data != "" && !safety_trigger_)
    {
      motor_fault_hint_.data = "";
      motor_fault_hint_pub_.publish(motor_fault_hint_);
    }
  }
  motor_fault_pub_.publish(motor_fault_);
}

bool Controller::getTwist(bool max, geometry_msgs::Twist& twist)
{
  const std::vector< std::complex<double> > output = getOutput(max);
  std::complex<double> v;
  double w;

  if (output.size())
  {
    Lock lock(mutex_);

    if (max)
    {
      v = kinematic_.dXY(output, 0.0);
      w = v.real() * angular_speed_ratio_;
    }
    else
    {
      w = kinematic_.dTheta(output);
      v = kinematic_.dXY(output, w);
    }
  }
  else
  {
    return false;
  }

  twist.linear.x = v.real();
  twist.linear.y = v.imag();
  twist.angular.z = w;
  return true;
}

bool Controller::onResetOdom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Reset odom");

  // reset odom variable
  resetOdom(ros::Time::now(), true);

  return true;
}

void Controller::mileageTimerCallback(const ros::TimerEvent& event)
{
  double mileage;
  {
    Lock lock(mutex_);
    mileage = mileage_;
  }
  writeMileage(mileage);
}

bool Controller::onResetMileage(agv05_msgs::ResetMileage::Request& request, agv05_msgs::ResetMileage::Response& response)
{
  {
    Lock lock(mutex_);
    mileage_ = request.mileage;
  }
  writeMileage(request.mileage);
  return true;
}

void Controller::writeMileage(double mileage)
{
  // write mileage
  std::ostringstream oss;
  oss << std::setprecision(std::numeric_limits<double>::digits10) << mileage;
  variable_storage_.setVariable("mileage", oss.str());

  // publish mileage
  std_msgs::Float64 msg;
  msg.data = mileage;
  mileage_pub_.publish(msg);
}

void Controller::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.addf("Straight Distance (m)", "%.3f", straight_.data);
  stat.addf("Rotational Distance (rad)", "%.3f", rotational_.data);

  double mileage;
  {
    Lock lock(mutex_);
    mileage = mileage_;
  }
  stat.addf("Mileage (m)", "%.3f", mileage);

  tf2::Quaternion q;
  tf2::fromMsg(base_.odom_.pose.pose.orientation, q);
  stat.addf("Odom - X (m)", "%.3f", base_.odom_.pose.pose.position.x);
  stat.addf("Odom - Y (m)", "%.3f", base_.odom_.pose.pose.position.y);
  stat.addf("Odom - Theta (°)", "%.2f", angles::to_degrees(tf2::getYaw(q)));

  tf2::fromMsg(wheel_.odom_.pose.pose.orientation, q);
  stat.addf("Odom Wheel - X (m)", "%.3f", wheel_.odom_.pose.pose.position.x);
  stat.addf("Odom Wheel - Y (m)", "%.3f", wheel_.odom_.pose.pose.position.y);
  stat.addf("Odom Wheel - Theta (°)", "%.2f", angles::to_degrees(tf2::getYaw(q)));

  stat.add("Safety Trigger", static_cast<bool>(safety_trigger_));
  stat.add("Navigation Enable", static_cast<bool>(navigation_enable_.data));
  stat.add("Motor Fault", static_cast<bool>(motor_fault_.data));
  stat.add("Motor Fault Hint", motor_fault_hint_.data == "" ? "-" : motor_fault_hint_.data);

  // summary
  if (config_.imu_odom_timeout > 0.0)
  {
    if ((last_move_ - last_imu_time_).toSec() > config_.imu_odom_timeout)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU data timeout");
      return;
    }
  }
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status OK");
}

void Controller::publishOdom(const ros::Time& now)
{
  // publish tf and odom
  base_.transform_.header.stamp = base_.odom_.header.stamp = now;
  tfb_.sendTransform(base_.transform_);
  odom_pub_.publish(base_.odom_);

  wheel_.transform_.header.stamp = wheel_.odom_.header.stamp = now;
  tfb_.sendTransform(wheel_.transform_);
  odom_wheel_pub_.publish(wheel_.odom_);
}

void Controller::updateOdom(const ros::Time& now, const std::vector< std::complex<double> >& v,
                            const std::vector< std::complex<double> >& dxy, double dist)
{
  std::complex<double> v_base, dxy_wheel, dxy_base;
  double vx, vy, w, dtheta_wheel, dtheta, dx;

  dist = std::abs(dist);

  {
    Lock lock(mutex_);

    w = kinematic_.dTheta(v);
    v_base = kinematic_.dXY(v, w);

    dtheta_wheel = kinematic_.dTheta(dxy);
    dxy_wheel = kinematic_.dXY(dxy, dtheta_wheel);

    dtheta = yawChange(dtheta_wheel, dist > 0.0);
    dxy_base = kinematic_.dXY(dxy, dtheta);
  }

  vx = v_base.real();
  vy = v_base.imag();
  dx = dxy_base.real();
  base_.update(dx, dxy_base.imag(), dtheta, vx, vy, w);
  wheel_.update(dxy_wheel.real(), dxy_wheel.imag(), dtheta_wheel, vx, vy, w);

  publishOdom(now);

  // publish only on change
  if (dist > 0.0 || dtheta != 0.0)
  {
    straight_.data += dx;
    rotational_.data += dtheta;

    straight_pub_.publish(straight_);
    rotational_pub_.publish(rotational_);

    // update mileage
    Lock lock(mutex_);
    mileage_ += dist;
  }
}

Controller::Odom::Odom(const std::string& frame_id)
{
  // initialize transform and odom
  transform_.header.frame_id = odom_.header.frame_id = "odom";
  transform_.child_frame_id = odom_.child_frame_id = frame_id;

  transform_.transform.translation.x = odom_.pose.pose.position.x = 0;
  transform_.transform.translation.y = odom_.pose.pose.position.y = 0;
  transform_.transform.translation.z = odom_.pose.pose.position.z = 0;
  transform_.transform.rotation.x = odom_.pose.pose.orientation.x = 0;
  transform_.transform.rotation.y = odom_.pose.pose.orientation.y = 0;
  transform_.transform.rotation.z = odom_.pose.pose.orientation.z = 0;
  transform_.transform.rotation.w = odom_.pose.pose.orientation.w = 1;

  // initialize odom pose covariance (required if using robot_pose_ekf)
  odom_.pose.covariance[0] = 0.5 * 0.5;
  odom_.pose.covariance[7] = 0.5 * 0.5;
  odom_.pose.covariance[35] = (M_PI / 12) * (M_PI / 12);

  odom_.pose.covariance[14] = DBL_MAX;  // set a non-zero covariance on unused
  odom_.pose.covariance[21] = DBL_MAX;  // dimensions (z, pitch and roll); this
  odom_.pose.covariance[28] = DBL_MAX;  // is a requirement of robot_pose_ekf

  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = 0;
}

void Controller::Odom::reset(bool pos)
{
  if (pos)
  {
    transform_.transform.translation.x = odom_.pose.pose.position.x = 0;
    transform_.transform.translation.y = odom_.pose.pose.position.y = 0;
    transform_.transform.rotation.x = odom_.pose.pose.orientation.x = 0;
    transform_.transform.rotation.y = odom_.pose.pose.orientation.y = 0;
    transform_.transform.rotation.z = odom_.pose.pose.orientation.z = 0;
    transform_.transform.rotation.w = odom_.pose.pose.orientation.w = 1;
  }

  odom_.twist.twist.linear.x = 0;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.angular.z = 0;
}

void Controller::Odom::update(double dx, double dy, double dtheta, double vx, double vy, double w)
{
  // velocity (required if using dwa_local_planner)
  odom_.twist.twist.linear.x = vx;
  odom_.twist.twist.linear.y = vy;
  odom_.twist.twist.angular.z = w;

  tf2::Quaternion q;
  tf2::fromMsg(odom_.pose.pose.orientation, q);
  double theta = tf2::getYaw(q);
  q.setRPY(0, 0, theta + dtheta);
  odom_.pose.pose.orientation = tf2::toMsg(q);

  std::complex<double> dxy = Kinematic2D<double>::dXY(std::complex<double>(dx, dy), dtheta, theta);
  odom_.pose.pose.position.x += dxy.real();
  odom_.pose.pose.position.y += dxy.imag();

  transform_.transform.translation.x = odom_.pose.pose.position.x;
  transform_.transform.translation.y = odom_.pose.pose.position.y;
  transform_.transform.rotation = odom_.pose.pose.orientation;
}

}  // namespace agv05

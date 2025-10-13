/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_MOTOR_CONTROLLER_H
#define AGV05_MOTOR_CONTROLLER_H

#include <agv05_variable_storage/variable_storage.h>
#include <angles/angles.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <agv05_motor/MotorConfig.h>
#include <agv05_msgs/GetTwist.h>
#include <agv05_msgs/ResetMileage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>

// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// Remove this header when https://github.com/ros/geometry_experimental/pull/78 is released
#include <agv05_motor/tf2_sensor_msgs_imu.h>

#define LOOP_FREQUENCY 100.0
#define FEEDBACK_WINDOW_SIZE_MAX 0x10  // must be 2^n
#define FEEDBACK_WINDOW_MASK     (FEEDBACK_WINDOW_SIZE_MAX-1)


namespace agv05
{

template<class T>
class Feedback
{
public:
  Feedback() : index_(FEEDBACK_WINDOW_SIZE_MAX) {}

  bool setup(const T& msg)
  {
    if (index_ == FEEDBACK_WINDOW_SIZE_MAX)
    {
      while (index_)
      {
        feedback_[--index_] = msg;
      }
      return true;
    }
    return false;
  }

  const T& operator=(const T& msg)
  {
    feedback_[index_] = msg;
    index_ = index_ ? index_ - 1 : FEEDBACK_WINDOW_MASK;
    return msg;
  }

  const T& operator()(uint8_t count = 1)
  {
    return feedback_[(index_ + count) & FEEDBACK_WINDOW_MASK];
  }

protected:
  T feedback_[FEEDBACK_WINDOW_SIZE_MAX];
  uint8_t index_;
};


template<class T>
class Kinematic2D
{
public:
  void setup(const std::vector< std::complex<T> >& range)
  {
    r_ = range;
    r_sum_ = 0;
    for (const auto& r : r_)
    {
      r_sum_ += r;
    }
  }

  std::vector< std::complex<T> > control(const std::complex<T>& v, T w)
  {
    std::vector< std::complex<T> > ret;
    if (r_.size() == 1)
    {
      ret.push_back(r_sum_ * v.real() / r_sum_.real() + cross(w, r_sum_));
    }
    else for (const auto& r : r_)
    {
      ret.push_back(v + cross(w, r));
    }
    return ret;
  }

  T dTheta(const std::vector< std::complex<T> >& dxy)
  {
    T sum = 0;
    size_t s = std::min(dxy.size(), r_.size());
    for (size_t i = 0; i < s; i++)
    {
      sum += (cross(r_[i], dxy[i]) / std::norm(r_[i]));
    }
    return (s > 1) ? sum / s : sum;
  }

  std::complex<T> dXY(const std::vector< std::complex<T> >& dxy, T dtheta)
  {
    std::complex<T> sum = cross(-dtheta, r_sum_);
    for (const auto& d : dxy)
    {
      sum += d;
    }
    size_t s = dxy.size();
    return (s > 1) ? sum /= s : sum.real();
  }

  static std::complex<T> dXY(const std::complex<T>& dxy, T dtheta, T theta)
  {
    // Runge-Kutta 2nd order integration
    return std::polar<T>(1, 0.5 * dtheta + theta) * dxy;
  }

  static T dot(const std::complex<T>& a, const std::complex<T>& b)
  {
    return a.real() * b.real() + a.imag() * b.imag();
  }
  static T cross(const std::complex<T>& a, const std::complex<T>& b)
  {
    return a.real() * b.imag() - a.imag() * b.real();
  }
  static std::complex<T> cross(const std::complex<T>& a, T b)
  {
    return std::complex<T>(a.imag() * b, -a.real() * b);
  }
  static std::complex<T> cross(T a, const std::complex<T>& b)
  {
    return std::complex<T>(-a * b.imag(), a * b.real());
  }

protected:
  std::vector< std::complex<T> > r_;
  std::complex<T> r_sum_;  // for asymmetrical design
};


class Controller
{
public:
  explicit Controller(size_t wheel_num);

  // dynamic reconfigure
  virtual void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level, bool force_update = false)
  {
    config_ = config;

    if (force_update)
    {
      publishOdom(ros::Time::now());
    }
  }

protected:
  // timer for speed smoothening
  void timerCallback(const ros::TimerEvent& event);
  virtual void ctrlCallback(bool enable, float period = 0.0f, const std::vector< std::complex<double> >& speed = {}) = 0;

  void timer2Callback(const ros::TimerEvent& event)
  {
    diagnostic_updater_.update();

    std_msgs::UInt8 msg;
    msg.data = config_.imu_odom_timeout && (ros::Time::now() - last_imu_time_).toSec() > config_.imu_odom_timeout;
    safety_heartbeat_pub_.publish(msg);
  }

  // motor feedback
  void callbackFeedback(const ros::Time& now, bool enable, const std::string& motor_fault_hint);

  void callbackFeedbackRate(const std_msgs::Float32& msg)
  {
    expected_process_frequency_ = std::max(10.0f, msg.data);
  }
  void callbackMaxSpeed(const std_msgs::Float32& msg)
  {
    max_speed_ = msg.data;
  }
  void callbackCmdVel(const geometry_msgs::Twist& msg)
  {
    velocity_ = msg;
    last_velocity_ = ros::Time::now();
  }
  void callbackSafetyTrigger(const std_msgs::Bool& msg)
  {
    safety_trigger_ = msg.data;
    last_safety_trigger_ = ros::Time::now();
  }
  void callbackButtonUnbrake(const std_msgs::Bool& msg)
  {
    button_unbrake_ = msg.data;
  }
  void callbackImu(const sensor_msgs::Imu& msg)
  {
    const sensor_msgs::Imu *imu = &msg;
    sensor_msgs::Imu imu_base;
    tf2::Quaternion q;

    if (msg.header.frame_id != base_.transform_.child_frame_id)
    {
      try
      {
        tf_.transform(msg, imu_base, base_.transform_.child_frame_id);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "IMU Transform failure: " << ex.what());
        return;
      }
      imu = &imu_base;
    }

    tf2::fromMsg(imu->orientation, q);
    imu_yaw_ = angles::normalize_angle_positive(tf2::getYaw(q));
    last_imu_time_ = msg.header.stamp;
  }
  virtual std::vector< std::complex<double> > getOutput(bool max)
  {
    return {};
  }
  virtual bool getTwist(bool max, geometry_msgs::Twist& twist);
  bool onGetTwist(agv05_msgs::GetTwist::Request& request, agv05_msgs::GetTwist::Response& response)
  {
    return getTwist(request.max, response.data);
  }
  bool onResetOdom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  // mileage
  void mileageTimerCallback(const ros::TimerEvent& event);
  bool onResetMileage(agv05_msgs::ResetMileage::Request& request, agv05_msgs::ResetMileage::Response& response);
  void writeMileage(double mileage);

  // diagnostic
  virtual void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // helper function
  virtual void publishOdom(const ros::Time& now);
  virtual void resetOdom(const ros::Time& now, bool pos = false)
  {
    base_.reset(pos);
    wheel_.reset(pos);
    publishOdom(now);
  }
  void updateOdom(const ros::Time& now, const std::vector< std::complex<double> >& v,
                  const std::vector< std::complex<double> >& dxy, double dist);

  double yawChange(double theta_diff, bool moving)
  {
    double ret = theta_diff;
    ros::Time now = ros::Time::now();
    if (moving)
    {
      last_move_ = now;
    }
    if ((now - last_move_).toSec() < config_.imu_odom_timeout)
    {
      if (imu_yaw_ >= 0.0f && imu_yaw_prev_ >= 0.0f)
      {
        ret = angles::normalize_angle(imu_yaw_ - imu_yaw_prev_);
      }
    }
    imu_yaw_prev_ = imu_yaw_;
    imu_yaw_ = -1.0f;
    return ret;
  }

protected:
  /* ROS tf2 listener and broadcaster */
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tfl_;
  tf2_ros::TransformBroadcaster tfb_;

  /* ROS publishers */
  ros::Publisher navigation_enable_pub_;
  ros::Publisher steering_align_pub_;
  ros::Publisher motor_fault_pub_;
  ros::Publisher motor_fault_hint_pub_;
  ros::Publisher safety_heartbeat_pub_;

  ros::Publisher odom_pub_;
  ros::Publisher odom_wheel_pub_;
  ros::Publisher straight_pub_;
  ros::Publisher rotational_pub_;
  ros::Publisher mileage_pub_;

  /* ROS subscribers */
  ros::Subscriber feedback_rate_sub_;
  ros::Subscriber max_speed_sub_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber safety_trigger_sub_;
  ros::Subscriber button_unbrake_sub_;

  ros::Subscriber imu_sub_;

  /* ROS services */
  ros::ServiceServer get_twist_srv_;
  ros::ServiceServer reset_odom_srv_;
  ros::ServiceServer reset_mileage_srv_;

  /* ROS diagnostic */
  double expected_process_frequency_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::FrequencyStatus diagnostic_frequency_;

  /* Timer */
  ros::Timer timer_;
  ros::Timer timer2_;

  /* Mileage timer */
  ros::CallbackQueue mileage_callback_queue_;
  ros::AsyncSpinner mileage_spinner_;
  ros::Timer mileage_timer_;

  /* Variable storage */
  agv05::VariableStorage variable_storage_;

  /* Mutex */
  boost::mutex mutex_;
  typedef boost::unique_lock<boost::mutex> Lock;

  /* Data inputs */
  agv05_motor::MotorConfig config_;
  float max_speed_;
  float angular_speed_ratio_;

  geometry_msgs::Twist velocity_;
  ros::Time last_velocity_;
  ros::Time last_safety_trigger_;
  bool safety_trigger_;
  bool button_unbrake_;

  ros::Time last_move_;
  ros::Time last_imu_time_;
  double imu_yaw_prev_;
  double imu_yaw_;

  /* Data outputs */
  std_msgs::Bool navigation_enable_;
  std_msgs::Bool motor_fault_;
  std_msgs::String motor_fault_hint_;
  ros::Time last_no_motor_fault_;

  std_msgs::Float64 straight_;
  std_msgs::Float64 rotational_;
  double mileage_;  // accessible by both main thread and mileage thread

  /* Kinematic */
  Kinematic2D<double> kinematic_;  // accessible by both main thread and config thread

  class Odom
  {
  public:
    explicit Odom(const std::string& frame_id);
    void reset(bool pos = false);
    void update(double dx, double dy, double dtheta, double vx, double vy, double w);

    geometry_msgs::TransformStamped transform_;
    nav_msgs::Odometry odom_;
  } base_, wheel_;

  class Wheel
  {
  public:
    explicit Wheel(const agv05_motor::MotorConfig& config) :
      speed_(0.0f), feedback_(0.0f), index_(0.0f), error_(0.0f), config_(config)
    {}

    float control(float target, float period)
    {
      if (speed_ * feedback_ <= 0.0f) speed_ = 0.0f;
      if (speed_ == target) return speed_;

      float inc = config_.max_acceleration * period;
      float dec = config_.max_deceleration * period;

      // forward
      if (speed_ > 0.0f || speed_ == 0.0f && target > 0.0f)
      {
        if (speed_ + inc < target) speed_ += inc;  // acceleration
        else if (speed_ - dec > target) speed_ -= dec;  // deceleration
        else speed_ = target;
      }
      // reverse
      else
      {
        if (speed_ + dec < target) speed_ += dec;  // deceleration
        else if (speed_ - inc > target) speed_ -= inc;  // acceleration
        else speed_ = target;
      }
      return speed_;
    }

    float feedback(const ros::Time& now, float speed)
    {
      // update speed and speed index
      feedback_ = speed;
      index_ = (std::abs(speed_) <= config_.motor_fault_trigger_min_speed) ? 1.0f : feedback_ / speed_;
      error_ = motor_fault_.convolution(now, config_.motor_fault_trigger_timeout / FEEDBACK_WINDOW_MASK,
                                        (index_ < config_.motor_fault_trigger_sensitivity) ? speed_ - feedback_ : 0.0f);
      return index_;
    }

    float speed_;     // command speed
    float feedback_;  // actual speed
    float index_;     // ratio of actual speed to command speed
    float error_;     // convolution speed error over time

  protected:
    const agv05_motor::MotorConfig& config_;

    class MotorFault
    {
    public:
      MotorFault()
      {
        clear();
      }

      void clear(const ros::Time& now = ros::Time::now())
      {
        last_ = now;
        for (size_t i = 0; i < FEEDBACK_WINDOW_SIZE_MAX; i++)
        {
          time_[i] = last_;
          sub_[i] = 0.0f;
        }
        sum_ = 0.0f;
        index_ = 0;
      }

      float convolution(const ros::Time& now, float period, float error)
      {
        if (now < last_ || (now - last_).toSec() > period)
        {
          clear(now);
          return sum_;
        }
        if ((now - time_[index_]).toSec() > period)
        {
          index_ = index_ ? index_ - 1 : FEEDBACK_WINDOW_MASK;
          sum_ -= sub_[index_];
          sub_[index_] = 0;
          time_[index_] = now;
        }
        error *= (now - last_).toSec();
        last_ = now;
        sub_[index_] += error;
        sum_ += error;
        return sum_;
      }

    private:
      ros::Time last_;
      ros::Time time_[FEEDBACK_WINDOW_SIZE_MAX];
      float sub_[FEEDBACK_WINDOW_SIZE_MAX];
      float sum_;
      uint8_t index_;
    } motor_fault_;
  };
  std::vector<Wheel> wheels_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_CONTROLLER_H

/*
 * Copyright (c) 2019, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_STATE_FILTER_STATE_FILTER_H
#define AGV05_STATE_FILTER_STATE_FILTER_H

#include <agv05_state_filter/StateFilterConfig.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <csm/csm_all.h>

namespace agv05
{

class StateFilterBase
{
public:
  StateFilterBase(): tfl_(tf_) {}

  void process(float rate)
  {
    safety_heartbeat_pub_.publish(std_msgs::UInt8());
  }

  // helper functions
  const char* getScanIcpError() {return getError(scan_icp_error_);}
  const char* getLinearError()  {return getError(linear_error_);}
  const char* getAngularError() {return getError(angular_error_);}

private:
  const char* getError(const std::vector<bool>& error)
  {
    for (int i = 0; i < error.size(); i++)
    {
      if (error[i]) return topics_[i].c_str();
    }
    return NULL;
  }

  int addTopic(const std::string& topic)
  {
    topics_.push_back(topic);

    scan_icp_error_.push_back(false);
    linear_error_.push_back(false);
    angular_error_.push_back(false);

    return topics_.size() - 1;
  }

  void updateIcpError(int id, bool error)
  {
    scan_icp_error_[id] = error;
    if (error)
    {
      updater_.update();
    }
  }

  void updateSlippageError(int id, double linear_error, double angular_error)
  {
    // check error against the threshold
    linear_error_[id] = std::abs(linear_error) > config_.linear_error_threshold;
    angular_error_[id] = std::abs(angular_error) > config_.angular_error_threshold;

    // publish wheel slippage
    std_msgs::Bool wheel_slippage;
    wheel_slippage.data = getLinearError() || getAngularError();
    wheel_slippage_pub_.publish(wheel_slippage);

    // publish diagnostic
    updater_.update();
  }

  friend class StateFilterBaseSensor;

public:
  ros::NodeHandle nh_;

protected:
  // dynamic reconfigure server
  dynamic_reconfigure::Server<agv05_state_filter::StateFilterConfig> ds_;
  agv05_state_filter::StateFilterConfig config_;

  // tf listener
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tfl_;

  // publishers
  ros::Publisher wheel_slippage_pub_;
  ros::Publisher safety_heartbeat_pub_;

  // subscribers
  std::vector<std::string> topics_;

  // diagnostic
  diagnostic_updater::Updater updater_;

private:
  // error status
  std::vector<bool> scan_icp_error_;
  std::vector<bool> linear_error_;
  std::vector<bool> angular_error_;
};


class StateFilterBaseSensor
{
public:
  explicit StateFilterBaseSensor(StateFilterBase& base, const std::string& topic);

  // callbacks
  virtual void handleConfig();
  virtual void statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

protected:
  // helper functions
  const char* getTopic()
  {
    return base_.topics_[id_].c_str();
  }
  void updateIcpError(bool error)
  {
    base_.updateIcpError(id_, error);
  }
  void updateSlippageError(double linear_error, double angular_error)
  {
    // insert error into history array
    linear_error_accum_[error_accum_i_] = linear_error;
    angular_error_accum_[error_accum_i_] = angular_error;
    if (++error_accum_i_ >= config_.error_average_window)
    {
      error_accum_i_ = 0;
    }

    // average the error over time
    linear_error = 0;
    angular_error = 0;
    for (int i = 0; i < config_.error_average_window; i++)
    {
      linear_error += linear_error_accum_[i];
      angular_error += angular_error_accum_[i];
    }
    linear_error_avg_ = linear_error / config_.error_average_window;
    angular_error_avg_ = angular_error / config_.error_average_window;

    base_.updateSlippageError(id_, linear_error_avg_, angular_error_avg_);
  }

  void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t)
  {
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
  }

  void convertTfToXYTheta(const tf2::Transform& t, double& x, double& y, double& theta)
  {
    tf2::Vector3 o = t.getOrigin();
    tf2::Quaternion r = t.getRotation();
    x = o.x();
    y = o.y();
    theta = tf2::getYaw(r);
  }
  void convertTfToXYThetaDeg(const tf2::Transform& t, double buffer[3])
  {
    convertTfToXYTheta(t, buffer[0], buffer[1], buffer[2]);
    buffer[2] *= (180.0 / M_PI);
  }

protected:
  agv05_state_filter::StateFilterConfig& config_;
  tf2_ros::Buffer& tf_;

  bool initialized_;
  ros::Time last_processed_;
  tf2::Stamped<tf2::Transform> last_odom_;
  tf2::Stamped<tf2::Transform> last_amcl_;

private:
  StateFilterBase& base_;

  int id_;

  double linear_error_accum_[10];
  double angular_error_accum_[10];
  double linear_error_avg_;
  double angular_error_avg_;
  int error_accum_i_;
};


class StateFilterOdom: public StateFilterBaseSensor
{
public:
  explicit StateFilterOdom(StateFilterBase& base, const std::string& topic);

private:
  // callbacks
  void handleOdom(const nav_msgs::Odometry& odom);

private:
  // subscribers
  ros::Subscriber odom_sub_;

  // other variables
  tf2::Transform odom_to_base_;
};


class StateFilterICP: public StateFilterBaseSensor
{
public:
  explicit StateFilterICP(StateFilterBase& base, const std::string& topic);

  // callbacks
  void handleConfig();

private:
  // callbacks
  void handleScan(const sensor_msgs::LaserScan& scan);

  // helper functions
  bool scanICP(LDP& cur_ldp, const tf2::Transform& odom_delta, tf2::Transform& corr_ch);

  bool getBaseToLaserTf(const std::string& frame_id);
  void laserScanToLDP(const sensor_msgs::LaserScan& scan, LDP& laser_data);

  void publishOdom(const ros::Time& now, const tf2::Transform& delta);

private:
  // publishers
  ros::Publisher odom_pub_;

  // subscribers
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;

  // CSM variable
  sm_params sm_input_;
  sm_result sm_output_;
  LDP prev_laser_data_;

  // other variables
  tf2::Transform base_to_laser_;  // static, cached
  tf2::Transform laser_to_base_;  // static, cached
  tf2::Transform odom_to_base_;
  nav_msgs::Odometry odom_;
};


class StateFilter: public StateFilterBase
{
public:
  explicit StateFilter(const std::string& topics_param_name);

private:
  // callbacks
  void handleConfig(agv05_state_filter::StateFilterConfig& config, uint32_t level);
  void statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // helper functions
  bool obtainParam(ros::NodeHandle& nh, const std::string& suffix, std::string& param_value);

private:
  // odom topic handlers
  std::vector< boost::shared_ptr<StateFilterBaseSensor> > filters_;
};

}  // namespace agv05

#endif  // AGV05_STATE_FILTER_STATE_FILTER_H

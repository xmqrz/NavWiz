/*
 * Copyright (c) 2019, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_state_filter/state_filter.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace agv05
{

StateFilterBaseSensor::StateFilterBaseSensor(StateFilterBase& base, const std::string& topic):
  base_(base), config_(base.config_), tf_(base.tf_),
  initialized_(false)
{
  // dynamic reconfigure server
  handleConfig();

  id_ = base_.addTopic(topic);
}

void StateFilterBaseSensor::handleConfig()
{
  error_accum_i_ = 0;
  for (int i = 0; i < config_.error_average_window; i++)
  {
    linear_error_accum_[i] = 0;
    angular_error_accum_[i] = 0;
  }
}

void StateFilterBaseSensor::statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::ostringstream oss;
  oss << base_.topics_[id_] << " linear_error";
  stat.add(oss.str(), linear_error_avg_);
  oss.str("");
  oss << base_.topics_[id_] << " angular_error";
  stat.add(oss.str(), angular_error_avg_);
}


StateFilterOdom::StateFilterOdom(StateFilterBase& base, const std::string& topic):
  StateFilterBaseSensor(base, topic)
{
  // subscriber
  odom_sub_ = base.nh_.subscribe(topic, 1, &StateFilterOdom::handleOdom, this);
}

void StateFilterOdom::handleOdom(const nav_msgs::Odometry& odom)
{
  ros::Time odom_time = odom.header.stamp;

  double dt = (odom_time - last_processed_).toSec();
  if (dt < (1.0 / config_.process_rate))
  {
    return;
  }

  tf2::Stamped<tf2::Transform> odom_wheel;
  try
  {
    tf2::fromMsg(tf_.lookupTransform("odom", "wheel", odom_time, ros::Duration(0.05)), odom_wheel);
  }
  catch (tf2::TransformException& ex)
  {
    // this should not happen as both topic and TF are published at the same time
    ROS_WARN("Fail to get odom pose");
    return;
  }

  last_processed_ = odom_time;

  /*
  tf2::Stamped<tf2::Transform> amcl;
  try
  {
    tf2::fromMsg(tf_.lookupTransform("map", "wheel", odom_time, ros::Duration(0.05)), amcl);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Fail to get amcl pose");
    return;
  }
  */

  tf2::Transform odom_delta = last_odom_.inverseTimes(odom_wheel);
  last_odom_ = odom_wheel;
  double odom_d[3];
  convertTfToXYThetaDeg(odom_delta, odom_d);

  /*
  tf2::Transform amcl_delta = last_amcl_.inverseTimes(amcl);
  last_amcl_ = amcl;
  double amcl_d[3];
  convertTfToXYThetaDeg(amcl_delta, amcl_d);
  */

  tf2::Transform odom_base;
  tf2::fromMsg(odom.pose.pose, odom_base);

  if (!initialized_)
  {
    odom_to_base_ = odom_base;
    initialized_ = true;
    return;
  }

  tf2::Transform base_delta = odom_to_base_.inverseTimes(odom_base);
  odom_to_base_ = odom_base;
  double base_d[3];
  convertTfToXYThetaDeg(base_delta, base_d);

  double linear_error = (odom_d[0] - base_d[0]) / dt;
  double angular_error = (odom_d[2] - base_d[2]) / dt;

  updateSlippageError(linear_error, angular_error);

  /*
  ROS_INFO("Delta:");
  ROS_INFO("  - PL-ICP (%f, %f, %f)", icp_d[0], icp_d[1], icp_d[2]);
  ROS_INFO("  - Odom   (%f, %f, %f)", odom_d[0], odom_d[1], odom_d[2]);
  ROS_INFO("  - AMCL   (%f, %f, %f)", amcl_d[0], amcl_d[1], amcl_d[2]);
  */
}


StateFilterICP::StateFilterICP(StateFilterBase& base, const std::string& topic):
  StateFilterBaseSensor(base, topic),
  laser_scan_sub_(base.nh_, topic, 100),
  laser_scan_filter_(laser_scan_sub_, tf_, "odom", 100, base.nh_)
{
  // dynamic reconfigure server
  handleConfig();

  // publisher
  odom_pub_ = base.nh_.advertise<nav_msgs::Odometry>("odom/" + topic, 1, true);

  // subscriber
  laser_scan_filter_.registerCallback<sensor_msgs::LaserScan>(boost::bind(&StateFilterICP::handleScan, this, _1));

  // initialize variables
  sm_input_.laser[0] = 0.0;
  sm_input_.laser[1] = 0.0;
  sm_input_.laser[2] = 0.0;

  // initialize output vectors as Null for error-checking
  sm_output_.cov_x_m = 0;
  sm_output_.dx_dy1_m = 0;
  sm_output_.dx_dy2_m = 0;

  // initialize odom
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base";

  // initialize odom pose covariance (required if using robot_pose_ekf)
  odom_.pose.covariance[0] = 0.5 * 0.5;
  odom_.pose.covariance[7] = 0.5 * 0.5;
  odom_.pose.covariance[35] = (M_PI / 12) * (M_PI / 12);

  odom_.pose.covariance[14] = DBL_MAX;  // set a non-zero covariance on unused
  odom_.pose.covariance[21] = DBL_MAX;  // dimensions (z, pitch and roll); this
  odom_.pose.covariance[28] = DBL_MAX;  // is a requirement of robot_pose_ekf
}

void StateFilterICP::handleConfig()
{
  sm_input_.max_angular_correction_deg = config_.max_angular_correction_deg;
  sm_input_.max_linear_correction = config_.max_linear_correction;

  sm_input_.max_iterations = config_.max_iterations;
  sm_input_.epsilon_xy = config_.epsilon_xy;
  sm_input_.epsilon_theta = config_.epsilon_theta;

  sm_input_.max_correspondence_dist = config_.max_correspondence_dist;
  sm_input_.use_corr_tricks = config_.use_corr_tricks;

  sm_input_.restart = config_.restart;
  sm_input_.restart_threshold_mean_error = config_.restart_threshold_mean_error;
  sm_input_.restart_dt = config_.restart_dt;
  sm_input_.restart_dtheta = config_.restart_dtheta;

  sm_input_.outliers_maxPerc = config_.outliers_maxPerc;
  sm_input_.outliers_adaptive_order = config_.outliers_adaptive_order;
  sm_input_.outliers_adaptive_mult = config_.outliers_adaptive_mult;
  sm_input_.outliers_remove_doubles = config_.outliers_remove_doubles;

  sm_input_.clustering_threshold = config_.clustering_threshold;
  sm_input_.orientation_neighbourhood = config_.orientation_neighbourhood;

  sm_input_.do_alpha_test = config_.do_alpha_test;
  sm_input_.do_alpha_test_thresholdDeg = config_.do_alpha_test_thresholdDeg;
  sm_input_.do_visibility_test = config_.do_visibility_test;

  sm_input_.use_point_to_line_distance = config_.use_point_to_line_distance;
  sm_input_.use_ml_weights = config_.use_ml_weights;
  sm_input_.use_sigma_weights = config_.use_sigma_weights;

  sm_input_.do_compute_covariance = config_.do_compute_covariance;
  sm_input_.debug_verify_tricks = config_.debug_verify_tricks;

  sm_input_.sigma = config_.sigma;

  StateFilterBaseSensor::handleConfig();
}

void StateFilterICP::handleScan(const sensor_msgs::LaserScan& scan)
{
  ros::Time scan_time = scan.header.stamp;
  ros::Time now = ros::Time::now();

  double dt = (scan_time - last_processed_).toSec();
  if (dt < (1.0 / config_.process_rate))
  {
    return;
  }
  last_processed_ = scan_time;

  ros::Duration scan_age = now - scan_time;
  if (scan_age.toSec() > 1.0)
  {
    ROS_WARN("%s SCAN SEEMS TOO OLD (%f seconds, 1.0 threshold) scan time: %f, now %f", scan.header.frame_id.c_str(), scan_age.toSec(), scan_time.toSec(), now.toSec());
    // Don't error out before caching odom and laser data
  }

  tf2::Stamped<tf2::Transform> odom;
  try
  {
    tf2::fromMsg(tf_.lookupTransform("odom", "wheel", scan_time, ros::Duration(0.05)), odom);
  }
  catch (tf2::TransformException& ex)
  {
    // this should not happen with proper setup of tf2_ros::MessageFilter
    ROS_WARN("Fail to get odom pose at %s scan time", scan.header.frame_id.c_str());
    return;
  }

  /*
  tf2::Stamped<tf2::Transform> amcl;
  try
  {
    tf2::fromMsg(tf_.lookupTransform("map", "wheel", scan_time, ros::Duration(0.05)), amcl);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Fail to get amcl pose at %s scan time", scan.header.frame_id.c_str());
    return;
  }
  */

  tf2::Transform odom_delta = last_odom_.inverseTimes(odom);
  last_odom_ = odom;
  double odom_d[3];
  convertTfToXYThetaDeg(odom_delta, odom_d);

  /*
  tf2::Transform amcl_delta = last_amcl_.inverseTimes(amcl);
  last_amcl_ = amcl;
  double amcl_d[3];
  convertTfToXYThetaDeg(amcl_delta, amcl_d);
  */

  if (!initialized_)
  {
    sm_input_.min_reading = scan.range_min;
    sm_input_.max_reading = scan.range_max;

    if (!getBaseToLaserTf(scan.header.frame_id))
    {
      ROS_WARN("Skipping %s scan", scan.header.frame_id.c_str());
      return;
    }

    laserScanToLDP(scan, prev_laser_data_);
    odom_to_base_ = odom;
    odom_.header.stamp = scan_time;
    initialized_ = true;
    return;
  }

  LDP laser_data;
  laserScanToLDP(scan, laser_data);

  tf2::Transform icp_delta;
  if (!scanICP(laser_data, odom_delta, icp_delta))
  {
    if (config_.use_odometry_guess)
    {
      publishOdom(scan_time, odom_delta);
    }
    updateIcpError(true);
    return;
  }
  updateIcpError(false);

  double icp_d[3];
  convertTfToXYThetaDeg(icp_delta, icp_d);

  double linear_error = (odom_d[0] - icp_d[0]) / dt;
  double angular_error = (odom_d[2] - icp_d[2]) / dt;

  updateSlippageError(linear_error, angular_error);

  /*
  ROS_INFO("Delta:");
  ROS_INFO("  - PL-ICP (%f, %f, %f)", icp_d[0], icp_d[1], icp_d[2]);
  ROS_INFO("  - Odom   (%f, %f, %f)", odom_d[0], odom_d[1], odom_d[2]);
  ROS_INFO("  - AMCL   (%f, %f, %f)", amcl_d[0], amcl_d[1], amcl_d[2]);
  */

  // publish odom
  publishOdom(scan_time, icp_delta);
}

bool StateFilterICP::scanICP(LDP& laser_data, const tf2::Transform& odom_delta, tf2::Transform& corr_ch)
{
  ros::WallTime start = ros::WallTime::now();

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_laser_data_->odometry[0] = 0.0;
  prev_laser_data_->odometry[1] = 0.0;
  prev_laser_data_->odometry[2] = 0.0;

  prev_laser_data_->estimate[0] = 0.0;
  prev_laser_data_->estimate[1] = 0.0;
  prev_laser_data_->estimate[2] = 0.0;

  prev_laser_data_->true_pose[0] = 0.0;
  prev_laser_data_->true_pose[1] = 0.0;
  prev_laser_data_->true_pose[2] = 0.0;

  sm_input_.laser_ref  = prev_laser_data_;
  sm_input_.laser_sens = laser_data;

  // the predicted change of the laser's position, in the laser frame
  tf2::Transform laser_delta = laser_to_base_ * odom_delta * base_to_laser_;

  if (config_.use_odometry_guess)
  {
    sm_input_.first_guess[0] = laser_delta.getOrigin().getX();
    sm_input_.first_guess[1] = laser_delta.getOrigin().getY();
    sm_input_.first_guess[2] = tf2::getYaw(laser_delta.getRotation());
  }
  else
  {
    sm_input_.first_guess[0] = 0;
    sm_input_.first_guess[1] = 0;
    sm_input_.first_guess[2] = 0;
  }

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (sm_output_.cov_x_m)
  {
    gsl_matrix_free(sm_output_.cov_x_m);
    sm_output_.cov_x_m = 0;
  }
  if (sm_output_.dx_dy1_m)
  {
    gsl_matrix_free(sm_output_.dx_dy1_m);
    sm_output_.dx_dy1_m = 0;
  }
  if (sm_output_.dx_dy2_m)
  {
    gsl_matrix_free(sm_output_.dx_dy2_m);
    sm_output_.dx_dy2_m = 0;
  }

  // scan match - using point to line icp from CSM
  sm_icp(&sm_input_, &sm_output_);

  // swap old and new
  ld_free(prev_laser_data_);
  prev_laser_data_ = laser_data;

  /*
  // **** statistics
  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_INFO("Scan matcher total duration: %.1f ms (%d iterations) - %.2f%% valid", dur, sm_output_.iterations, 100.0 * sm_output_.nvalid / laser_data->nrays);
  */

  if (!sm_output_.valid)
  {
    ROS_WARN("Error in %s matching", getTopic());
    return false;
  }
  if (sm_output_.nvalid < laser_data->nrays * config_.inliers_minPerc)
  {
    ROS_WARN("Low correspondence between %s laser scans", getTopic());
    return false;
  }

  // the correction of the laser's position, in the laser frame
  tf2::Transform corr_ch_l;
  createTfFromXYTheta(sm_output_.x[0], sm_output_.x[1], sm_output_.x[2], corr_ch_l);

  // the correction of the base's position, in the base frame
  corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

  return true;
}

bool StateFilterICP::getBaseToLaserTf(const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf2::Stamped<tf2::Transform> base_to_laser_tf;
  try
  {
    tf2::fromMsg(tf_.lookupTransform("base", frame_id, t, ros::Duration(1.0)), base_to_laser_tf);
  }
  catch (tf2::TransformException ex)
  {
    ROS_WARN("Could not get initial transform from base to %s frame, %s", frame_id.c_str(), ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_tf.inverse();
  return true;
}


void StateFilterICP::laserScanToLDP(const sensor_msgs::LaserScan& scan, LDP& ldp)
{
  unsigned int n = scan.ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    double r = scan.ranges[i];
    if (r > scan.range_min && r < scan.range_max)
    {
      // fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }
    ldp->theta[i]    = scan.angle_min + i * scan.angle_increment;
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void StateFilterICP::publishOdom(const ros::Time& now, const tf2::Transform& delta)
{
  tf2::Quaternion q;
  double x, y, theta;
  double rate = 1.0 / (now - odom_.header.stamp).toSec();
  odom_.header.stamp = now;

  // pose
  odom_to_base_ *= delta;
  convertTfToXYTheta(odom_to_base_, x, y, theta);
  odom_.pose.pose.position.x = x;
  odom_.pose.pose.position.y = y;
  q.setRPY(0, 0, theta);
  odom_.pose.pose.orientation = tf2::toMsg(q);

  // velocity (required if using dwa_local_planner)
  convertTfToXYTheta(delta, x, y, theta);
  odom_.twist.twist.linear.x = x * rate;
  odom_.twist.twist.linear.y = y * rate;  // check if omni?
  odom_.twist.twist.angular.z = theta * rate;
  odom_pub_.publish(odom_);
}


StateFilter::StateFilter(const std::string& topics_param_name)
{
  // dynamic reconfigure server
  config_.error_average_window = 1;
  ds_.setCallback(boost::bind(&StateFilter::handleConfig, this, _1, _2));

  // publisher
  wheel_slippage_pub_ = nh_.advertise<std_msgs::Bool>("agv05/safety/wheel_slippage", 1, true);
  safety_heartbeat_pub_ = nh_.advertise<std_msgs::UInt8>("agv05/safety/heartbeat/" + ros::this_node::getName(), 1);

  // diagnostic
  updater_.setHardwareID("AGV05");
  updater_.add("Status", this, &StateFilter::statusDiagnostic);

  boost::shared_ptr<StateFilterOdom> odom_filter;
  odom_filter.reset(new StateFilterOdom(*this, "odom"));
  filters_.push_back(odom_filter);
  // TODO(someday): split out StateFilterICP just to publish "odom/scan" topics which to be subscribed here

  std::string topics;
  if (obtainParam(nh_, topics_param_name, topics))
  {
    std::istringstream iss(topics);
    std::string topic;
    while (iss >> topic)
    {
      boost::shared_ptr<StateFilterICP> filter;
      filter.reset(new StateFilterICP(*this, topic));
      filters_.push_back(filter);
    }

    ROS_INFO("agv05_state_filter started with scan topics: %s", topics.c_str());
  }
  else
  {
    ROS_ERROR("agv05_state_filter started without any scan topic");
  }
}

void StateFilter::handleConfig(agv05_state_filter::StateFilterConfig& config, uint32_t level)
{
  config_ = config;

  for (std::vector< boost::shared_ptr<StateFilterBaseSensor> >::iterator it = filters_.begin(); it != filters_.end(); it++)
  {
    (*it)->handleConfig();
  }
}

void StateFilter::statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  for (std::vector< boost::shared_ptr<StateFilterBaseSensor> >::iterator it = filters_.begin(); it != filters_.end(); it++)
  {
    (*it)->statusDiagnostic(stat);
  }

  const char *error;
  if ((error = getScanIcpError()))
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Scan Matching Error (%s)", error);
  }
  else if ((error = getLinearError()))
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Translational Slippage (%s)", error);
  }
  else if ((error = getAngularError()))
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Rotational Slippage (%s)", error);
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status OK");
  }
}

bool StateFilter::obtainParam(ros::NodeHandle& nh, const std::string& suffix, std::string& param_value)
{
  std::string param_name = nh.resolveName(suffix);

  nh.getParam(param_name, param_value);
  if (param_value.empty())
  {
    std::vector<std::string> params;
    nh.getParamNames(params);

    for (std::vector<std::string>::const_iterator it = params.begin(); it != params.end(); it++)
    {
      param_name = *it;
      if ((param_name.size() >= suffix.size()) &&
          (param_name.compare(param_name.size() - suffix.size(), suffix.size(), suffix) == 0))
      {
        nh.getParam(param_name, param_value);
        if (!param_value.empty()) break;
      }
    }
  }

  return !param_value.empty();
}

}  // namespace agv05

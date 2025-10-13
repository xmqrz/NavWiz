/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#ifndef AGV05_LINE_SENSOR_AGV05_LINE_SENSOR_H_
#define AGV05_LINE_SENSOR_AGV05_LINE_SENSOR_H_

#include <agv05_variable_storage/variable_storage.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <agv05_line_sensor/LineSensorConfig.h>
#include <agv05_msgs/LineSensor.h>
#include <agv05_msgs/LineSensorActivation.h>
#include <agv05_msgs/MsbRaw.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>


namespace agv05
{

struct LineSensorConfig
{
  bool enable;
  uint8_t polarity;
  float distance_to_center;
  float line_width;
  float junction_width;
  float activation_percentage;
  float iir_percentage;
  float timeout;

  bool duo;
  float duo_distance;
};

struct LineSensorKeyNames
{
  /* display name in diagnostic status */
  std::string name;

  /* variable storage keys */
  std::string min;
  std::string max;

  /* topic names */
  std::string raw;
  std::string raw_rate;
  std::string prefer_side;
  std::string calibration;
  std::string output;
  std::string activation;
};


class LineSensor
{
  enum { NORTH = 1, SOUTH = 2 };
  enum { PREFER_CENTER = 0, PREFER_LEFT, PREFER_RIGHT, PREFER_N };

public:
  LineSensor(const LineSensorKeyNames& keys, const LineSensor& peer,
             bool main, diagnostic_updater::Updater& updater);

  void loadCalibration(const std::string& key_min, const std::string& key_max);
  void writeCalibration();

  void checkTimeout();
  void tick();

private:
  void loadCalibration0(const std::string& key, std::vector<uint16_t>& data);
  void writeCalibration0(const std::string& key, const std::vector<uint16_t>& data);
  void computeCalibrationRange();

public:
  void callbackMsbRaw(const agv05_msgs::MsbRawConstPtr& msg);

  void callbackMsbRawRate(const std_msgs::Float32& msg)
  {
    raw_rate_ = std::max(10.0f, msg.data);
    if (config_.enable)
    {
      expected_process_frequency_ = raw_rate_;
    }
  }

  void callbackPreferSide(const std_msgs::UInt8& msg)
  {
    if (msg.data < PREFER_N)
    {
      prefer_side_ = msg.data;
    }
  }

  void callbackCalibration(const std_msgs::Bool& msg)
  {
    calibration_ = msg.data;
  }

  void callbackConfig(const LineSensorConfig& config);
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  std::string name_;
  const LineSensor& peer_;
  bool main_;

  /* ROS publishers */
  ros::Publisher output_pub_;
  ros::Publisher activation_pub_;

  /* ROS subscribers */
  ros::Subscriber msb_raw_sub_;
  ros::Subscriber msb_raw_rate_sub_;
  ros::Subscriber prefer_side_sub_;
  ros::Subscriber calibration_sub_;

  /* ROS diagnostics */
  double expected_process_frequency_;
  diagnostic_updater::FrequencyStatus diagnostic_frequency_;

  /* Variable storage */
  agv05::VariableStorage variable_storage_;

  /* Config */
  LineSensorConfig config_;
  std::string key_min_;  // variable storage key for calibrated values
  std::string key_max_;
  std::vector<uint16_t> min_;  // calibrated min readings
  std::vector<uint16_t> max_;  // calibrated max readings
  std::vector<uint16_t> range_;  // calibrated range (max - min)

  /* Inputs */
  agv05_msgs::MsbRawConstPtr raw_;
  float sensor_separation_distance_;
  float raw_rate_;
  uint8_t prefer_side_;
  bool calibration_;

  /* Outputs */
  ros::Time last_update_;
  agv05_msgs::LineSensor output_;
  agv05_msgs::LineSensorActivation activation_;
  bool calibration_active_;
  bool calibration_mismatch_;  // calibration data size does not match raw data size
};


class Agv05LineSensor
{
public:
  Agv05LineSensor();

private:
  void timerCallback(const ros::TimerEvent& event)
  {
    // 0.1s timer
    front_sensor_.checkTimeout();
    front_sensor2_.checkTimeout();
    rear_sensor_.checkTimeout();
    rear_sensor2_.checkTimeout();
  }

  void timer2Callback(const ros::TimerEvent& event)
  {
    // 1.0s timer
    front_sensor_.tick();
    front_sensor2_.tick();
    rear_sensor_.tick();
    rear_sensor2_.tick();

    diagnostic_updater_.update();
  }

  void callbackConfig(agv05_line_sensor::LineSensorConfig& config, uint32_t level)
  {
    // disabled by parameter (set by hardware node)
    if (ros::param::param<bool>("~disabled", false))
    {
      config.front_sensor_enable = false;
      config.rear_sensor_enable = false;
    }

    LineSensorConfig cfg;
    cfg.enable = config.front_sensor_enable;
    cfg.polarity = config.front_sensor_polarity;
    cfg.distance_to_center = config.front_sensor_distance_to_center;
    cfg.line_width = static_cast<float>(config.front_sensor_line_width);
    cfg.junction_width = static_cast<float>(config.front_sensor_junction_width);
    cfg.activation_percentage = config.front_sensor_activation_percentage;
    cfg.iir_percentage = config.front_sensor_iir_percentage;
    cfg.timeout = config.front_sensor_timeout;
    cfg.duo = config.front_sensor_duo;
    cfg.duo_distance = config.front_sensor_duo_distance;
    front_sensor_.callbackConfig(cfg);
    front_sensor2_.callbackConfig(cfg);

    cfg.enable = config.rear_sensor_enable;
    cfg.polarity = config.rear_sensor_polarity;
    cfg.distance_to_center = config.rear_sensor_distance_to_center;
    cfg.line_width = static_cast<float>(config.rear_sensor_line_width);
    cfg.junction_width = static_cast<float>(config.rear_sensor_junction_width);
    cfg.activation_percentage = config.rear_sensor_activation_percentage;
    cfg.iir_percentage = config.rear_sensor_iir_percentage;
    cfg.timeout = config.rear_sensor_timeout;
    cfg.duo = config.rear_sensor_duo;
    cfg.duo_distance = config.rear_sensor_duo_distance;
    rear_sensor_.callbackConfig(cfg);
    rear_sensor2_.callbackConfig(cfg);
  }

private:
  /* Dynamic reconfigure server */
  dynamic_reconfigure::Server<agv05_line_sensor::LineSensorConfig> ds_;

  /* ROS diagnostic */
  diagnostic_updater::Updater diagnostic_updater_;

  /* Timers */
  ros::Timer timer_;
  ros::Timer timer2_;

  /* Data */
  LineSensor front_sensor_;
  LineSensor front_sensor2_;
  LineSensor rear_sensor_;
  LineSensor rear_sensor2_;

  /* Const */
  static const LineSensorKeyNames front_sensor_keys_;
  static const LineSensorKeyNames front_sensor2_keys_;
  static const LineSensorKeyNames rear_sensor_keys_;
  static const LineSensorKeyNames rear_sensor2_keys_;
};


class DisabledAgv05LineSensor
{
public:
  DisabledAgv05LineSensor()
  {
    // diagnostic updater
    diagnostic_updater_.setHardwareID("AGV05");
    diagnostic_updater_.add("Status", this, &DisabledAgv05LineSensor::diagnosticStatus);

    // timer
    ros::NodeHandle nh;
    timer_ = nh.createTimer(ros::Duration(1.0), &DisabledAgv05LineSensor::timerCallback, this);
  }

private:
  void timerCallback(const ros::TimerEvent& event)
  {
    diagnostic_updater_.update();
  }

  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disabled");
  }

private:
  /* ROS diagnostic */
  diagnostic_updater::Updater diagnostic_updater_;

  /* Timers */
  ros::Timer timer_;
};

}  // namespace agv05

#endif  // AGV05_LINE_SENSOR_AGV05_LINE_SENSOR_H_

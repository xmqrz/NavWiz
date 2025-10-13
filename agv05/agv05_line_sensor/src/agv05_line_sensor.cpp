/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#include "agv05_line_sensor/agv05_line_sensor.h"
#include <algorithm>
#include <math.h>  // std::round not available in cmath in C++03

#define LOOP_FREQUENCY 100


namespace agv05
{

/* LineSensor class */
LineSensor::LineSensor(const LineSensorKeyNames& keys, const LineSensor& peer,
                       bool main, diagnostic_updater::Updater& updater) :
  name_(keys.name), peer_(peer), main_(main),
  expected_process_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_process_frequency_,
                        &expected_process_frequency_, 0.1, 1), keys.name + " Frequency"),
  sensor_separation_distance_(0.01f),  // default to 1 cm
  raw_rate_(LOOP_FREQUENCY),
  prefer_side_(PREFER_CENTER),
  calibration_(false),
  calibration_active_(false),
  calibration_mismatch_(false)
{
  // load calibration
  loadCalibration(keys.min, keys.max);

  ros::NodeHandle nh_line("agv05/line_sensor");
  ros::NodeHandle nh_msb("agv05/msb");

  // ROS publishers
  output_pub_ = nh_line.advertise<agv05_msgs::LineSensor>(keys.output, 1, true);
  activation_pub_ = nh_line.advertise<agv05_msgs::LineSensorActivation>(keys.activation, 1);

  // ROS subscribers
  msb_raw_sub_ = nh_msb.subscribe(keys.raw, 10, &LineSensor::callbackMsbRaw, this);
  msb_raw_rate_sub_ = nh_msb.subscribe(keys.raw_rate, 1, &LineSensor::callbackMsbRawRate, this);
  prefer_side_sub_ = nh_line.subscribe(keys.prefer_side, 1, &LineSensor::callbackPreferSide, this);
  calibration_sub_ = nh_line.subscribe(keys.calibration, 1, &LineSensor::callbackCalibration, this);

  // diagnostics
  updater.add(diagnostic_frequency_);
  updater.add(keys.name + " Status", this, &LineSensor::diagnosticStatus);
}

void LineSensor::loadCalibration(const std::string& key_min, const std::string& key_max)
{
  key_min_ = key_min;
  key_max_ = key_max;
  loadCalibration0(key_min_, min_);
  loadCalibration0(key_max_, max_);

  if (min_.size() == max_.size())
  {
    computeCalibrationRange();
  }
  else
  {
    min_.clear();
    max_.clear();
    range_.clear();
  }
}

void LineSensor::writeCalibration()
{
  ROS_ASSERT(min_.size() == max_.size());
  writeCalibration0(key_min_, min_);
  writeCalibration0(key_max_, max_);
  computeCalibrationRange();
}

void LineSensor::computeCalibrationRange()
{
  range_.resize(min_.size());
  for (int i = 0, n = min_.size(); i < n; ++i)
  {
    if (min_[i] > max_[i]) std::swap(min_[i], max_[i]);
    range_[i] = max_[i] - min_[i];
  }
}

void LineSensor::loadCalibration0(const std::string& key, std::vector<uint16_t>& data)
{
  data.clear();
  std::string value;
  if (variable_storage_.getVariable(key, value))
  {
    std::istringstream iss(value);
    uint16_t v;

    while (iss >> v)
    {
      data.push_back(v);
    }
  }
}

void LineSensor::writeCalibration0(const std::string& key, const std::vector<uint16_t>& data)
{
  std::ostringstream oss;
  for (std::vector<uint16_t>::const_iterator it = data.begin(), it_end = data.end(); it != it_end; ++it)
  {
    oss << " " << *it;
  }
  variable_storage_.setVariable(key, oss.str());
}

void LineSensor::checkTimeout()
{
  if (config_.enable)
  {
    if ((ros::Time::now() - last_update_).toSec() > config_.timeout && !output_.sensor_error)
    {
      output_.sensor_error = agv05_msgs::LineSensor::COMMUNICATION_ERROR;
      output_pub_.publish(output_);
    }
  }
}

void LineSensor::tick()
{
  if (!config_.enable)
  {
    diagnostic_frequency_.tick();
  }
}

void LineSensor::callbackMsbRaw(const agv05_msgs::MsbRawConstPtr& msg)
{
  if (!config_.enable) return;

  raw_ = msg;
  const std::vector<uint16_t>& raw = raw_->raw;
  if (raw.size() == 0) return;

  diagnostic_frequency_.tick();

  calibration_mismatch_ = raw.size() != min_.size();
  bool calibration_pending = calibration_ && !calibration_active_;

  if (calibration_mismatch_ && !calibration_pending)
  {
    calibration_active_ = false;
    return;
  }

  last_update_ = ros::Time::now();

  // calibration - get min and max values
  if (calibration_)
  {
    if (!calibration_active_)
    {
      min_ = raw;
      max_ = raw;
      calibration_active_ = true;
    }
    else
    {
      std::vector<uint16_t>::iterator p_min = min_.begin();
      std::vector<uint16_t>::iterator p_max = max_.begin();
      for (std::vector<uint16_t>::const_iterator it = raw.begin(), it_end = raw.end();
           it != it_end; ++it, ++p_min, ++p_max)
      {
        *p_min = std::min(*p_min, *it);
        *p_max = std::max(*p_max, *it);
      }
    }
    return;
  }
  else if (calibration_active_)
  {
    writeCalibration();
    calibration_active_ = false;
  }

  // continue normal processing
  // convert raw values to percentage
  std::vector<float> normalized;
  normalized.resize(raw.size());
  for (int i = 0, size = raw.size(); i < size; ++i)
  {
    float n = raw[i] - min_[i];
    float m = range_[i];
    normalized[i] = std::min(std::max(n / m, 0.0f), 1.0f);
  }

  // compute activation data and expand it by inserting 2 new values in between each data
  std::vector<uint8_t>& activation = activation_.activation;
  activation.resize(raw.size() * 3 - 2);
  if (config_.polarity == NORTH)
  {
    float threshold = 1.0f - config_.activation_percentage;
    float threshold3 = threshold * 3.0f;

    std::vector<float>::const_iterator p_norm = normalized.begin(), p_end = normalized.end();
    std::vector<uint8_t>::iterator it = activation.begin();

    uint8_t prev = *p_norm < threshold;
    *it++ = prev;

    for (++p_norm; p_norm != p_end; ++p_norm)
    {
      uint8_t cur = *p_norm < threshold;
      if (prev == cur)  // both adjacent data have the same activation
      {
        // the 2 new values will have the same activation
        *it++ = cur;
        *it++ = cur;
        *it++ = cur;
      }
      else
      {
        // compute the 2 new values according to ratio
        *it++ = *(p_norm - 1) * 2 + *p_norm < threshold3;
        *it++ = *(p_norm - 1) + *p_norm * 2 < threshold3;
        *it++ = cur;
      }
      prev = cur;
    }
  }
  else  // SOUTH
  {
    float threshold = config_.activation_percentage;
    float threshold3 = threshold * 3.0f;

    std::vector<float>::const_iterator p_norm = normalized.begin(), p_end = normalized.end();
    std::vector<uint8_t>::iterator it = activation.begin();

    uint8_t prev = *p_norm > threshold;
    *it++ = prev;

    for (++p_norm; p_norm != p_end; ++p_norm)
    {
      uint8_t cur = *p_norm > threshold;
      if (prev == cur)  // both adjacent data have the same activation
      {
        // the 2 new values will have the same activation
        *it++ = cur;
        *it++ = cur;
        *it++ = cur;
      }
      else
      {
        // compute the 2 new values according to ratio
        *it++ = *(p_norm - 1) * 2 + *p_norm > threshold3;
        *it++ = *(p_norm - 1) + *p_norm * 2 > threshold3;
        *it++ = cur;
      }
      prev = cur;
    }
  }
  if (activation_pub_.getNumSubscribers() > 0)
  {
    activation_pub_.publish(activation_);
  }

  // check for junction
  sensor_separation_distance_ = msg->sensor_separation_distance;
  if (sensor_separation_distance_ == 0)
  {
    sensor_separation_distance_ = 0.01f;  // default to 1 cm
  }
  int junction_width = std::ceil(config_.junction_width / sensor_separation_distance_ * 3);
  if (std::count(activation.begin(), activation.end(), 1) >= junction_width)
  {
    // junction detected
    output_.linear_error = 0.0f;
    output_.angular_error = 0.0f;
    output_.heading_error = 0.0f;
    output_.out_of_line = false;
    output_.on_junction = true;
    output_.sensor_error = agv05_msgs::LineSensor::NORMAL;
    output_pub_.publish(output_);
    return;
  }

  // TODO(someone): activation filter
  // set activation to 1 if most of the neighbour activation is set but not the middle one

  // find the line position
  // multiply every sensor position by 2 so that the center pos can be represented in integer
  int line_width = static_cast<int>(std::ceil(config_.line_width / sensor_separation_distance_ * 3)) << 1;
  int center = activation.size() - 1;  // center position multiplied by 2
  int line_pos = 0;
  int start = -1;
  bool line_found = false;

  // take previous line pos as reference
  int prev_line_pos = 0;
  int prev_to_cur = 10000;  // to minimize distance from prev to cur line
  if (prefer_side_ == PREFER_CENTER && !output_.out_of_line && !output_.sensor_error)
  {
    prev_line_pos = round(output_.linear_error * sensor_separation_distance_ * 3 * 2);
  }

  for (int i = 0, n = activation.size(); i < n + 1; ++i)
  {
    if (i < n && activation[i])
    {
      if (start < 0)
      {
        start = i << 1;  // multiply by 2
      }
    }
    else if (start >= 0)
    {
      // found line, i is one index past the line
      line_found = true;
      int end = i << 1;  // multiply by 2

      // line width is too small (give exception if it is located on the edge of sensor)
      if (start != 0 && i != n && end - start < line_width)
      {
        start = -1;
        continue;
      }

      int cur_line_pos = (start + end - 2) >> 1;
      cur_line_pos -= center;

      if (prefer_side_ == PREFER_CENTER)
      {
        // take the line closest to previous line reference
        int absdiff = std::abs(cur_line_pos - prev_line_pos);
        if (absdiff < prev_to_cur)
        {
          line_pos = cur_line_pos;
          prev_to_cur = absdiff;
        }
      }
      else
      {
        line_pos = cur_line_pos;
        if (prefer_side_ == PREFER_LEFT) break;  // take the first left line
      }
      start = -1;
    }
  }

  // compute linear and angular error
  if (line_found)
  {
    float linear_error = line_pos * sensor_separation_distance_ / 3 / 2;
    if (output_.out_of_line || output_.sensor_error)  // was error, ignore previous value
    {
      output_.linear_error = linear_error;
    }
    else
    {
      output_.linear_error += (linear_error - output_.linear_error) * config_.iir_percentage;
    }
    output_.angular_error = std::atan2(output_.linear_error, config_.distance_to_center);
    output_.heading_error = 0.0f;
    if (main_ && config_.duo)
    {
      // validate peer's output
      const agv05_msgs::LineSensor& po = peer_.output_;
      if (po.enable && !po.sensor_error && !po.out_of_line && !po.on_junction)
      {
        float linear_diff = output_.linear_error - po.linear_error;
        output_.heading_error = std::atan2(linear_diff, config_.duo_distance);
      }
    }
    output_.out_of_line = false;
    output_.on_junction = false;
    output_.sensor_error = agv05_msgs::LineSensor::NORMAL;
  }
  else
  {
    // out of line
    output_.linear_error = 0.0f;
    output_.angular_error = 0.0f;
    output_.heading_error = 0.0f;
    output_.out_of_line = true;
    output_.on_junction = false;
    output_.sensor_error = agv05_msgs::LineSensor::NORMAL;
  }
  output_pub_.publish(output_);
}

void LineSensor::callbackConfig(const LineSensorConfig& config)
{
  ROS_INFO("agv05_line_sensor: config received");
  config_ = config;

  if (!main_)
  {
    config_.enable = config_.enable && config_.duo;
  }

  if (config_.enable)
  {
    output_.enable = true;
    output_.sensor_error = agv05_msgs::LineSensor::COMMUNICATION_ERROR;
    expected_process_frequency_ = raw_rate_;
  }
  else
  {
    output_.linear_error = 0.0f;
    output_.angular_error = 0.0f;
    output_.heading_error = 0.0f;
    output_.out_of_line = false;
    output_.on_junction = false;
    output_.enable = false;
    output_.sensor_error = agv05_msgs::LineSensor::NORMAL;
    output_pub_.publish(output_);

    expected_process_frequency_ = 1.0;
  }

  config_.line_width /= 100.0f;  // convert from cm to m
  config_.junction_width /= 100.0f;
  config_.activation_percentage /= 100.0f;  // convert from % to decimal
  config_.iir_percentage /= 100.0f;
}

void LineSensor::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::ostringstream oss_raw;
  std::ostringstream oss_min;
  std::ostringstream oss_max;
  std::ostringstream oss_activ;

  if (!output_.enable || !raw_)
  {
    oss_raw << "-";
    oss_min << "-";
    oss_max << "-";
    oss_activ << "-";
  }
  else if (output_.sensor_error || calibration_mismatch_)
  {
    const std::vector<uint16_t>& raw = raw_->raw;
    int n = raw.size();
    int divider = ((n & 3) == 0) ? n >> 2 : ((n & 1) == 0) ? n >> 1 : n;

    for (int i = 0; i < n; ++i)
    {
      if (i && i % divider == 0)
      {
        oss_raw << ": ";
      }
      oss_raw << raw[i] << " ";
    }
    oss_min << "-";
    oss_max << "-";
    oss_activ << "-";
  }
  else
  {
    const std::vector<uint16_t>& raw = raw_->raw;
    int n = raw.size();
    int divider = ((n & 3) == 0) ? n >> 2 : ((n & 1) == 0) ? n >> 1 : n;
    std::vector<uint8_t>::const_iterator p_activ = activation_.activation.begin();

    for (int i = 0; i < n; ++i)
    {
      if (i && i % divider == 0)
      {
        oss_raw << ": ";
        oss_min << ": ";
        oss_max << ": ";
        oss_activ << ": ";
      }
      oss_raw << raw[i] << " ";
      oss_min << min_[i] << " ";
      oss_max << max_[i] << " ";

      if (i && i < n - 1) oss_activ << "01"[*p_activ++];
      oss_activ << "01"[*p_activ++] << "01"[*p_activ++] << " ";
    }
  }

  stat.add(name_ + " Enable", config_.enable);
  stat.add(name_ + " Error", static_cast<int>(output_.sensor_error));
  stat.addf(name_ + " Linear Error (m)", "%.4f", output_.linear_error);
  stat.addf(name_ + " Angular Error (rad)", "%.4f", output_.angular_error);
  stat.addf(name_ + " Heading Error (rad)", "%.4f", output_.heading_error);
  stat.add(name_ + " Out of Line", static_cast<bool>(output_.out_of_line));
  stat.add(name_ + " On Junction", static_cast<bool>(output_.on_junction));
  stat.add(name_ + " Calibrating", calibration_active_);
  stat.add(name_ + " Preferred Side", prefer_side_ == PREFER_CENTER ? "Center" : prefer_side_ == PREFER_LEFT ? "Left" : "Right");
  stat.addf(name_ + " Sensor Separation Distance (cm)", "%.2f", sensor_separation_distance_ * 100.0f);
  stat.add(name_ + " Line Width (N)", static_cast<int>(std::ceil(config_.line_width / sensor_separation_distance_)));
  stat.add(name_ + " Junction Width (N)", static_cast<int>(std::ceil(config_.junction_width / sensor_separation_distance_)));
  stat.add(name_ + " Raw", oss_raw.str());
  stat.add(name_ + " Min", oss_min.str());
  stat.add(name_ + " Max", oss_max.str());
  stat.add(name_ + " Activ", oss_activ.str());

  if (output_.sensor_error)
  {
    if (calibration_mismatch_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Calibration data size mismatch");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Sensor data timeout");
    }
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
}


/* Agv05LineSensor class */
Agv05LineSensor::Agv05LineSensor() :
  front_sensor_(front_sensor_keys_, front_sensor2_, true, diagnostic_updater_),
  front_sensor2_(front_sensor2_keys_, front_sensor_, false, diagnostic_updater_),
  rear_sensor_(rear_sensor_keys_, rear_sensor2_, true, diagnostic_updater_),
  rear_sensor2_(rear_sensor2_keys_, rear_sensor_, false, diagnostic_updater_)
{
  ros::NodeHandle nh_line("agv05/line_sensor");

  // dynamic reconfigure
  ds_.setCallback(boost::bind(&Agv05LineSensor::callbackConfig, this, _1, _2));

  // diagnostic updater
  diagnostic_updater_.setHardwareID("AGV05");

  // timer
  timer_ = nh_line.createTimer(ros::Duration(0.1), &Agv05LineSensor::timerCallback, this);
  timer2_ = nh_line.createTimer(ros::Duration(1.0), &Agv05LineSensor::timer2Callback, this);
}

const LineSensorKeyNames Agv05LineSensor::front_sensor_keys_ =
{
  "Front Sensor 1",
  "msb_front_raw_min", "msb_front_raw_max", "msb_front_raw", "msb_front_raw_rate",
  "front_prefer_side", "front_calibration", "front_sensor", "front_activation",
};

const LineSensorKeyNames Agv05LineSensor::front_sensor2_keys_ =
{
  "Front Sensor 2",
  "msb_front2_raw_min", "msb_front2_raw_max", "msb_front2_raw", "msb_front2_raw_rate",
  "front_prefer_side", "front_calibration", "front2_sensor", "front2_activation",
};

const LineSensorKeyNames Agv05LineSensor::rear_sensor_keys_ =
{
  "Rear Sensor 1",
  "msb_rear_raw_min", "msb_rear_raw_max", "msb_rear_raw", "msb_rear_raw_rate",
  "rear_prefer_side", "rear_calibration", "rear_sensor", "rear_activation",
};

const LineSensorKeyNames Agv05LineSensor::rear_sensor2_keys_ =
{
  "Rear Sensor 2",
  "msb_rear2_raw_min", "msb_rear2_raw_max", "msb_rear2_raw", "msb_rear2_raw_rate",
  "rear_prefer_side", "rear_calibration", "rear2_sensor", "rear2_activation",
};

}  // namespace agv05


// main function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_line_sensor");
  ROS_INFO_STREAM("agv05_line_sensor started.");

  // disabled by parameter (set by hardware node)
  if (ros::param::param<bool>("~disabled", false))
  {
    agv05::DisabledAgv05LineSensor no_line_sensor;
    ros::spin();
  }
  else
  {
    agv05::Agv05LineSensor line_sensor;
    ros::spin();
  }
}

/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include "agv05_nav/actions.h"


namespace agv05
{

void ActionTunePID::initialize()
{
  ros::NodeHandle nh("~");

  nh.param("tune_pid_ku", kp_, 10.0);
  nh.param("tune_pid_ki", ki_, 0.0);
  nh.param("tune_pid_kd", kd_, 0.0);
  nh.param("tune_pid_kh", kh_, 0.5);

  if (!nh.hasParam("tune_pid_kh"))
  {
    char *motor = getenv("ZALPHA_MOTOR");
    if (motor)
    {
      if (strstr(motor, "maxon"))
      {
        kh_ = 0.01;
      }
      else if (strstr(motor, "mabuchi"))
      {
        kd_ = 1.0;
        kh_ = 0.1;
      }
    }
  }

  error_ = std::vector<complex>();

  resetPose();
}

Goal& ActionTunePID::goal_init(Nav& nav, const Goal& goal)
{
  g_ = goal;
  g_.line_follow_type = 2;
  g_.speed = nav.config().straight_junction_min_speed;
  g_.enable_sensor = false;
  g_.io_trigger_type = 0;
  g_.error_io_trigger_type = 0;
  g_.next_motion = 0;
  return g_;
}

float ActionTunePID::computeTu()
{
  size_t i = 0, n = error_.size();
  while (n > 1)
  {
    n >>= 1;
    i++;
  }
  n <<= i;
  i = error_.size() - n;

  if (n > LOOP_FREQUENCY)
  {
    std::vector<complex> x(error_.begin() + i, error_.end());
    fft(x);

    float max = 0.0f;
    size_t m = 1;
    for (i = m; i < LOOP_FREQUENCY; i++)
    {
      float d = std::norm(x[i]);
      if (max < d)
      {
        max = d;
        m = i;
      }
    }

    float w[3] = {std::abs(x[m - 1]), std::abs(x[m]), std::abs(x[m + 1])};
    float count = (w[2] - w[0]) / (w[0] + w[1] + w[2]) + m;

    return 1.0f * n / LOOP_FREQUENCY / count;
  }
  return 0.0f;
}

void ActionTunePID::updatePID()
{
  char idx = '1' + config_.getProfileIndex();

  if (ros::service::exists("agv05_navx/set_parameters", false))
  {
    ros::NodeHandle nh;
    ros::ServiceClient cfg_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("agv05_navx/set_parameters");
    dynamic_reconfigure::Reconfigure srv;

    srv.request.config.doubles.resize(4);
    srv.request.config.doubles[0].name = std::string("path_follow_p") + idx;
    srv.request.config.doubles[0].value = kp_;
    srv.request.config.doubles[1].name = std::string("path_follow_i") + idx;
    srv.request.config.doubles[1].value = ki_;
    srv.request.config.doubles[2].name = std::string("path_follow_d") + idx;
    srv.request.config.doubles[2].value = kd_;
    srv.request.config.doubles[3].name = std::string("path_follow_ahead_distance") + idx;
    srv.request.config.doubles[3].value = round(config_.getProfile().line_follow_ahead_distance * 1e7) * 1e-7;

    cfg_client.call(srv);
    cfg_client.shutdown();
  }

  // self-service calling will cause ROS node hang
  switch (idx)
  {
#define UPDATE_PID_PROFILE(i) \
  config_.line_follow_p##i = kp_; \
  config_.line_follow_i##i = ki_; \
  config_.line_follow_d##i = kd_; \
  updateConfig()

  case '1': UPDATE_PID_PROFILE(1); break;
  case '2': UPDATE_PID_PROFILE(2); break;
  case '3': UPDATE_PID_PROFILE(3); break;
  case '4': UPDATE_PID_PROFILE(4); break;
  case '5': UPDATE_PID_PROFILE(5); break;
  default: break;

#undef UPDATE_PID_PROFILE
  }
}

void ActionTunePID::completeNav(uint8_t result)
{
  if (result == Result::RESULT_SUCCESS)
  {
    if (state_ == STOP_EXIT_JUNCTION)
    {
      float tu = computeTu();
      if (tu > 0.0f)
      {
        float ki = 0.54f * kp_ / tu;

        kp_ *= 0.45;
        if ((ki_ > 0.0f) && (ki_ < ki))
        {
          kp_ *= (ki_ / ki);
        }
        else
        {
          ki_ = ki;
        }

        ROS_INFO("Tu %f, Kp %f, Ki %f, Kd %f, Kh %f, Ahead %f",
                 tu, kp_, ki_, kd_, kh_, config_.getProfile().line_follow_ahead_distance);
        updatePID();
      }
      else
      {
        result = Result::RESULT_ERROR_TRIGGERED;
      }
    }
    else
    {
      result = Result::RESULT_ERROR_TRIGGERED;
    }
  }
  ActionProcessor::completeNav(result);
}

uint8_t ActionTunePID::checkSafetyStatus(bool enable_sensor, bool resume, bool trigger_out_of_line, UseLineSensor use_line_sensor)
{
  uint8_t status = ActionProcessor::checkSafetyStatus(enable_sensor, resume, trigger_out_of_line, use_line_sensor);
  if (status != Feedback::STATUS_NORMAL)
  {
    state_ = STOP_EXIT;
  }
  return status;
}

void ActionTunePID::lineFollow(float frequency, float speed, float line_angular_error, float line_heading_error)
{
  if (frequency <= 0.0f) return;

  if ((speed > 0.0f) && (speed == v_))
  {
    // store error during steady speed
    error_.push_back(complex(line_angular_error, line_heading_error));
  }
  else
  {
    v_ = speed;
  }

  // calculate p
  float error_p = line_angular_error * (1.0f - kh_) +
                  line_heading_error * kh_;

  // calculate output
  float output = kp_ * error_p;

  // saturate output
  float output_limit = config_.getProfile().turn_max_speed;

  if (speed == 0.0f)
  {
    output = 0.0f;
  }
  else if (output > output_limit)
  {
    output = output_limit;
  }
  else if (output < -output_limit)
  {
    output = -output_limit;
  }

  // write output to motor
  if (forward_dir_)
  {
    base_.setSpeed(speed, -output);
  }
  else
  {
    base_.setSpeed(-speed, -output);
  }

  ROS_DEBUG("Speed %f, Angular error %f, Heading error %f, Output %f",
            forward_dir_ ? speed : -speed, line_angular_error, line_heading_error, -output);
}

float ActionTunePID::getHeadingError0()
{
  float dtheta = pose_.theta - line_start_.theta;   // -2π < Δθ < 2π
  return fmod(dtheta + 3 * M_PI, 2 * M_PI) - M_PI;  // normalize to -π < Δθ < π
}

}  // namespace agv05

/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAV_ACTION_PROCESSOR_H
#define AGV05_NAV_ACTION_PROCESSOR_H

#include <agv05_nav/nav.h>


namespace agv05
{

class ActionProcessor
{
public:
  static boost::shared_ptr<ActionProcessor> create(Nav& nav, const Goal& goal);

  ActionProcessor(Nav& nav, const Goal& goal);
  virtual ~ActionProcessor() {}

  static void process(float frequency, Nav& nav);
  virtual void process(float frequency) = 0;
  virtual void handleNavControl(const NavControl& nav_control);

  // getters & setters
  uint8_t getActionType()
  {
    return goal_.nav;
  }
  uint8_t getStatus()
  {
    return status_;
  }
  uint8_t getResult()
  {
    return result_;
  }
  void ackStatusUpdate()
  {
    status_updated_ = false;
  }
  bool isStatusUpdated()
  {
    return status_updated_;
  }
  bool isCompleted()
  {
    return completed_;
  }
  virtual float getOvershoot()
  {
    return 0.0f;
  }
  void updateConfig()
  {
    nav_.handleConfig(config_, ~0);
    nav_.ds_.updateConfig(config_);
  }

protected:
  enum UseLineSensor
  {
    LINE_SENSOR_FRONT,
    LINE_SENSOR_REAR,
    LINE_SENSOR_DISABLE,
  };

protected:
  // publish functions
  void publishStatus(uint8_t status, uint8_t led_side = 0);
  static void completeNav(Nav& nav);
  virtual void completeNav(uint8_t result);
  void completeAction(uint8_t result);

  // helper functions
  virtual uint8_t checkSafetyStatus(bool enable_sensor, bool resume = false, bool trigger_out_of_line = false, UseLineSensor use_line_sensor = LINE_SENSOR_DISABLE);

  // getters
  bool isAborted()
  {
    return aborted_;
  }
  bool isPaused()
  {
    return paused_;
  }

protected:
  // robot components
  Base& base_;
  Io& io_;
  LaserSensor& laser_sensor_;
  LineSensor& line_sensor_;
  Panel& panel_;
  Safety& safety_;

  // dynamic reconfigure variable
  NavConfig& config_;

  // goal
  const Goal goal_;
  const bool old_forward_flag_;
  bool forward_flag_;

  // safety
  uint8_t safety_status_;
  std::string safety_internal_message_;

private:
  // nav node
  Nav& nav_;

  // flags
  bool aborted_;
  bool paused_;

  // published status
  bool status_updated_;
  bool completed_;
  uint8_t status_;
  uint8_t result_;
};

}  // namespace agv05

#endif  // AGV05_NAV_ACTION_PROCESSOR_H

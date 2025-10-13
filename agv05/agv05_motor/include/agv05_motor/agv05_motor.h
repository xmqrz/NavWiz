/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_MOTOR_AGV05_MOTOR_H
#define AGV05_MOTOR_AGV05_MOTOR_H

#include <dynamic_reconfigure/server.h>

#include <agv05_motor/MotorConfig.h>


namespace agv05
{
class Controller;

class Agv05Motor
{
public:
  Agv05Motor();

private:
  // dynamic reconfigure
  void callbackConfig(agv05_motor::MotorConfig &config, uint32_t level);

private:
  /* Dynamic reconfigure server */
  dynamic_reconfigure::Server<agv05_motor::MotorConfig> ds_;

  /* Controller */
  boost::shared_ptr<Controller> controller_;
};

}  // namespace agv05

#endif  // AGV05_MOTOR_AGV05_MOTOR_H

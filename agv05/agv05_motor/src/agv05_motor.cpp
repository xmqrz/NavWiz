/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <agv05_motor/agv05_motor.h>
#include <agv05_motor/controllers/custom_drive.h>
#include <agv05_motor/controllers/differential_drive.h>
#include <agv05_motor/controllers/mecanum_drive.h>
#include <agv05_motor/controllers/swerve_drive.h>
#include <agv05_motor/controllers/tricycle_drive.h>


namespace agv05
{

Agv05Motor::Agv05Motor()
{
  // dynamic reconfigure
  ds_.setCallback(boost::bind(&Agv05Motor::callbackConfig, this, _1, _2));
}

void Agv05Motor::callbackConfig(agv05_motor::MotorConfig &config, uint32_t level)
{
  ROS_INFO("agv05_motor: config received");

  // create controller on first configuration
  if (!controller_)
  {
    if (config.drive_type == "differential")
    {
      controller_ = boost::make_shared<DifferentialDrive>();
    }
    else if (config.drive_type == "tricycle_steering")
    {
      controller_ = boost::make_shared<TricycleDrive>();
    }
    else if (config.drive_type == "swerve")
    {
      controller_ = boost::make_shared<SwerveDrive>();
    }
    else if (config.drive_type == "mecanum")
    {
      controller_ = boost::make_shared<MecanumDrive>();
    }
    else if (config.drive_type == "custom")
    {
      controller_ = boost::make_shared<CustomDrive>();
    }
    if (controller_)
    {
      controller_->callbackConfig(config, level, true);
    }
  }
  else
  {
    controller_->callbackConfig(config, level);
  }
}

}  // namespace agv05


/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_motor");
  ROS_INFO("agv05_motor started.");

  agv05::Agv05Motor motor;
  ros::spin();
}

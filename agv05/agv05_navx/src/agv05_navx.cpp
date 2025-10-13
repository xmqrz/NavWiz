/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/nav.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_navx");
  agv05::Nav nav;

  ros::Rate rate(LOOP_FREQUENCY);
  while (ros::ok())
  {
    ros::spinOnce();
    nav.process(LOOP_FREQUENCY);
    rate.sleep();
  }
}

/*
 * Copyright (c) 2019, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_state_filter/state_filter.h"
#include <ros/ros.h>

#define LOOP_FREQUENCY 10


int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_state_filter");
  agv05::StateFilter state_filter("nav2d_scan_topics");

  ros::Rate r(LOOP_FREQUENCY);
  while (ros::ok())
  {
    ros::spinOnce();
    state_filter.process(LOOP_FREQUENCY);
    r.sleep();
  }
}

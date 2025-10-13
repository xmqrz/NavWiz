/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/components.h"

#include <sstream>


namespace agv05
{

Io::Io(ros::NodeHandle& nh)
{
  for (size_t i = 0; i < NUM_PORTS; i++)
  {
    inputs_[i] = 0;

    std::ostringstream oss;
    oss << "agv05/io/port" << i + 1 << "/input";
    input_subs_[i] = nh.subscribe<std_msgs::UInt16>(oss.str(), 1, boost::bind(&Io::handleInput, this, i + 1, _1));
  }
}

void Io::handleInput(size_t port, const std_msgs::UInt16ConstPtr& msg)
{
  if (port - 1 >= NUM_PORTS)
  {
    return;
  }
  inputs_[port - 1] = msg->data;
}

}  // namespace agv05

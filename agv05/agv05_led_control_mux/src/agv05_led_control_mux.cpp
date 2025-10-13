/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <ros/ros.h>
#include <agv05_msgs/LedControl.h>


/* Globals */
ros::Publisher g_pub;
agv05_msgs::LedControl g_output;

std::vector<ros::Subscriber> g_subs;
std::vector<uint8_t> g_inputs;


void inputCallback(int index, const agv05_msgs::LedControlConstPtr& msg)
{
  if (g_inputs[index] == msg->mode)
  {
    return;
  }
  g_inputs[index] = msg->mode;

  for (int i = 0, n = g_inputs.size(); i < n; ++i)
  {
    if (g_inputs[i])
    {
      g_output.mode = g_inputs[i];
      g_pub.publish(g_output);
      return;
    }
  }
  g_output.mode = agv05_msgs::LedControl::OFF;
  g_pub.publish(g_output);
}

void init()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  g_pub = nh.advertise<agv05_msgs::LedControl>("agv05/led/mode", 1, true);

  // list of input topics separated by whitespace, order by highest to lowest priority
  std::string topics(
    "agv05/led/control/user "
    "agv05/led/control/navigation "
    "agv05/led/control/power ");
  private_nh.getParam("topics", topics);

  std::istringstream iss(topics);
  std::string topic;
  while (iss >> topic)
  {
    ros::Subscriber sub = nh.subscribe<agv05_msgs::LedControl>(topic, 1,
                          boost::bind(&inputCallback, g_subs.size(), _1));
    g_subs.push_back(sub);
  }
  g_inputs.resize(g_subs.size());

  // initialize
  g_pub.publish(g_output);
}

/* main function */
int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "agv05_led_control_mux");
  ROS_INFO("agv05_led_control_mux start");

  init();
  ros::spin();
}

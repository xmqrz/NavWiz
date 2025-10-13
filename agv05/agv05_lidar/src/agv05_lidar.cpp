/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include <agv05_lidar/LidarConfig.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "agv05_lidar/lidar_watchdog.h"


/* Global variables */
LidarInspector g_lidar_inspector_1;
LidarInspector g_lidar_inspector_2;
LidarInspector g_lidar_inspector_3;
LidarInspector g_lidar_inspector_4;
LidarInspector g_lidar_inspector_5;

/* Function declaration */
void init();
void callbackConfig(const agv05_lidar::LidarConfig &config, uint32_t level);
void processActivation();
void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);


/* Function definition */
void init()
{
  ros::NodeHandle nh_private("~");

  // Init all inspector
  g_lidar_inspector_1.init(nh_private, "Primary Lidar Obstacle Sensors");
  g_lidar_inspector_2.init(nh_private, "Secondary Lidar Obstacle Sensors");
  g_lidar_inspector_3.init(nh_private, "Tertiary Lidar Obstacle Sensors");
  g_lidar_inspector_4.init(nh_private, "Quaternary Lidar Obstacle Sensors");
  g_lidar_inspector_5.init(nh_private, "Quinary Lidar Obstacle Sensors");
}

void callbackConfig(const agv05_lidar::LidarConfig& config, uint32_t level)
{
  std::string lidar_1_area_list[NUM_AREAS - 1] =
  {
    config.lidar_1_area_1_,
    config.lidar_1_area_2_,
    config.lidar_1_area_3_,
    config.lidar_1_area_4_,
    config.lidar_1_area_5_,
    config.lidar_1_area_6_,
    config.lidar_1_area_7_,
    config.lidar_1_area_8_,
    config.lidar_1_area_9_,
    config.lidar_1_area_10_,
    config.lidar_1_area_11_,
    config.lidar_1_area_12_,
    config.lidar_1_area_13_,
    config.lidar_1_area_14_,
    config.lidar_1_area_15_,
    config.lidar_1_area_16_,
    config.lidar_1_area_17_,
    config.lidar_1_area_18_,
    config.lidar_1_area_19_,
    config.lidar_1_area_20_,
    config.lidar_1_area_21_,
    config.lidar_1_area_22_,
    config.lidar_1_area_23_,
    config.lidar_1_area_24_,
    config.lidar_1_area_25_,
    config.lidar_1_area_26_,
    config.lidar_1_area_27_,
    config.lidar_1_area_28_,
    config.lidar_1_area_29_,
    config.lidar_1_area_30_,
    config.lidar_1_area_31_,
  };
  std::string lidar_2_area_list[NUM_AREAS - 1] =
  {
    config.lidar_2_area_1_,
    config.lidar_2_area_2_,
    config.lidar_2_area_3_,
    config.lidar_2_area_4_,
    config.lidar_2_area_5_,
    config.lidar_2_area_6_,
    config.lidar_2_area_7_,
    config.lidar_2_area_8_,
    config.lidar_2_area_9_,
    config.lidar_2_area_10_,
    config.lidar_2_area_11_,
    config.lidar_2_area_12_,
    config.lidar_2_area_13_,
    config.lidar_2_area_14_,
    config.lidar_2_area_15_,
    config.lidar_2_area_16_,
    config.lidar_2_area_17_,
    config.lidar_2_area_18_,
    config.lidar_2_area_19_,
    config.lidar_2_area_20_,
    config.lidar_2_area_21_,
    config.lidar_2_area_22_,
    config.lidar_2_area_23_,
    config.lidar_2_area_24_,
    config.lidar_2_area_25_,
    config.lidar_2_area_26_,
    config.lidar_2_area_27_,
    config.lidar_2_area_28_,
    config.lidar_2_area_29_,
    config.lidar_2_area_30_,
    config.lidar_2_area_31_,
  };
  std::string lidar_3_area_list[NUM_AREAS - 1] =
  {
    config.lidar_3_area_1_,
    config.lidar_3_area_2_,
    config.lidar_3_area_3_,
    config.lidar_3_area_4_,
    config.lidar_3_area_5_,
    config.lidar_3_area_6_,
    config.lidar_3_area_7_,
    config.lidar_3_area_8_,
    config.lidar_3_area_9_,
    config.lidar_3_area_10_,
    config.lidar_3_area_11_,
    config.lidar_3_area_12_,
    config.lidar_3_area_13_,
    config.lidar_3_area_14_,
    config.lidar_3_area_15_,
    config.lidar_3_area_16_,
    config.lidar_3_area_17_,
    config.lidar_3_area_18_,
    config.lidar_3_area_19_,
    config.lidar_3_area_20_,
    config.lidar_3_area_21_,
    config.lidar_3_area_22_,
    config.lidar_3_area_23_,
    config.lidar_3_area_24_,
    config.lidar_3_area_25_,
    config.lidar_3_area_26_,
    config.lidar_3_area_27_,
    config.lidar_3_area_28_,
    config.lidar_3_area_29_,
    config.lidar_3_area_30_,
    config.lidar_3_area_31_,
  };
  std::string lidar_4_area_list[NUM_AREAS - 1] =
  {
    config.lidar_4_area_1_,
    config.lidar_4_area_2_,
    config.lidar_4_area_3_,
    config.lidar_4_area_4_,
    config.lidar_4_area_5_,
    config.lidar_4_area_6_,
    config.lidar_4_area_7_,
    config.lidar_4_area_8_,
    config.lidar_4_area_9_,
    config.lidar_4_area_10_,
    config.lidar_4_area_11_,
    config.lidar_4_area_12_,
    config.lidar_4_area_13_,
    config.lidar_4_area_14_,
    config.lidar_4_area_15_,
    config.lidar_4_area_16_,
    config.lidar_4_area_17_,
    config.lidar_4_area_18_,
    config.lidar_4_area_19_,
    config.lidar_4_area_20_,
    config.lidar_4_area_21_,
    config.lidar_4_area_22_,
    config.lidar_4_area_23_,
    config.lidar_4_area_24_,
    config.lidar_4_area_25_,
    config.lidar_4_area_26_,
    config.lidar_4_area_27_,
    config.lidar_4_area_28_,
    config.lidar_4_area_29_,
    config.lidar_4_area_30_,
    config.lidar_4_area_31_,
  };
  std::string lidar_5_area_list[NUM_AREAS - 1] =
  {
    config.lidar_5_area_1_,
    config.lidar_5_area_2_,
    config.lidar_5_area_3_,
    config.lidar_5_area_4_,
    config.lidar_5_area_5_,
    config.lidar_5_area_6_,
    config.lidar_5_area_7_,
    config.lidar_5_area_8_,
    config.lidar_5_area_9_,
    config.lidar_5_area_10_,
    config.lidar_5_area_11_,
    config.lidar_5_area_12_,
    config.lidar_5_area_13_,
    config.lidar_5_area_14_,
    config.lidar_5_area_15_,
    config.lidar_5_area_16_,
    config.lidar_5_area_17_,
    config.lidar_5_area_18_,
    config.lidar_5_area_19_,
    config.lidar_5_area_20_,
    config.lidar_5_area_21_,
    config.lidar_5_area_22_,
    config.lidar_5_area_23_,
    config.lidar_5_area_24_,
    config.lidar_5_area_25_,
    config.lidar_5_area_26_,
    config.lidar_5_area_27_,
    config.lidar_5_area_28_,
    config.lidar_5_area_29_,
    config.lidar_5_area_30_,
    config.lidar_5_area_31_,
  };
  g_lidar_inspector_1.updateConfig(lidar_1_area_list, config.lidar_1_min_activation_);
  g_lidar_inspector_2.updateConfig(lidar_2_area_list, config.lidar_2_min_activation_);
  g_lidar_inspector_3.updateConfig(lidar_3_area_list, config.lidar_3_min_activation_);
  g_lidar_inspector_4.updateConfig(lidar_4_area_list, config.lidar_4_min_activation_);
  g_lidar_inspector_5.updateConfig(lidar_5_area_list, config.lidar_5_min_activation_);
}

void processActivation()
{
  double now = ros::Time::now().toSec();
  g_lidar_inspector_1.processActivation(now);
  g_lidar_inspector_2.processActivation(now);
  g_lidar_inspector_3.processActivation(now);
  g_lidar_inspector_4.processActivation(now);
  g_lidar_inspector_5.processActivation(now);
}

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_lidar");
  ROS_INFO_STREAM("Node started.");

  // dynamic_reconfigure
  dynamic_reconfigure::Server<agv05_lidar::LidarConfig> server;
  server.setCallback(callbackConfig);

  // diagnostic updater
  diagnostic_updater::Updater updater;
  updater.setHardwareID("AGV05");
  updater.add("Status", diagnosticStatus);

  init();

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    processActivation();
    updater.update();
    r.sleep();
  }
  return 0;
}

void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (g_lidar_inspector_1.enabled)
  {
    stat.add("Primary Lidar Obstacle Sensors Area", g_lidar_inspector_1.status.area);
    stat.add("Primary Lidar Obstacle Sensors Activation", g_lidar_inspector_1.status.activation);
    stat.add("Primary Lidar Obstacle Sensors Activation Hint", g_lidar_inspector_1.status.activation_hint);
  }
  if (g_lidar_inspector_2.enabled)
  {
    stat.add("Secondary Lidar Obstacle Sensors Area", g_lidar_inspector_2.status.area);
    stat.add("Secondary Lidar Obstacle Sensors Activation", g_lidar_inspector_2.status.activation);
    stat.add("Secondary Lidar Obstacle Sensors Activation Hint", g_lidar_inspector_2.status.activation_hint);
  }
  if (g_lidar_inspector_3.enabled)
  {
    stat.add("Tertiary Lidar Obstacle Sensors Area", g_lidar_inspector_3.status.area);
    stat.add("Tertiary Lidar Obstacle Sensors Activation", g_lidar_inspector_3.status.activation);
    stat.add("Tertiary Lidar Obstacle Sensors Activation Hint", g_lidar_inspector_3.status.activation_hint);
  }
  if (g_lidar_inspector_4.enabled)
  {
    stat.add("Quaternary Lidar Obstacle Sensors Area", g_lidar_inspector_4.status.area);
    stat.add("Quaternary Lidar Obstacle Sensors Activation", g_lidar_inspector_4.status.activation);
    stat.add("Quaternary Lidar Obstacle Sensors Activation Hint", g_lidar_inspector_4.status.activation_hint);
  }
  if (g_lidar_inspector_5.enabled)
  {
    stat.add("Quinary Lidar Obstacle Sensors Area", g_lidar_inspector_5.status.area);
    stat.add("Quinary Lidar Obstacle Sensors Activation", g_lidar_inspector_5.status.activation);
    stat.add("Quinary Lidar Obstacle Sensors Activation Hint", g_lidar_inspector_5.status.activation_hint);
  }

  // Summary
  if (g_lidar_inspector_1.status.activation.rfind("Lidar Malfunction", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Primary lidar obstacle sensors malfunction");
  }
  else if (g_lidar_inspector_2.status.activation.rfind("Lidar Malfunction", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Secondary lidar obstacle sensors malfunction");
  }
  else if (g_lidar_inspector_3.status.activation.rfind("Lidar Malfunction", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Tertiary lidar obstacle sensors malfunction");
  }
  else if (g_lidar_inspector_4.status.activation.rfind("Lidar Malfunction", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Quaternary lidar obstacle sensors malfunction");
  }
  else if (g_lidar_inspector_5.status.activation.rfind("Lidar Malfunction", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Quinary lidar obstacle sensors malfunction");
  }
  else if (g_lidar_inspector_1.status.activation == "Invalid Area Configuration")
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Primary lidar obstacle sensors has invalid configuration");
  }
  else if (g_lidar_inspector_2.status.activation == "Invalid Area Configuration")
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Secondary lidar obstacle sensors has invalid configuration");
  }
  else if (g_lidar_inspector_3.status.activation == "Invalid Area Configuration")
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Tertiary lidar obstacle sensors has invalid configuration");
  }
  else if (g_lidar_inspector_4.status.activation == "Invalid Area Configuration")
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Quaternary lidar obstacle sensors has invalid configuration");
  }
  else if (g_lidar_inspector_5.status.activation == "Invalid Area Configuration")
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Quinary lidar obstacle sensors has invalid configuration");
  }
  else if (g_lidar_inspector_1.status.activation.rfind("Invalid Laser Trasnformation", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Primary lidar obstacle sensors fail to obtain trasnformation");
  }
  else if (g_lidar_inspector_2.status.activation.rfind("Invalid Laser Trasnformation", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Secondary lidar obstacle sensors fail to obtain trasnformation");
  }
  else if (g_lidar_inspector_3.status.activation.rfind("Invalid Laser Trasnformation", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Tertiary lidar obstacle sensors fail to obtain trasnformation");
  }
  else if (g_lidar_inspector_4.status.activation.rfind("Invalid Laser Trasnformation", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Quaternary lidar obstacle sensors fail to obtain trasnformation");
  }
  else if (g_lidar_inspector_5.status.activation.rfind("Invalid Laser Trasnformation", 0) == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Quinary lidar obstacle sensors fail to obtain trasnformation");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status OK");
  }
}

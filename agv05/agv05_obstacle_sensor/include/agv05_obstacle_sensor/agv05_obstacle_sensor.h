/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#ifndef AGV05_OBSTACLE_SENSOR_AGV05_OBSTACLE_SENSOR_H
#define AGV05_OBSTACLE_SENSOR_AGV05_OBSTACLE_SENSOR_H

#include <boost/algorithm/string.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <json/json.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <agv05_lidar/lidar_watchdog.h>
#include <agv05_msgs/ObstacleSensor.h>
#include <agv05_msgs/ObstacleSensorArea.h>
#include <agv05_obstacle_sensor/ObstacleSensorConfig.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>

// loop frequency
#define LOOP_FREQUENCY        20


// functions
void init();
void processObstacleSensor(float frequency);
void processObstacleSensorArea(float frequency);
std::string generateEnumSrc(dict* p_lidar = NULL);

// helper function
std::vector<std::string> getListFromCSV(std::string csv);
void capitalize(std::string &s);
void updateRobotInflationRadius(int profile);

// callbacks
void callbackArea(const agv05_msgs::ObstacleSensorArea& area);
void callbackActivation(int index, const agv05_msgs::ObstacleSensorConstPtr& activation);

// diagnostic function
void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

// service function
bool enumServerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

// callbackConfig
void callBackConfig(agv05_obstacle_sensor::ObstacleSensorConfig& config, uint32_t level);

#endif  // AGV05_OBSTACLE_SENSOR_AGV05_OBSTACLE_SENSOR_H

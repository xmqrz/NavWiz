/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_VARIABLE_STORAGE_VARIABLE_STORAGE_H
#define AGV05_VARIABLE_STORAGE_VARIABLE_STORAGE_H

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace agv05
{

class VariableStorageBackend;

class VariableStorage
{
public:
  explicit VariableStorage(const ros::NodeHandle& nh = ros::NodeHandle("~"));

  bool hasVariable(const std::string& name);
  bool getVariable(const std::string& name, std::string& value);
  void setVariable(const std::string& name, const std::string& value);
  void deleteVariable(const std::string& name);

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<VariableStorageBackend> backend_;
};

}  // namespace agv05

#endif  // AGV05_VARIABLE_STORAGE_VARIABLE_STORAGE_H

/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "agv05_variable_storage/variable_storage.h"


TEST(VariableStorageTest, MainTest)
{
  agv05::VariableStorage storage;
  const std::string name = "cpp_test_key_1";
  const std::string name2 = "cpp_test_key_2";
  const std::string value = "cpp test value 1024";
  const std::string value2 = "cpp test value 2048";
  std::string rcv_value;

  storage.setVariable(name, value);
  storage.setVariable(name2, value2);
  ASSERT_TRUE(storage.hasVariable(name));
  ASSERT_TRUE(storage.hasVariable(name2));
  ASSERT_TRUE(storage.getVariable(name, rcv_value));
  ASSERT_EQ(rcv_value, value);
  ASSERT_TRUE(storage.getVariable(name2, rcv_value));
  ASSERT_EQ(rcv_value, value2);

  storage.setVariable(name, value2);
  storage.setVariable(name2, value);
  ASSERT_TRUE(storage.hasVariable(name));
  ASSERT_TRUE(storage.hasVariable(name2));
  ASSERT_TRUE(storage.getVariable(name, rcv_value));
  ASSERT_EQ(rcv_value, value2);
  ASSERT_TRUE(storage.getVariable(name2, rcv_value));
  ASSERT_EQ(rcv_value, value);

  storage.deleteVariable(name);
  storage.deleteVariable(name2);
  ASSERT_FALSE(storage.hasVariable(name));
  ASSERT_FALSE(storage.hasVariable(name2));
  ASSERT_FALSE(storage.getVariable(name, rcv_value));
  ASSERT_FALSE(storage.getVariable(name2, rcv_value));
};


/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_cpp");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

cmake_minimum_required(VERSION 2.8.3)
project(agv05_executive_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  std_srvs
)

add_service_files(
  DIRECTORY srv
  FILES
  GetRegister.srv
  GetVariable.srv
  SetModuleRunning.srv
  SetRegister.srv
  SetVariable.srv
  SetRobotRunning.srv
  TriggerTask.srv
)

generate_messages(DEPENDENCIES std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs std_srvs)

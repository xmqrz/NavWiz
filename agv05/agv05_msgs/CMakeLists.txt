cmake_minimum_required(VERSION 2.8.3)
project(agv05_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
  std_srvs
)

add_message_files(
  DIRECTORY msg
  FILES
  AudioControl.msg
  BatteryState.msg
  HardwareInfo.msg
  HardwareInfoEntry.msg
  LedControl.msg
  LineSensor.msg
  LineSensorActivation.msg
  Marker2D.msg
  MecanumControl.msg
  MecanumFeedback.msg
  ModbusReply.msg
  ModbusRequest.msg
  Motor.msg
  MotorControl.msg
  MotorFeedback.msg
  MsbRaw.msg
  NavControl.msg
  ObstacleSensor.msg
  ObstacleSensorArea.msg
  OutputPort.msg
  Path.msg
  PolygonArray.msg
  PowerSource.msg
  RemoteIoReply.msg
  RemoteIoRequest.msg
  SafetyTriggers.msg
  SteeringControl.msg
  SteeringFeedback.msg
  SwerveControl.msg
  SwerveFeedback.msg
  UIPopup.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetTwist.srv
  ResetMileage.srv
)

add_action_files(
  DIRECTORY action
  FILES
  NavAction.action
  NavxAction.action
)


generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs std_srvs)

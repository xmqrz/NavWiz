cmake_minimum_required(VERSION 2.8.3)
project(agv05_motor)

find_package(catkin REQUIRED COMPONENTS
  agv05_msgs
  agv05_variable_storage
  angles
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Motor.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

set(agv05_motor_srcs
  src/agv05_motor.cpp
  src/controller.cpp
  src/controllers/custom_drive.cpp
  src/controllers/differential_drive.cpp
  src/controllers/mecanum_drive.cpp
  src/controllers/swerve_drive.cpp
  src/controllers/tricycle_drive.cpp
)

add_executable(agv05_motor ${agv05_motor_srcs})
target_link_libraries(agv05_motor ${catkin_LIBRARIES})
add_dependencies(agv05_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_motor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  roslint_cpp()
  roslint_add_test()
endif()

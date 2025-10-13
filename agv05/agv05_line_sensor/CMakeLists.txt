cmake_minimum_required(VERSION 2.8.3)
project(agv05_line_sensor)

find_package(catkin REQUIRED COMPONENTS
  agv05_msgs
  agv05_variable_storage
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LineSensor.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(agv05_line_sensor_srcs
  src/agv05_line_sensor.cpp
)

add_executable(agv05_line_sensor ${agv05_line_sensor_srcs})
target_link_libraries(agv05_line_sensor ${catkin_LIBRARIES})
add_dependencies(agv05_line_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_line_sensor
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

cmake_minimum_required(VERSION 2.8.3)
project(agv05_obstacle_sensor)

find_package(catkin REQUIRED COMPONENTS
  agv05_lidar
  agv05_msgs
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  std_srvs
)
find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

# pkg-config package
find_package(PkgConfig)
pkg_search_module(jsoncpp REQUIRED jsoncpp)

generate_dynamic_reconfigure_options(
  cfg/ObstacleSensor.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
)

set(agv05_obstacle_sensor_srcs
  src/agv05_obstacle_sensor.cpp
)

add_executable(agv05_obstacle_sensor ${agv05_obstacle_sensor_srcs})
target_link_libraries(agv05_obstacle_sensor
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${jsoncpp_LIBRARIES}
)
add_dependencies(agv05_obstacle_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_obstacle_sensor
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

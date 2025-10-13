cmake_minimum_required(VERSION 2.8.3)
project(agv05_lidar)

find_package(catkin REQUIRED COMPONENTS
  agv05_msgs
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(Boost REQUIRED COMPONENTS thread)

# pkg-config package
find_package(PkgConfig)
pkg_search_module(jsoncpp REQUIRED jsoncpp)

generate_dynamic_reconfigure_options(
  cfg/Lidar.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agv05_lidar_watchdog
  DEPENDS agv05_msgs
)


###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
)

set(agv05_lidar_srcs
  src/agv05_lidar.cpp
)

set(agv05_lidar_watchdog_srcs
  src/lidar_watchdog.cpp
)

add_library(agv05_lidar_watchdog ${agv05_lidar_watchdog_srcs})
target_link_libraries(agv05_lidar_watchdog ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${jsoncpp_LIBRARIES})
add_dependencies(agv05_lidar_watchdog ${catkin_EXPORTED_TARGETS})

add_executable(agv05_lidar ${agv05_lidar_srcs})
target_link_libraries(agv05_lidar agv05_lidar_watchdog ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${jsoncpp_LIBRARIES})
add_dependencies(agv05_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_lidar agv05_lidar_watchdog
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  roslint_cpp()
  roslint_add_test()
  add_subdirectory(test)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(agv05_state_filter)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Find CSM library
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)


generate_dynamic_reconfigure_options(
  cfg/StateFilter.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

set(agv05_state_filter_srcs
  src/agv05_state_filter.cpp
  src/state_filter.cpp
)

add_executable(agv05_state_filter ${agv05_state_filter_srcs})
target_link_libraries(agv05_state_filter ${catkin_LIBRARIES} ${csm_LIBRARIES})
add_dependencies(agv05_state_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_state_filter
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

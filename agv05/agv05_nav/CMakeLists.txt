cmake_minimum_required(VERSION 2.8.3)
project(agv05_nav)

if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  agv05_msgs
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS
  system
)

# pkg-config package
find_package(PkgConfig)
pkg_search_module(jsoncpp REQUIRED jsoncpp)

generate_dynamic_reconfigure_options(
  cfg/Nav.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agv05_nav_components
  DEPENDS agv05_msgs
  DEPENDS geometry_msgs
  DEPENDS nav_msgs
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
)

set(agv05_nav_components_srcs
  src/components/base.cpp
  src/components/io.cpp
  src/components/laser_sensor.cpp
  src/components/line_sensor.cpp
  src/components/panel.cpp
  src/components/safety.cpp
)

set(agv05_nav_srcs
  src/actions/free_motor.cpp
  src/actions/line_calibrate.cpp
  src/actions/manual_control.cpp
  src/actions/processor.cpp
  src/actions/scan_laser.cpp
  src/actions/select_laser_profile.cpp
  src/actions/select_nav_profile.cpp
  src/actions/straight.cpp
  src/actions/tune_pid.cpp
  src/actions/turn.cpp
  src/actions/wait_traffic.cpp
  src/agv05_nav.cpp
  src/diagnostic.cpp
  src/nav.cpp
  src/nav_config.cpp
)

add_library(agv05_nav_components ${agv05_nav_components_srcs})
target_link_libraries(agv05_nav_components ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(agv05_nav_components ${catkin_EXPORTED_TARGETS})

add_executable(agv05_nav ${agv05_nav_srcs})
target_link_libraries(agv05_nav agv05_nav_components ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${jsoncpp_LIBRARIES})
add_dependencies(agv05_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_nav agv05_nav_components
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS
  scripts/agv05_nav_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  set(ROSLINT_PYTHON_OPTS "--max-line-length=199" "--ignore=E128,E741,W503,W504,W606")
  roslint_cpp()
  roslint_python()
  roslint_add_test()
endif()

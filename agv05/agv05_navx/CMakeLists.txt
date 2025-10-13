cmake_minimum_required(VERSION 2.8.3)
project(agv05_navx)

if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  agv05_lidar
  agv05_msgs
  agv05_nav
  costmap_2d
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  global_planner
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

find_package(CGAL REQUIRED COMPONENTS
  Core
)
if("${CGAL_LIBRARIES}" STREQUAL "")
  if(CGAL_VERSION VERSION_LESS 4.9.0)
    message("CGAL_VERSION ${CGAL_VERSION} < 4.9.0")
    set(CGAL_LIBRARIES CGAL CGAL_Core gmp)
  else()
    message("CGAL_VERSION ${CGAL_VERSION} >= 4.9.0")
    set(CGAL_LIBRARIES CGAL::CGAL CGAL::CGAL_Core)
  endif()
endif()

# pkg-config package
find_package(PkgConfig)
pkg_search_module(jsoncpp REQUIRED jsoncpp)

generate_dynamic_reconfigure_options(
  cfg/Nav.cfg
  cfg/ForbiddenLayer.cfg
  cfg/StaticPathLayer.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    agv05_costmap_layers
  CATKIN_DEPENDS
    agv05_msgs
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
  DEPENDS
    Boost
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
)

set(agv05_navx_srcs
  src/actions/bezier.cpp
  src/actions/dock.cpp
  src/actions/dynamic.cpp
  src/actions/manual_control.cpp
  src/actions/omni.cpp
  src/actions/omni_turn.cpp
  src/actions/processor.cpp
  src/actions/select_laser_profile.cpp
  src/actions/select_nav_profile.cpp
  src/actions/straight.cpp
  src/actions/turn.cpp
  src/agv05_navx.cpp
  src/components/base.cpp
  src/components/dynamic_obstacle_sensor.cpp
  src/diagnostic.cpp
  src/nav.cpp
  src/nav_config.cpp
)

add_executable(agv05_navx ${agv05_navx_srcs})
target_link_libraries(agv05_navx ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CGAL_LIBRARIES} ${jsoncpp_LIBRARIES})
add_dependencies(agv05_navx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(agv05_costmap_layers
  src/plugins/costmap/forbidden_layer.cpp
  src/plugins/costmap/static_path_layer.cpp
)
set_target_properties(agv05_costmap_layers PROPERTIES
  COMPILE_FLAGS
    -frounding-math
  LINK_FLAGS
    -Wl,--no-undefined
)
target_link_libraries(agv05_costmap_layers ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CGAL_LIBRARIES})
add_dependencies(agv05_costmap_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS
  agv05_navx
  agv05_costmap_layers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/agv05_navx_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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

cmake_minimum_required(VERSION 2.8.3)
project(marker_localization)

if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
  endif ()
endif ()

find_package(catkin REQUIRED COMPONENTS
  agv05_msgs
  angles
  aruco
  aruco_msgs
  aruco_ros
  cv_bridge
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/MarkerLocalization.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(marker_localization_srcs
  src/detectors/ar.cpp
  src/detectors/bar_ar.cpp
  src/detectors/bar_reflector.cpp
  src/detectors/custom.cpp
  src/detectors/image_pallet.cpp
  src/detectors/pallet.cpp
  src/detectors/reflector.cpp
  src/detectors/vmarker.cpp
  src/detectors/vmarker/bar_mixin.cpp
  src/detectors/vmarker/line_extraction.cpp
  src/detectors/vmarker/utilities.cpp
  src/detectors/vmarker/v_extraction.cpp
  src/marker_localization.cpp
)

add_executable(marker_localization ${marker_localization_srcs})
target_link_libraries(marker_localization ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(marker_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS marker_localization
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

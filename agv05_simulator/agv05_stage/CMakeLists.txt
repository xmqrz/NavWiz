cmake_minimum_required(VERSION 2.8.3)
project(agv05_stage)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rosgraph_msgs
  sensor_msgs
  tf2_ros)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(stage REQUIRED)

catkin_package()


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS})

add_executable(agv05_stage src/agv05_stage.cpp)
target_link_libraries(agv05_stage ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${STAGE_LIBRARIES})
add_dependencies(agv05_stage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_stage
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY providers
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


############
## Testng ##
############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  roslaunch_add_file_check(providers/launch)
  roslint_cpp()
  roslint_add_test()
endif()

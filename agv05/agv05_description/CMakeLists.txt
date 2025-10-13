cmake_minimum_required(VERSION 2.8.3)
project(agv05_description)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  urdf
  xacro
)

generate_dynamic_reconfigure_options(
  cfg/Description.cfg
)

catkin_package(
  CATKIN_DEPENDS urdf xacro)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(agv05_description src/agv05_description.cpp)
target_link_libraries(agv05_description ${catkin_LIBRARIES})
add_dependencies(agv05_description ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_description
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  roslint_cpp()
  roslint_add_test()
endif()

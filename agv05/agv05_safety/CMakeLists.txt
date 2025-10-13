cmake_minimum_required(VERSION 2.8.3)
project(agv05_safety)

find_package(catkin REQUIRED COMPONENTS
  agv05_msgs
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Safety.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

set(agv05_safety_srcs
  src/agv05_safety.cpp)

add_executable(agv05_safety ${agv05_safety_srcs})
target_link_libraries(agv05_safety ${catkin_LIBRARIES})
add_dependencies(agv05_safety ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS agv05_safety
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


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

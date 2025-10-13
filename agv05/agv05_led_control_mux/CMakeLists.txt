cmake_minimum_required(VERSION 2.8.3)
project(agv05_led_control_mux)

find_package(catkin REQUIRED COMPONENTS
  agv05_msgs
  roscpp
)

catkin_package()


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(agv05_led_control_mux src/agv05_led_control_mux.cpp)
target_link_libraries(agv05_led_control_mux ${catkin_LIBRARIES})
add_dependencies(agv05_led_control_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_led_control_mux
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

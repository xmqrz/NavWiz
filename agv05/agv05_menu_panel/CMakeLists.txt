cmake_minimum_required(VERSION 2.8.3)
project(agv05_menu_panel)

find_package(catkin REQUIRED COMPONENTS
  agv05_executive_msgs
  agv05_msgs
  diagnostic_updater
  dynamic_reconfigure
  linux_wifi
  roscpp
  std_msgs
  std_srvs
)

# pkg-config package
find_package(PkgConfig)
pkg_search_module(jsoncpp REQUIRED jsoncpp)

generate_dynamic_reconfigure_options(
  cfg/MenuPanel.cfg
)

catkin_package()


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${jsoncpp_INCLUDE_DIRS})

set(agv05_menu_panel_srcs
  src/agv05_menu_panel.cpp
)

add_executable(agv05_menu_panel ${agv05_menu_panel_srcs})
target_link_libraries(agv05_menu_panel ${catkin_LIBRARIES} ${jsoncpp_LIBRARIES})
add_dependencies(agv05_menu_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_menu_panel
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

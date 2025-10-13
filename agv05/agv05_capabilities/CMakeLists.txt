cmake_minimum_required(VERSION 2.8.3)
project(agv05_capabilities)

find_package(catkin REQUIRED)

catkin_package()


#############
## Install ##
#############

install(DIRECTORY interfaces
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY providers
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(providers/launch)
endif()

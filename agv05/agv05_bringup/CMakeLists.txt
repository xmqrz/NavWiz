cmake_minimum_required(VERSION 2.8.3)
project(agv05_bringup)

find_package(catkin REQUIRED)

catkin_package()


#############
## Install ##
#############

install(DIRECTORY configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

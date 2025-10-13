cmake_minimum_required(VERSION 2.8.3)
project(agv05_gazebo)

find_package(catkin REQUIRED)

catkin_package()


#############
## Install ##
#############

install(DIRECTORY providers
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
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
  roslaunch_add_file_check(providers/launch)
endif()

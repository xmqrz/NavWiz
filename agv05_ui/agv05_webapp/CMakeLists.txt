cmake_minimum_required(VERSION 2.8.3)
project(agv05_webapp)
find_package(catkin REQUIRED)
catkin_package()


###########
## Build ##
###########

add_custom_target(${PROJECT_NAME}_npm_install ALL
  COMMAND ./npm_install.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Install required node modules, then build using vite."
)


#############
## Install ##
#############

install(DIRECTORY
  extension
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  hwapp_build
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  dashboard_build
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
#if (CATKIN_ENABLE_TESTING)
#  find_package(roslint REQUIRED)
#  roslint_add_test()
#endif()

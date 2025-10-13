cmake_minimum_required(VERSION 2.8.3)
project(agv05_modbus_server)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


###########
## Build ##
###########


#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/modbus_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_PYTHON_OPTS "--max-line-length=199" "--ignore=E128,E741,W503,W504,W606")
  roslint_python()
  roslint_add_test()
endif()

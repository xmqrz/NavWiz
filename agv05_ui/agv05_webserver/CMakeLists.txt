cmake_minimum_required(VERSION 2.8.3)
project(agv05_webserver)
find_package(catkin REQUIRED COMPONENTS pip_install)

pip_install_requirements(
  src/requirements.d/prod.txt
  src/requirements.d/base.txt
  src/requirements.d/base-indirect.txt
)

catkin_python_setup()

catkin_package()


###########
## Build ##
###########

set(sockjs__init_py__build ${CMAKE_CURRENT_BINARY_DIR}/pip-generated/${PYTHON_INSTALL_DIR}/sockjs/__init__.py)
set(sockjs__init_py__devel ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/sockjs/__init__.py)

if ($ENV{ROS_PYTHON_VERSION} EQUAL 2)
  add_custom_target(${PROJECT_NAME}_pip_install_fix ALL
    COMMAND touch ${sockjs__init_py__build} && pycompile ${sockjs__init_py__build}
    COMMAND touch ${sockjs__init_py__devel} && pycompile ${sockjs__init_py__devel}
    DEPENDS ${PROJECT_NAME}_pip_install_requirements
    COMMENT "Fix broken package from pip install."
)
else()
  add_custom_target(${PROJECT_NAME}_pip_install_fix ALL
    COMMAND touch ${sockjs__init_py__build} && python3 -m compileall -b ${sockjs__init_py__build}
    COMMAND touch ${sockjs__init_py__devel} && python3 -m compileall -b ${sockjs__init_py__devel}
    DEPENDS ${PROJECT_NAME}_pip_install_requirements
    COMMENT "Fix broken package from pip install."
)
endif()

set(enum__build ${CMAKE_CURRENT_BINARY_DIR}/pip-generated/${PYTHON_INSTALL_DIR}/enum)
set(enum__devel ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/enum)
set(enumfields__build ${CMAKE_CURRENT_BINARY_DIR}/pip-generated/${PYTHON_INSTALL_DIR}/enumfields/enum)
set(enumfields__devel ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/enumfields/enum)

add_custom_target(${PROJECT_NAME}_pip_install_move ALL
  COMMAND [ ! -e ${enum__build} ] || (rm -rf ${enumfields__build} && mv ${enum__build} ${enumfields__build})
  COMMAND [ ! -e ${enum__devel} ] || (rm -rf ${enumfields__devel} && mv ${enum__devel} ${enumfields__devel})
  DEPENDS ${PROJECT_NAME}_pip_install_requirements
  COMMENT "Relocate enum module to prevent module name collision."
)

set(Cryptodome__SelfTest__build ${CMAKE_CURRENT_BINARY_DIR}/pip-generated/${PYTHON_INSTALL_DIR}/Cryptodome/SelfTest)
set(Cryptodome__SelfTest__devel ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/Cryptodome/SelfTest)

add_custom_target(${PROJECT_NAME}_pip_install_strip ALL
  COMMAND rm -rf ${Cryptodome__SelfTest__build}
  COMMAND rm -rf ${Cryptodome__SelfTest__devel}
  DEPENDS ${PROJECT_NAME}_pip_install_requirements
  COMMENT "Strip unneeded files from pip install."
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/manage.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/celery
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  src/requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  src/requirements.d
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  debian_files/mysql/agv05.cnf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mysql
)

install(DIRECTORY
  assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/hw_app_dev.py
  scripts/dashboard_dev.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/hw_app_dev
  scripts/dashboard_dev
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  set(ROSLINT_PYTHON_OPTS "--max-line-length=199" "--ignore=E128,E741,W503,W504,W606")
  roslaunch_add_file_check(launch)
  roslint_python(src)
  roslint_add_test()

  find_package(rostest REQUIRED)
  add_rostest(tests/test_io_channel.test)
endif()

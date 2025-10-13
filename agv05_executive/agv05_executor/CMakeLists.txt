cmake_minimum_required(VERSION 2.8.3)
project(agv05_executor)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/Executor.cfg)

catkin_package()


###########
## Build ##
###########


#############
## Install ##
#############

install(PROGRAMS
  scripts/overlay_chroot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/execute.py
  scripts/execute
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  set(ROSLINT_PYTHON_OPTS "--max-line-length=199" "--ignore=E128,E741,W503,W504,W606")
  roslaunch_add_file_check(launch)
  roslint_python()
  roslint_add_test()

  add_rostest(tests/test_models.test DEPENDENCIES ${catkin_EXPORTED_TARGETS})
  add_rostest(tests/test_models_x.test DEPENDENCIES ${catkin_EXPORTED_TARGETS})
  add_rostest(tests/test_tracked_sim.test DEPENDENCIES ${catkin_EXPORTED_TARGETS})
  add_rostest(tests/test_trackless_sim.test DEPENDENCIES ${catkin_EXPORTED_TARGETS})
endif()

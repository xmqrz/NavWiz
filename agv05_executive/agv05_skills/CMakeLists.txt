cmake_minimum_required(VERSION 2.8.3)
project(agv05_skills)
find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


#############
## Install ##
#############


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rosunit REQUIRED)
  find_package(rostest REQUIRED)

  set(ROSLINT_PYTHON_OPTS "--max-line-length=199" "--ignore=E128,E741,W503,W504,W606")
  roslint_python()
  roslint_add_test()
  catkin_add_nosetests(tests/units DEPENDENCIES ${catkin_EXPORTED_TARGETS})

  add_rostest(tests/test_plan.test DEPENDENCIES ${catkin_EXPORTED_TARGETS})
  add_rostest(tests/test_plan_x.test DEPENDENCIES ${catkin_EXPORTED_TARGETS})
endif()

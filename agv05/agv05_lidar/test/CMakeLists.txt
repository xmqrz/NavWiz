find_package(catkin REQUIRED COMPONENTS
  agv05_lidar
  roscpp
  rostest)

# cpp
add_rostest_gtest(agv05_lidar-cpp_test test_cpp.test test.cpp)
target_link_libraries(agv05_lidar-cpp_test ${catkin_LIBRARIES})
add_dependencies(agv05_lidar-cpp_test ${catkin_EXPORTED_TARGETS})

add_dependencies(tests agv05_lidar-cpp_test)

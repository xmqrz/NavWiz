find_package(catkin REQUIRED COMPONENTS
  agv05_variable_storage
  rostest)

# cpp
add_rostest_gtest(agv05_variable_storage-cpp_test test_cpp.test test.cpp)
target_link_libraries(agv05_variable_storage-cpp_test ${catkin_LIBRARIES})
add_dependencies(agv05_variable_storage-cpp_test ${catkin_EXPORTED_TARGETS})

add_dependencies(tests agv05_variable_storage-cpp_test)


# python
add_rostest(test_py.test DEPENDENCIES agv05_variable_storage)

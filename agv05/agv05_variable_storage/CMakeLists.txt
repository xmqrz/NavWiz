cmake_minimum_required(VERSION 2.8.3)
project(agv05_variable_storage)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(MySQLConnectorCPP REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agv05_variable_storage
  DEPENDS MySQLConnectorCPP)


###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${MYSQLCONNECTORCPP_INCLUDE_DIRS})

set(agv05_variable_storage_srcs
  src/variable_storage.cpp)

add_library(agv05_variable_storage ${agv05_variable_storage_srcs})
target_link_libraries(agv05_variable_storage ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${MYSQLCONNECTORCPP_LIBRARIES})
add_dependencies(agv05_variable_storage ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS agv05_variable_storage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  roslint_cpp()
  roslint_python()
  roslint_add_test()
  add_subdirectory(test)
endif()

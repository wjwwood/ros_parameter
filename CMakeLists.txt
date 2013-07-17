cmake_minimum_required(VERSION 2.8.3)
project(ros_parameter)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
add_library(ros_parameter src/${PROJECT_NAME}/ros_parameter.cpp)

## Declare a cpp executable
add_executable(example_node src/example_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(example_node ros_parameter ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ros_parameter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_parameter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

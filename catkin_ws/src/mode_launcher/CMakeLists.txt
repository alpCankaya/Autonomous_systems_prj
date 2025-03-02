cmake_minimum_required(VERSION 3.0.2)
project(mode_launcher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
)

find_package(octomap REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs pcl_conversions pcl_ros
  DEPENDS octomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for the mode launcher node.
## Note: Use the full relative path to the source file.
add_executable(${PROJECT_NAME}_node src/drone_frontier_exploration/src/drone_frontier_exploration.cpp)

## Link the executable with necessary libraries.
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cmake_minimum_required(VERSION 3.0.2)
project(d_star_nav_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for dstar_explorer
add_executable(dstar_explorer_node src/dstar_explorer.cpp src/dstar_lite.cpp)
target_link_libraries(dstar_explorer_node ${catkin_LIBRARIES})
add_dependencies(dstar_explorer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable for frontier_detector
add_executable(frontier_detector_node src/frontier_detector.cpp)
target_link_libraries(frontier_detector_node ${catkin_LIBRARIES})
add_dependencies(frontier_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


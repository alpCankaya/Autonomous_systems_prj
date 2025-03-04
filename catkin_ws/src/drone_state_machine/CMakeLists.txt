cmake_minimum_required(VERSION 3.0.2)
project(drone_state_machine)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  std_srvs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
## include_directories(include ...)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Build the node
add_executable(drone_state_machine_node src/drone_state_machine_node.cpp)
target_link_libraries(drone_state_machine_node
  ${catkin_LIBRARIES}
)

## Mark other files for installation (launch, etc.) as needed


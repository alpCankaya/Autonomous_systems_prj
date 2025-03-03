cmake_minimum_required(VERSION 3.0.2)
project(mode_launcher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Derlenecek nod: mode_launcher.cpp (bu dosya, "mode_launcher" paketinizin src dizininde yer almalıdır)
add_executable(${PROJECT_NAME}_node src/mode_launcher.cpp)

## Link libraries to the executable
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


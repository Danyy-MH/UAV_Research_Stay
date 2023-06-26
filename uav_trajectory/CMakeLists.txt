cmake_minimum_required(VERSION 3.0.2)
project(uav_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  geometry_msgs
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp rospy
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(lemniscate src/lemniscate.cpp)
target_link_libraries(lemniscate ${catkin_LIBRARIES})
add_dependencies(lemniscate uav_control_generate_messages_cpp)
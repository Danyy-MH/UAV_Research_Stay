cmake_minimum_required(VERSION 3.0.2)
project(uav_px4_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  geometry_msgs
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_px4_control
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

# add python files
catkin_install_python(PROGRAMS src/waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# add c++ files
add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})
add_dependencies(offb_node uav_px4_control_generate_messages_cpp)
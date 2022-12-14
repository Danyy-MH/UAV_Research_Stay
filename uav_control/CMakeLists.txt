cmake_minimum_required(VERSION 3.0.2)
project(uav_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp rospy
)

catkin_install_python(PROGRAMS src/waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(hexa_6dof_dynamics src/hexa_6dof_dynamics.cpp)
target_link_libraries(hexa_6dof_dynamics ${catkin_LIBRARIES})
add_dependencies(hexa_6dof_dynamics uav_control_generate_messages_cpp)

add_executable(pid_control_4dof src/pid_control_4dof.cpp)
target_link_libraries(pid_control_4dof ${catkin_LIBRARIES})
add_dependencies(pid_control_4dof uav_control_generate_messages_cpp)

add_executable(pid_control_6dof src/pid_control_6dof.cpp)
target_link_libraries(pid_control_6dof ${catkin_LIBRARIES})
add_dependencies(pid_control_6dof uav_control_generate_messages_cpp)

add_executable(trajectory_1 src/trajectory_1.cpp)
target_link_libraries(trajectory_1 ${catkin_LIBRARIES})
add_dependencies(trajectory_1 uav_control_generate_messages_cpp)

add_executable(gazebo_broadcaster src/gazebo_broadcaster.cpp)
target_link_libraries(gazebo_broadcaster ${catkin_LIBRARIES})
add_dependencies(gazebo_broadcaster uav_control_generate_messages_cpp)

add_executable(px4_communication src/px4_communication.cpp)
target_link_libraries(px4_communication ${catkin_LIBRARIES})
add_dependencies(px4_communication uav_control_generate_messages_cpp)

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})
add_dependencies(offb_node uav_control_generate_messages_cpp)
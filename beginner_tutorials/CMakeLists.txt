cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS geometry_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(dynamic_model_hexarotor src/dynamic_model_hexarotor.cpp)
target_link_libraries(dynamic_model_hexarotor ${catkin_LIBRARIES})
add_dependencies(dynamic_model_hexarotor beginner_tutorials_generate_messages_cpp)

add_executable(pid_control src/pid_control.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES})
add_dependencies(pid_control beginner_tutorials_generate_messages_cpp)

add_executable(ref src/ref.cpp)
target_link_libraries(ref ${catkin_LIBRARIES})
add_dependencies(ref beginner_tutorials_generate_messages_cpp)

add_executable(hexa_gazebo_broadcaster src/hexa_gazebo_broadcaster.cpp)
target_link_libraries(hexa_gazebo_broadcaster ${catkin_LIBRARIES})
add_dependencies(hexa_gazebo_broadcaster beginner_tutorials_generate_messages_cpp)
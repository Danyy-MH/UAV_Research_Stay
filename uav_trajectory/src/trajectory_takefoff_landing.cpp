/*
    file: trajectory_takeoff_landing.cpp
    date: April, 2023
    author: Adán Márquez
    e-mail: adanmarquez200@outlook.com
    brief: Takeoff and Landing trajectory for UAV

*/

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "mavros_msgs/PositionTarget.h"
#include <UAV.hpp>
#include <sstream>

float x, y, z, roll, pitch, yaw;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_n_landing");
    ros::NodeHandle nh;

    // Declare publishers
    ros::Publisher trajectory_pub = nh.advertise<mavros_msgs::PositionTarget>("trajectory_publisher", 100);

    // Declare ROS rate
    ros::Rate loop_rate(100);

    // Initial conditions
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    // While loop in ROS Node
    ros::Duration(1).sleep();
    while(ros::ok()) {

    }
    return 0;
}
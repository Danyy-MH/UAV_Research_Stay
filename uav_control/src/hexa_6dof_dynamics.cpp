/*
    file: hexarotor_dynamics.cpp
    date: Nov, 2022
    author: Adán Márquez
    e-mail: adanmarquez200@outlook.com
    brief: Mathematical model of a Hexarotor with Newton-Euler

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include <UAV.hpp>
#include <sstream>

using namespace std;

// Build an UAV object

UAV uav = UAV();

// Variables to use

float roll, pitch, yaw;
float u, v, w, p, q, r;

MatrixXf Phi(6,6);
MatrixXf Rib(6,6);
MatrixXf M(6,6);

VectorXf u_fc(6); // feedback controller u
VectorXf fg(6); 
VectorXf forces(6); 
VectorXf pose_x(6);
VectorXf pose_x_dot(6);
VectorXf pose_x_dot_last(6);
VectorXf vel(6);
VectorXf v_dot(6);
VectorXf v_dot_last(6);

void get_forces(const geometry_msgs::Wrench::ConstPtr& msg) {
    u_fc(0) = msg -> force.x;
    u_fc(1) = msg -> force.y;
    u_fc(2) = msg -> force.z;
    u_fc(3) = msg -> torque.x;
    u_fc(4) = msg -> torque.y;
    u_fc(5) = msg -> torque.z;
    // ROS_INFO("Receiving \n Feedback forces : {%f, %f, %f, %f, %f, %f}",
    // u_fc(0), u_fc(1), u_fc(2), u_fc(3), u_fc(4), u_fc(5));
}

// Main loop
int main(int argc, char **argv) {

    ros::init(argc, argv, "dynamic_model_hexarotor");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Twist>("pose_x", 10);
    ros::Publisher pose_dot_pub = n.advertise<geometry_msgs::Twist>("pose_x_dot", 10);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("vel", 10);

    // Declare Subscribers
    ros::Subscriber forces_sub = n.subscribe("Forces", 10, get_forces);

    // Message rate
    ros::Rate loop_rate(100);

    // initial conditions
    pose_x << 0, 0, 0, 0, 0, 0;
    pose_x_dot << 0, 0, 0, 0, 0, 0;
    pose_x_dot_last << 0, 0, 0, 0, 0, 0;

    vel << 0, 0, 0, 0, 0, 0;
    v_dot << 0, 0, 0, 0, 0, 0;
    v_dot_last << 0, 0, 0, 0, 0, 0;

    u_fc << 0, 0, 0, 0, 0, 0;
    forces << 0, 0, 0, 0, 0, 0;

    M << uav.m, 0, 0, 0, 0, 0,
        0, uav.m, 0, 0, 0, 0,
        0, 0, uav.m, 0, 0, 0,
        0, 0, 0, uav.Jxx, 0, 0,
        0, 0, 0, 0, uav.Jyy, 0,
        0, 0, 0, 0, 0, uav.Jzz;

    // While loop in ROS node
    //ros::Duration(4).sleep();
    while(ros::ok()) {

        // dynamics operations
        roll = pose_x(3);
        pitch = pose_x(4);
        yaw = pose_x(5);

        u = vel(0);
        v = vel(1);
        w = vel(2);
        p = vel(3);
        q = vel(4);
        r = vel(5);

        Phi << 0, -r, q, 0, -w, v,
            r, 0, -p , w, 0, -u,
            -q, p, 0,  -v, u, 0,
            0, 0, 0, 0, -r, q, 
            0, 0, 0, r, 0, -p,
            0, 0, 0, -q, p, 0;

        Rib << cos(yaw)*cos(pitch), -cos(roll)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch), sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch), 0, 0, 0,
            cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll), 0, 0, 0,
            -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), 0, 0, 0,
            0, 0, 0, 1, sin(roll)*tan(pitch), cos(roll)*tan(pitch),
            0, 0, 0, 0, cos(roll), -sin(roll),
            0, 0, 0, 0, sin(roll)/cos(pitch), cos(roll)/cos(pitch);

        fg << uav.m*uav.g*sin(pitch),
            -uav.m*uav.g*cos(pitch)*sin(roll),
            -uav.m*uav.g*cos(pitch)*cos(roll),
            0,
            0,
            0;

        forces = u_fc - fg;

        v_dot = M.inverse()*(forces + Phi*M*vel);

        vel = uav.integral_step*(v_dot + v_dot_last)/2 + vel;
        v_dot_last = v_dot;

        pose_x_dot = Rib * vel;

        pose_x = uav.integral_step*(pose_x_dot + pose_x_dot_last)/2 + pose_x;
        pose_x_dot_last = pose_x_dot;

        ROS_INFO("Receiving \n Pose_x : {%f, %f, %f, %f, %f, %f}",
        pose_x(0), pose_x(1), pose_x(2), pose_x(3), pose_x(4), pose_x(5));

        // Create messages
        geometry_msgs::Twist pose_x_msg;
        geometry_msgs::Twist pose_x_dot_msg;
        geometry_msgs::Twist vel_msg;

        // Prepare data to publish message

            // pose_x data
        pose_x_msg.linear.x = pose_x(0);
        pose_x_msg.linear.y = pose_x(1);
        pose_x_msg.linear.z = pose_x(2);
        pose_x_msg.angular.x = pose_x(3);
        pose_x_msg.angular.y = pose_x(4);
        pose_x_msg.angular.z = pose_x(5);

            // pose_x_dot data
        pose_x_dot_msg.linear.x = pose_x_dot(0);
        pose_x_dot_msg.linear.y = pose_x_dot(1);
        pose_x_dot_msg.linear.z = pose_x_dot(2);
        pose_x_dot_msg.angular.x = pose_x_dot(3);
        pose_x_dot_msg.angular.y = pose_x_dot(4);
        pose_x_dot_msg.angular.z = pose_x_dot(5);

            // vel data
        vel_msg.linear.x = vel(0);
        vel_msg.linear.y = vel(1);
        vel_msg.linear.z = vel(2);
        vel_msg.angular.x = vel(3);
        vel_msg.angular.y = vel(4);
        vel_msg.angular.z = vel(5);

        // Publish messages 
        pose_pub.publish(pose_x_msg);
        pose_dot_pub.publish(pose_x_dot_msg);
        vel_pub.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
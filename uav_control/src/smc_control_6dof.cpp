/*
    file: smc_control_6dof.cpp
    date: May, 2023
    author: Adán Márquez
    e-mail: adanmarquez200@outlook.com
    brief: Sliding Mode Controller for a Fully-Actuated Hexarotor

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <UAV.hpp>
#include <sstream>

// Build an UAV object

UAV uav = UAV();

float u, v, w, p, q, r;
float alpha = 0.5; // 

MatrixXf Phi(6,6); // Operator that allows vector product on 6D

MatrixXf M(6,6);
// SMC variables definition

MatrixXf K1(6,6);
MatrixXf K2(6,6);
VectorXf lambda(6);
VectorXf sigma (6);

//Error definition
VectorXf error(6);
VectorXf error_dot(6);

// Pose and vel definition
VectorXf pose_x(6);
VectorXf pose_x_dot(6);

VectorXf vel(6); // body frame velocities
VectorXf gc(6); // gravity compensation

VectorXf u_fc(6); // feedback controller u
VectorXf u_aux(6); // auxiliar u

// quaternion holders
VectorXf q_ref(7);
VectorXf q_ref_dot(7);
VectorXf q_ref_ddot(7);

// xyz & euler angles desired pose, vel ,accel
VectorXf ref(6); // desired pose
VectorXf ref_dot(6); // desired vel
VectorXf ref_ddot(6); // desired accel

void get_pose(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    pose_x(0) = msg -> twist.linear.x;
    pose_x(1) = msg -> twist.linear.y;
    pose_x(2) = msg -> twist.linear.z;
    pose_x(3) = msg -> twist.angular.x;
    pose_x(4) = msg -> twist.angular.y;
    pose_x(5) = msg -> twist.angular.z;
    
}

void get_pose_dot(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    pose_x_dot(0) = msg -> twist.linear.x;
    pose_x_dot(1) = msg -> twist.linear.y;
    pose_x_dot(2) = msg -> twist.linear.z;
    pose_x_dot(3) = msg -> twist.angular.x;
    pose_x_dot(4) = msg -> twist.angular.y;
    pose_x_dot(5) = msg -> twist.angular.z;

    // ROS_INFO("Receiving\n Pose_x_dot : {%f, %f, %f} \n",
    // pose_x_dot(0), pose_x_dot(1), pose_x_dot(2));
}

void get_vel(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    vel(0) = msg -> twist.linear.x;
    vel(1) = msg -> twist.linear.y;
    vel(2) = msg -> twist.linear.z;
    vel(3) = msg -> twist.angular.x;
    vel(4) = msg -> twist.angular.y;
    vel(5) = msg -> twist.angular.z;

    //  ROS_INFO("Receiving\n vel : {%f, %f, %f} \n",
    //  vel(0), vel(1), vel(2));
 }

void get_ref(const geometry_msgs::PoseStamped::ConstPtr& msg) {

     q_ref(0) = msg -> pose.position.x;
     q_ref(1) = msg -> pose.position.y;
     q_ref(2) = msg -> pose.position.z;
     q_ref(3) = msg -> pose.orientation.x;
     q_ref(4) = msg -> pose.orientation.y;
     q_ref(5) = msg -> pose.orientation.z;
     q_ref(5) = msg -> pose.orientation.w;

    //  ROS_INFO("Receiving\n Reference : {%f, %f, %f, %f, %f, %f} \n",
    //  ref(0), ref(1), ref(2), ref(3), ref(4), ref(5));
 }

 void get_ref_dot(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    q_ref_dot(0) = msg -> pose.position.x;
    q_ref_dot(1) = msg -> pose.position.y;
    q_ref_dot(2) = msg -> pose.position.z;
    q_ref_dot(3) = msg -> pose.orientation.x;
    q_ref_dot(4) = msg -> pose.orientation.y;
    q_ref_dot(5) = msg -> pose.orientation.z;
    q_ref_dot(6) = msg -> pose.orientation.w;
 }

void get_ref_ddot(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    q_ref_ddot(0) = msg -> pose.position.x;
    q_ref_ddot(1) = msg -> pose.position.y;
    q_ref_ddot(2) = msg -> pose.position.z;
    q_ref_ddot(3) = msg -> pose.orientation.x;
    q_ref_ddot(4) = msg -> pose.orientation.y;
    q_ref_ddot(5) = msg -> pose.orientation.z;
    q_ref_ddot(6) = msg -> pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_control_6dof");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher forces_pub = n.advertise<geometry_msgs::WrenchStamped>("Forces", 10);

    // Declare Subscribers
    ros::Subscriber pose_sub = n.subscribe("pose_x", 10, get_pose);
    ros::Subscriber pose_dot_sub = n.subscribe("pose_x_dot", 10, get_pose_dot);
    ros::Subscriber vel_sub = n.subscribe("vel", 10, get_vel);
    ros::Subscriber ref_sub = n.subscribe("pos_ref", 10, get_ref);
    ros::Subscriber ref_dot_sub = n.subscribe("pos_dot_ref", 10, get_ref_dot);
    ros::Subscriber ref_ddot_sub = n.subscribe("pos_ddot_ref", 10, get_ref_ddot);

    // ROS rate 
    ros::Rate loop_rate(100);

    // initial conditons
    ref_ddot << 0, 0, 0, 0, 0, 0;

    error << 0, 0, 0, 0, 0, 0;
    error_dot << 0, 0, 0, 0, 0, 0;

    pose_x << 0, 0, 0, 0, 0, 0;
    pose_x_dot << 0, 0, 0, 0, 0, 0;
    vel << 0, 0, 0, 0, 0, 0;
    ref << 0, 0, 0, 0, 0, 0;

    gc << 0, 0, uav.g, 0, 0, 0;

    M << uav.m, 0, 0, 0, 0, 0,
        0, uav.m, 0, 0, 0, 0,
        0, 0, uav.m, 0, 0, 0,
        0, 0, 0, uav.Jxx, 0, 0,
        0, 0, 0, 0, uav.Jyy, 0,
        0, 0, 0, 0, 0, uav.Jzz;

    // While loop in ROS node
    //ros::Duration(4).sleep();
    while(ros::ok()) {

        // PD control operations

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

        Phi = Phi.transpose();

        error = ref - pose_x;
        error_dot = - pose_x_dot;

        // u feedback control 
        u_fc = M*M.inverse()*Phi*M*vel + u_aux + ref_ddot; 

        // Create messages
        geometry_msgs::WrenchStamped feedback_forces;


        // Prepare data to publish message
        feedback_forces.wrench.force.x = u_fc(0);
        feedback_forces.wrench.force.y = u_fc(1);
        feedback_forces.wrench.force.z = u_fc(2);
        feedback_forces.wrench.torque.x = u_fc(3);
        feedback_forces.wrench.torque.y = u_fc(4);
        feedback_forces.wrench.torque.z = u_fc(5);

        feedback_forces.header.frame_id = "hexa_tilted";
        feedback_forces.header.stamp = ros::Time::now();

        // Publish messages
        forces_pub.publish(feedback_forces);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
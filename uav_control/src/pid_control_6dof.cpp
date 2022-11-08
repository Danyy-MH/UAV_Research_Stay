#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include <UAV.hpp>
#include <sstream>

// Build an UAV object

UAV uav = UAV();

float u, v, w, p, q, r;

MatrixXf Phi(6,6); // Operator that allows vector product on 6D

MatrixXf M(6,6);
// PD Gravity compensation values

MatrixXf kp(6,6);
MatrixXf kd(6,6);
VectorXf gravity_comp(6);

VectorXf error(6);
VectorXf error_dot(6);

VectorXf pose_x(6);
VectorXf pose_x_dot(6);

VectorXf vel(6); // body frame velocities

VectorXf u_fc(6); // feedback controller u
VectorXf u_aux(6); // auxiliar u
VectorXf gc(6); // gravity compensation

VectorXf ref(6); // auxiliar u
VectorXf ref_dot(6); // auxiliar u
VectorXf ref_ddot(6); // auxiliar u

void get_pose(const geometry_msgs::Twist::ConstPtr& msg) {

        pose_x(0) = msg -> linear.x;
        pose_x(1) = msg -> linear.y;
        pose_x(2) = msg -> linear.z;
        pose_x(3) = msg -> angular.x;
        pose_x(4) = msg -> angular.y;
        pose_x(5) = msg -> angular.z;

    // ROS_INFO("Receiving\n Pose_x : {%f, %f, %f} \n",
    // pose_x(0), pose_x(1), pose_x(2));
}

void get_pose_dot(const geometry_msgs::Twist::ConstPtr& msg) {

    pose_x_dot(0) = msg -> linear.x;
    pose_x_dot(1) = msg -> linear.y;
    pose_x_dot(2) = msg -> linear.z;
    pose_x_dot(3) = msg -> angular.x;
    pose_x_dot(4) = msg -> angular.y;
    pose_x_dot(5) = msg -> angular.z;

    // ROS_INFO("Receiving\n Pose_x_dot : {%f, %f, %f} \n",
    // pose_x_dot(0), pose_x_dot(1), pose_x_dot(2));
}

void get_vel(const geometry_msgs::Twist::ConstPtr& msg) {

    vel(0) = msg -> linear.x;
    vel(1) = msg -> linear.y;
    vel(2) = msg -> linear.z;
    vel(3) = msg -> angular.x;
    vel(4) = msg -> angular.y;
    vel(5) = msg -> angular.z;

    //  ROS_INFO("Receiving\n vel : {%f, %f, %f} \n",
    //  vel(0), vel(1), vel(2));
 }

void get_ref(const geometry_msgs::Twist::ConstPtr& msg) {

     ref(0) = msg -> linear.x;
     ref(1) = msg -> linear.y;
     ref(2) = msg -> linear.z;
     ref(3) = msg -> angular.x;
     ref(4) = msg -> angular.y;
     ref(5) = msg -> angular.z;

    //  ROS_INFO("Receiving\n Reference : {%f, %f, %f, %f, %f, %f} \n",
    //  ref(0), ref(1), ref(2), ref(3), ref(4), ref(5));
 }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_control");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher forces_pub = n.advertise<geometry_msgs::Wrench>("Forces", 10);

    // Declare Subscribers
    ros::Subscriber pose_sub = n.subscribe("pose_x", 10, get_pose);
    ros::Subscriber pose_dot_sub = n.subscribe("pose_x_dot", 10, get_pose_dot);
    ros::Subscriber vel_sub = n.subscribe("vel", 10, get_vel);
    ros::Subscriber ref_sub = n.subscribe("ref", 10, get_ref);

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

    gc << 0, 0, -uav.m*uav.g, 0, 0, 0;

    M << uav.m, 0, 0, 0, 0, 0,
        0, uav.m, 0, 0, 0, 0,
        0, 0, uav.m, 0, 0, 0,
        0, 0, 0, uav.Jxx, 0, 0,
        0, 0, 0, 0, uav.Jyy, 0,
        0, 0, 0, 0, 0, uav.Jzz;

    kp << 6, 0, 0, 0, 0, 0,
        0, 4.5, 0, 0, 0, 0,
        0, 0, 5, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    kd << 5, 0, 0, 0, 0, 0,
        0, 4.5, 0, 0, 0, 0,
        0, 0, 7, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.2;

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

        error = ref - pose_x;
        error_dot = -pose_x_dot;

        u_aux = kp*error + kd*error_dot + gc;
        u_fc = M*M.inverse()*Phi*M*vel + u_aux + ref_ddot; 

        // Create messages
        geometry_msgs::Wrench feedback_forces;


        // Prepare data to publish message
        feedback_forces.force.x = u_fc(0);
        feedback_forces.force.y = u_fc(1);
        feedback_forces.force.z = u_fc(2);
        feedback_forces.torque.x = u_fc(3);
        feedback_forces.torque.y = u_fc(4);
        feedback_forces.torque.z = u_fc(5);

        // Publish messages
        forces_pub.publish(feedback_forces);

        ros::spinOnce();
        loop_rate.sleep();
        }

  return 0;
}
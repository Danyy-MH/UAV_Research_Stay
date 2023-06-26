#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <UAV.hpp>
#include <sstream>

// Build an UAV object

UAV uav = UAV();

float u, v, w, p, q, r;

MatrixXf Phi(6,6); // Operator that allows vector product on 6D
MatrixXf Phi_T(6,6);

MatrixXf M(6,6);
// PD Gravity compensation values

MatrixXf kp(6,6);
MatrixXf kd(6,6);
VectorXf gravity_comp(6);

VectorXf error = VectorXf::Zero(6);
VectorXf error_dot = VectorXf::Zero(6);

VectorXf pose_x = VectorXf::Zero(6);;
VectorXf pose_x_dot = VectorXf::Zero(6);;

VectorXf vel = VectorXf::Zero(6);; // body frame velocities

VectorXf u_fc(6); // feedback controller u
VectorXf u_aux(6); // auxiliar u
VectorXf gc(6); // gravity compensation

VectorXf ref = VectorXf::Zero(6);
VectorXf ref_dot = VectorXf::Zero(6);
VectorXf ref_ddot = VectorXf::Zero(6); 

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

}

void get_vel(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    vel(0) = msg -> twist.linear.x;
    vel(1) = msg -> twist.linear.y;
    vel(2) = msg -> twist.linear.z;
    vel(3) = msg -> twist.angular.x;
    vel(4) = msg -> twist.angular.y;
    vel(5) = msg -> twist.angular.z;
 }

void get_ref(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ref(0) = msg -> pose.position.x;
    ref(1) = msg -> pose.position.y;
    ref(2) = msg -> pose.position.z;

    // quaternion to euler angles
    tf::Quaternion q_ref;
    tf::quaternionMsgToTF(msg -> pose.orientation, q_ref);
    double msg_phi, msg_theta, msg_psi;
    tf::Matrix3x3(q_ref).getRPY(msg_phi, msg_theta, msg_psi);

    ref(3) = msg_phi;
    ref(4) = msg_theta;
    ref(5) = msg_psi;

 }

 void get_ref_dot(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ref_dot(0) = msg -> pose.position.x;
    ref_dot(1) = msg -> pose.position.y;
    ref_dot(2) = msg -> pose.position.z;
    
    //quaterion to euler angles
    tf::Quaternion q_ref_dot;
    tf::quaternionMsgToTF(msg -> pose.orientation, q_ref_dot);
    double msg_phi_dot, msg_theta_dot, msg_psi_dot;
    tf::Matrix3x3(q_ref_dot).getRPY(msg_phi_dot, msg_theta_dot, msg_psi_dot);

    ref_dot(3) = msg_phi_dot;
    ref_dot(4) = msg_theta_dot;
    ref_dot(5) = msg_psi_dot;

 }

  void get_ref_ddot(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ref_dot(0) = msg -> pose.position.x;
    ref_dot(1) = msg -> pose.position.y;
    ref_dot(2) = msg -> pose.position.z;
    
    //quaterion to euler angles
    tf::Quaternion q_ref_ddot;
    tf::quaternionMsgToTF(msg -> pose.orientation, q_ref_ddot);
    double msg_phi_ddot, msg_theta_ddot, msg_psi_ddot;
    tf::Matrix3x3(q_ref_ddot).getRPY(msg_phi_ddot, msg_theta_ddot, msg_psi_ddot);

    ref_dot(3) = msg_phi_ddot;
    ref_dot(4) = msg_theta_ddot;
    ref_dot(5) = msg_psi_ddot;
 }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_control_6dof");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher forces_pub = n.advertise<geometry_msgs::WrenchStamped>("Forces", 10);
    ros::Publisher error_pub = n.advertise<geometry_msgs::TwistStamped>("error", 10);
    ros::Publisher error_dot_pub = n.advertise<geometry_msgs::TwistStamped>("error_dot", 10);

    // Declare Subscribers
    ros::Subscriber pose_sub = n.subscribe("pose_x", 10, get_pose);
    ros::Subscriber pose_dot_sub = n.subscribe("pose_x_dot", 10, get_pose_dot);
    ros::Subscriber vel_sub = n.subscribe("vel", 10, get_vel);
    ros::Subscriber ref_sub = n.subscribe("pos_ref", 10, get_ref);
    ros::Subscriber ref_dot_sub = n.subscribe("pos_dot_ref", 10, get_ref_dot);
    ros::Subscriber ref_ddot_sub = n.subscribe("pos_ddot_ref", 10, get_ref_ddot);

    // ROS rate 
    ros::Rate loop_rate(100);

    gc << 0, 0, uav.g, 0, 0, 0;

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

        Phi_T = Phi.transpose();

        // define errors
        error = ref - pose_x;
        error_dot = ref_dot- pose_x_dot;

        u_aux = kp*error + kd*error_dot + gc;
        // u feedback control 
        u_fc = M * ( - M.inverse() * Phi_T * M * vel + u_aux + ref_ddot); 

        ROS_INFO("Receiving \n Position_ref : {%f, %f, %f, %f, %f, %f}",
        ref(0), 
        ref(1), 
        ref(2), 
        ref(3), 
        ref(4), 
        ref(5));

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
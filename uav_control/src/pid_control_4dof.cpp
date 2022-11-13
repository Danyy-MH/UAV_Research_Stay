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

// Variables definition

float z_ddot, z_real, z_real_dot, z_d, error_z, error_z_dot, kp_z, kd_z, u_aux_z;
float x_real, x_real_dot, x_d, error_x, error_x_dot, kp_x, kd_x, u_aux_x;
float y_real, y_real_dot, y_d, error_y, error_y_dot, kp_y, kd_y, u_aux_y;
float roll, pitch, yaw, roll_des_calculation, pitch_des_calculation, roll_des, pitch_des;
float U1, tx, ty, tz;
float roll_real, roll_real_dot, roll_d, error_roll, error_roll_dot, kp_roll, kd_roll, u_aux_roll;
float pitch_real, pitch_real_dot, pitch_d, error_pitch, error_pitch_dot, kp_pitch, kd_pitch, u_aux_pitch;
float yaw_real, yaw_real_dot, yaw_d, error_yaw, error_yaw_dot, kp_yaw, kd_yaw, u_aux_yaw;

VectorXf pose_x(6);
VectorXf pose_x_dot(6);

VectorXf vel(6); // body frame velocities

VectorXf ref(6); // auxiliar u

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
    ros::init(argc, argv, "pid_control_4dof");
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

    pose_x << 0, 0, 0, 0, 0, 0;
    pose_x_dot << 0, 0, 0, 0, 0, 0;
    vel << 0, 0, 0, 0, 0, 0;
    ref << 0, 0, 0, 0, 0, 0;

    // While loop in ROS node
    //ros::Duration(4).sleep();
    while(ros::ok()) {

        // Postion PD Control
        // Determine u_uax_z
        z_ddot = 0;
        z_real = pose_x(2);
        z_real_dot = pose_x_dot(2);
        z_d = ref(2);

        error_z = z_d - z_real;
        error_z_dot = - z_real_dot;

        kp_z = 7;
        kd_z = 5;

        u_aux_z = kp_z*error_z + kd_z*error_z_dot; 

         // Determine u_aux_x
        x_real = pose_x(0);
        x_real_dot = pose_x_dot(0);
        x_d = ref(0);

        error_x = x_d - x_real;
        error_x_dot = - x_real_dot;

        kp_x = 2;
        kd_x = 2;

        u_aux_x = kp_x*error_x + kd_x*error_x_dot; 

        // Determine u_aux_y

        y_real = pose_x(1);
        y_real_dot = pose_x_dot(1);
        y_d = ref(1);

        error_y = y_d - y_real;
        error_y_dot =  - y_real_dot;

        kp_y = 2;
        kd_y = 2;

        u_aux_y = kp_y*error_y + kd_y*error_y_dot;

        // Roll and Pitch

        roll = pose_x(3);
        pitch = pose_x(4);
        yaw = ref(5);

        // Calculate thrust U1

        U1 = (uav.m/(cos(roll)*cos(pitch)))*(z_ddot - uav.g + u_aux_z);

        // Calculate Roll and Pitch desired

        roll_des_calculation = ((uav.m/U1)*u_aux_x*sin(yaw)) - ((uav.m/U1)*u_aux_y*cos(yaw));

        if (roll_des_calculation > 1)
        {
            roll_des_calculation = 1;
        }
        else if (roll_des_calculation < -1)
        {
            roll_des_calculation = -1;
        }
        else
        {
            roll_des_calculation = roll_des_calculation;
        }
        roll_des = asin(roll_des_calculation);
        
        pitch_des_calculation = (((uav.m/U1)*u_aux_x) - (sin(roll_des)*sin(yaw)))/(cos(roll_des)*cos(yaw));
        if (pitch_des_calculation > 1)
        {
            pitch_des_calculation = 1;
        }
        else if (pitch_des_calculation < -1)
        {
            pitch_des_calculation = -1;
        }
        else
        {
            pitch_des_calculation = pitch_des_calculation;
        }
        pitch_des = asin(pitch_des_calculation);
        
        // body frame angular velocites

        // Determine u_aux_roll

        roll_real = pose_x(3);
        roll_real_dot = pose_x_dot(3);
        roll_d = roll_des;

        error_roll = roll_d - roll_real;
        error_roll_dot = - roll_real_dot;

        kp_roll = 20;
        kd_roll = 5;

        u_aux_roll = kp_roll*error_roll + kd_roll*error_roll_dot;
        tx = uav.Jxx*((uav.Jzz - uav.Jyy)*pose_x_dot(4)*pose_x_dot(5)/(uav.Jxx) + u_aux_roll);

        // Determine u_aux_pitch

        pitch_real = pose_x(4);
        pitch_real_dot = pose_x_dot(4);
        pitch_d = pitch_des;

        error_pitch= pitch_d - pitch_real;
        error_pitch_dot = - pitch_real_dot;

        kp_pitch = 10;
        kd_pitch = 2;

        u_aux_pitch = kp_pitch*error_pitch + kd_pitch*error_pitch_dot;
        ty = uav.Jyy*((uav.Jxx - uav.Jzz)*pose_x_dot(3)*pose_x_dot(5)/(uav.Jyy) + u_aux_pitch);

        // Determine u_aux_yaw

        yaw_real = pose_x(5);
        yaw_real_dot = pose_x_dot(5);
        yaw_d = ref(5);

        error_yaw = yaw_d - yaw_real;
        error_yaw_dot = - yaw_real_dot;

        kp_yaw = 1;
        kd_yaw = 1;

        u_aux_yaw = kp_yaw*error_yaw + kd_yaw*error_yaw_dot;
        tz = uav.Jzz*((uav.Jyy - uav.Jxx)*pose_x_dot(3)*pose_x_dot(4)/(uav.Jzz) + u_aux_yaw);

        // Create messages
        geometry_msgs::Wrench feedback_forces;


        // Prepare data to publish message
        feedback_forces.force.x = 0;
        feedback_forces.force.y = 0;
        feedback_forces.force.z = U1;
        feedback_forces.torque.x = tx;
        feedback_forces.torque.y = ty;
        feedback_forces.torque.z = tz;

        // Publish messages
        forces_pub.publish(feedback_forces);

        ros::spinOnce();
        loop_rate.sleep();
        }

  return 0;
}
/*
    file: helicoidal.cpp
    date: May, 2023
    author: Adán Márquez
    e-mail: adanmarquez200@outlook.com
    brief: Trajectory publisher helicoidal
    heilocoidal trajectory

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <UAV.hpp>

Vector3f euler_angles = Vector3f::Zero(); // roll, pitch, yaw angles

int main(int argc, char **argv)
{
    ros::init(argc, argv, "helicoidal");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher ref_pub = n.advertise<geometry_msgs::PoseStamped>("pos_ref", 100);
    ros::Publisher ref_dot_pub = n.advertise<geometry_msgs::PoseStamped>("pos_dot_ref", 100);
    ros::Publisher ref_ddot_pub = n.advertise<geometry_msgs::PoseStamped>("pos_ddot_ref", 100);

    // ROS rate 
    ros::Rate loop_rate(100);
    
    // While loop in ROS node
    ros::Duration(2).sleep();
    auto initial_time = ros::Time::now().toSec();

    while(ros::ok()) {

      auto time = ros::Time::now().toSec() - initial_time;
      // Create messages

      geometry_msgs::PoseStamped ref_msg;
      geometry_msgs::PoseStamped ref_dot_msg;
      geometry_msgs::PoseStamped ref_ddot_msg;

      // Prepare data to publish message
        // linear positions
      ref_msg.pose.position.x = 5 * cos(time/2);
      ref_msg.pose.position.y = 5 * sin(time/2);
        // Linear vel
      ref_dot_msg.pose.position.x = -2.5 * sin(time/2);
      ref_dot_msg.pose.position.y = 2.5 * cos(time/2);
        // linear accel
      ref_ddot_msg.pose.position.x = - 1.25 * cos(time/2);
      ref_ddot_msg.pose.position.y = - 1.25 * sin(time/2);
      ref_ddot_msg.pose.position.z = 0;

      if (time > 60) {
        ref_msg.pose.position.z = 5;
        ref_dot_msg.pose.position.z = 0;
      }
      else {
        ref_msg.pose.position.z = time/12;
        ref_dot_msg.pose.position.z = 1/12;
      }
      if (time > 5) {
        // euler_angles(0) = 0.1; // inclinación de 0.1 rad en roll
        euler_angles(0) = 0.0;
      }

      // euler angles to quaternion
      tf::Quaternion q = tf::createQuaternionFromRPY(euler_angles(0), euler_angles(1), euler_angles(2));
      
      // Quaternion based orientations
      ref_msg.pose.orientation.x = q.getX();
      ref_msg.pose.orientation.y = q.getY();
      ref_msg.pose.orientation.z = q.getZ();
      ref_msg.pose.orientation.w = q.getW();

      ref_dot_msg.pose.orientation.x = 0;
      ref_dot_msg.pose.orientation.y = 0;
      ref_dot_msg.pose.orientation.z = 0;
      ref_dot_msg.pose.orientation.w = 1;

      ref_ddot_msg.pose.orientation.x = 0;
      ref_ddot_msg.pose.orientation.y = 0;
      ref_ddot_msg.pose.orientation.z = 0;
      ref_ddot_msg.pose.orientation.w = 1;

      // header
      ref_msg.header.stamp = ros::Time::now();
      ref_msg.header.frame_id = "trajectory";

      // Publish messages
      ref_pub.publish(ref_msg);
      ref_dot_pub.publish(ref_dot_msg);
      ref_ddot_pub.publish(ref_ddot_msg);

      ros::spinOnce();
      loop_rate.sleep();
      }

  return 0;
}
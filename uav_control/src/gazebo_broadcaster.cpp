#include <math.h>
#include <ros/ros.h>
#include <UAV.hpp>
#include <geometry_msgs/TwistStamped.h>
#include "gazebo_msgs/ModelState.h"
#include <tf2/LinearMath/Quaternion.h>

// Build and UAV object

UAV uav = UAV();

VectorXf pose_x(6);

void get_pose(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	pose_x(0) = msg -> twist.linear.x;
    pose_x(1) = msg -> twist.linear.y;
    pose_x(2) = msg -> twist.linear.z;
    pose_x(3) = msg -> twist.angular.x;
    pose_x(4) = msg -> twist.angular.y;
    pose_x(5) = msg -> twist.angular.z;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hexa_gazebo_broadcaster");
	ros::NodeHandle n;

    // Message rate
	ros::Rate loop_rate(100);

    // Declare Subscribers
	
	ros::Subscriber pos = n.subscribe("pose_x",10, get_pose);
	
    // Declare Publishers
	gazebo_msgs::ModelState states;
	ros::Publisher gazebo_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10);

	states.model_name = "hexa_urdf";
	tf2::Quaternion myQuaternion;
	tf2::Quaternion q;
	tf2::Quaternion q_rot;

	//ros::Duration(4).sleep();
	while(ros::ok())
	{
		// euler angles to quaternion
		myQuaternion.setRPY(pose_x(3), pose_x(4),pose_x(5));
		q_rot.setRPY(0,0,0);
		q = q_rot * myQuaternion;
		q.normalize();
		
		states.pose.position.x = pose_x(0);
		states.pose.position.y = pose_x(1);
		states.pose.position.z = pose_x(2);
		
		states.pose.orientation.x = q.x();
		states.pose.orientation.y = q.y();
		states.pose.orientation.z = q.z();
		states.pose.orientation.w = q.w();
		
		gazebo_pub.publish(states);
		
		ros::spinOnce();
		loop_rate.sleep();	
		
	}

    return 0;	
}
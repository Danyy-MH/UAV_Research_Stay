#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

float x, y, z, roll, pitch, yaw;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ref");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher ref_pub = n.advertise<geometry_msgs::Twist>("ref", 100);

    // ROS rate 
    ros::Rate loop_rate(100);

    // initial conditons
    x = 0;
    y = 0;
    z = 2;
    roll = 0;
    pitch = 0;
    yaw = 0;

    // While loop in ROS node
    //ros::Duration(4).sleep();
    while(ros::ok()) {

        // reference

        // Create messages
        geometry_msgs::Twist ref_msg;


        // Prepare data to publish message
        ref_msg.linear.x = x;
        ref_msg.linear.y = y;
        ref_msg.linear.z = z;
        ref_msg.angular.x = roll;
        ref_msg.angular.y = pitch;
        ref_msg.angular.z = yaw;

        // Publish messages
        ref_pub.publish(ref_msg);

        ros::spinOnce();
        loop_rate.sleep();
        }

  return 0;
}
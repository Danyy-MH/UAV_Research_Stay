#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

float x, y, z, roll, pitch, yaw;
float p, q, r, u_dot, v_dot, w_dot;

void get_imu_data_raw(const sensor_msgs::Imu::ConstPtr& msg) {
    p = msg -> angular_velocity.x;
    q = msg -> angular_velocity.y;
    r = msg -> angular_velocity.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_communication");
    ros::NodeHandle n;

    // Declare Publishers
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("body_frame_vel", 100);

    // Declare Subscribercd
    ros::Subscriber imu_data_raw_pub = n.subscribe("/mavros/imu/data_raw", 10, get_imu_data_raw);

    // ROS rate 
    ros::Rate loop_rate(100);

    // initial conditons
    p = 0;
    q = 0;
    r = 0;
 
    // While loop in ROS node
    ros::Duration(4).sleep();
    while(ros::ok()) {

        // reference

        // Create messages
        geometry_msgs::Twist vel_msg;

        // Prepare data to publish message
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_msg.angular.x = p;
        vel_msg.angular.y = q;
        vel_msg.angular.z = r;
    
        vel_pub.publish(vel_msg);


        ros::spinOnce();
        loop_rate.sleep();
        }

  return 0;
}
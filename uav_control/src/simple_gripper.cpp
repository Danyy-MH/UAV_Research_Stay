#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){

ros::init(argc, argv, "gripper");
ros::NodeHandle n;

ros::Rate loop_rate(10);

ros::Subscriber gripper_sub = n.subscribe("grip_command", 1000);

//ros::Publisher gripper_pub = n.advertise<geometry_msgs::Twist>("gripper", 100);

//geometry_msgs::Twist msg;

if (gripper_sub == "open"){

msg.data = 0.8;

gripper_pub.publish(msg);
}

if (gripper_sub == "close"){

msg.data = 0.2;

gripper_pub.publish(msg);
}

ros::spin();

return 0;
}

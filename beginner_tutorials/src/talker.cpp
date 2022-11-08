
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int rate = 100;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "dynamics ";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
#!/usr/bin/env python

import rospy
import time
from math import sqrt
from geometry_msgs.msg import PoseStamped

def states_pub():
    rospy.init_node("states_pub", anonymous = False)
    ref_pub = rospy.Publisher("uav_state", PoseStamped, queue_size=10)
    rate = rospy.Rate(100)

    rospy.loginfo("Reference node online")
    # time.sleep(10)
    # rospy.loginfo("time")
    while not rospy.is_shutdown():
        ref_pose = PoseStamped()
        
        ref_pose.pose.position.x = 0
        ref_pose.pose.position.y = 0
        ref_pose.pose.position.z = 0

        ref_pose.pose.orientation.x = 0
        ref_pose.pose.orientation.y = 0
        ref_pose.pose.orientation.z = 0
        ref_pose.pose.orientation.w = 0

        #ref_pose.header.stamp = rospy.Time.now
        ref_pose.header.stamp.nsecs = 0
        ref_pose.header.stamp.secs = 0
        ref_pose.header.frame_id = "uav_state"

        ref_pub.publish(ref_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        states_pub()
    except rospy.ROSInterruptException:
        pass
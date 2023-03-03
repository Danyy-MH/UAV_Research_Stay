#!/usr/bin/env python

import rospy
import time
from math import sqrt
from geometry_msgs.msg import PoseStamped

current_wp = 0

## List of waypoints
## xyz NED frame and quaternions xyzw(Gazebo)
waypoints = [[0, 0, 1.5, 0, 0, 0, 0],
             [3, 0, 1.5, 0, 0, 0, 0],
             [3, 3, 1.5, 0, 0, 0, 0],
             [0, 3, 1, 0, 0, 0, 0],
             [0, 0, 1.5, 0, 0, 0, 0],
             [0, 0, 1.5, 0, 0, 0, 0],]

# waypoints = [[25.652828, -100.286169, 1.5, 0, 0, 0, 0],
#              [25.6528276, -100.2861094, 1.5, 0, 0, 0, 0],
#              [25.6528651, -100.2861087, 1.5, 0, 0, 0, 0],
#              [25.652828, -100.286169, 1.5, 0, 0, 0, 0]]

## Calculate distance between current distance and waypoint
def get_distance(pointA, pointB):
    distance = sqrt((pointA[0] - pointB[0])**2 + (pointA[1] - pointB[1])**2 + (pointA[2] - pointB[2])**2)
    return distance

def get_pose(data):
    global current_wp
    global waypoints
    current_pose = [round(data.pose.position.x, 3), round(data.pose.position.y, 3), round(data.pose.position.z, 3)]
    rospy.loginfo("Current wp : " + str(current_wp))
    rospy.loginfo("Distance : " + str(get_distance(waypoints[current_wp], current_pose)))
    rospy.loginfo("Current pose : " + str(current_pose))
    if get_distance(waypoints[current_wp], current_pose) < 0.5 and current_wp < len(waypoints)-1: 
        #rospy.sleep(1)
        current_wp = current_wp + 1
        #rospy.loginfo("Reached goal " + str(current_wp+1) + ", moving to next")
    # if current_wp == len(waypoints)-1:
        #rospy.sleep(3)

        # current_wp = 0

def ref_pub():
    rospy.init_node("ref_pub", anonymous = False)
    ref_pub = rospy.Publisher("wp_ref", PoseStamped, queue_size=10)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, get_pose)
    rate = rospy.Rate(100)

    rospy.loginfo("Reference node online")
    # time.sleep(10)
    # rospy.loginfo("time")
    while not rospy.is_shutdown():
        ref_pose = PoseStamped()
        
        ref_pose.pose.position.x = waypoints[current_wp][0]
        ref_pose.pose.position.y = waypoints[current_wp][1]
        ref_pose.pose.position.z = waypoints[current_wp][2]

        ref_pose.pose.orientation.x = waypoints[current_wp][3]
        ref_pose.pose.orientation.y = waypoints[current_wp][4]
        ref_pose.pose.orientation.z = waypoints[current_wp][5]
        ref_pose.pose.orientation.w = waypoints[current_wp][6]

        #ref_pose.header.stamp = rospy.Time.now
        ref_pose.header.stamp.nsecs = 0
        ref_pose.header.stamp.secs = 0
        ref_pose.header.frame_id = "wp_ref"

        ref_pub.publish(ref_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        ref_pub()
    except rospy.ROSInterruptException:
        pass
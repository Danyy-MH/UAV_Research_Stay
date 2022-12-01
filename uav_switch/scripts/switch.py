#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from std_msgs.msg import String, Int64
import time

class Switch():
    def __init__(self):
        self.aruco = ""
        self.nodo = "waypoints"
        self.waypoint = 0

    def get_pose(self,msg):
        if(msg.pose.position.z < 0.55 and msg.pose.position.z > 0.5):
            self.despegue = True
        if(msg.pose.position.z < .1 and msg.pose.position.z > 0):
            self.aterrizaje = True

    def get_aruco(self,msg):
        self.aruco = msg.data
    
    def get_waypoint(self,msg):
        self.waypoint = msg.data

    def control(self):
        if(self.nodo == "waypoints" and self.waypoint == 2):
            print("Cambiando de wp a vs")
            self.nodo = "arucos"
            wp_pub.publish(0)
            aruco_pub.publish("Activada")
        if(self.nodo == "arucos" and self.aruco == "Centrado"):
            print("Cambiando de vs a wp")
            self.nodo = "waypoints"
            aruco_pub.publish("Desactivada")
            wp_pub.publish(1)
            time.sleep(10)


switch = Switch()
rospy.init_node('Switch')
rospy.Subscriber("mavros/local_position/pose", PoseStamped, switch.get_pose)
rospy.Subscriber("aruco",String,switch.get_aruco)
rospy.Subscriber("waypoint",Int64,switch.get_waypoint)
aruco_pub = rospy.Publisher("ibvs/camera_state",String,queue_size=10)
wp_pub = rospy.Publisher("wp/state", Int64, queue_size=10)
while not rospy.is_shutdown():
    switch.control()

rospy.spin()
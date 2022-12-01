#!/usr/bin/env python3

import rospy
import subprocess
import signal
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from std_msgs.msg import String

class Control:
    def __init__(self):
        self.estado = ""
        self.aruco = ""
        self.despegue = False
        self.aterrizaje = False
        self.nodo = "waypoints"
        self.waypoint = 1
    
    def get_pose(self, msg):
        if(msg.pose.position.z < 0.55 and msg.pose.position.z > 0.5):
            self.despegue = True
        if(msg.pose.position.z < .1 and msg.pose.position.z > 0):
            self.aterrizaje = True
    
    def get_aruco(self,msg):
        self.aruco=msg.data

    def simulation(self):
        gps = subprocess.Popen(["rosrun","uav_control","offboard"])
        self.estado = "Llendo al objetivo"
        while(not self.despegue):
            self.estado = "Despegando"
        vs = subprocess.Popen(["rosrun","uav_ibvs","arucos.py"]) 
        gps.send_signal(signal.SIGINT)
        aruco_pub.publish("Activada")
        self.estado = "Detectando el aruco"
        while(self.aruco != "Centrado"):
            self.estado = "Centrando aruco"
        land = subprocess.Popen(["rosrun", "uav_control", "landing"])
        vs.send_signal(signal.SIGINT)
        while(not self.aterrizaje):
            self.estado = "Aterrizando"
        gps = subprocess.Popen(["rosrun","uav_control","offboard"])
        land.send_signal(signal.SIGINT)
        self.estado = "Volviendo a la base"
        rospy.sleep(25)
        land = subprocess.Popen(["rosrun", "uav_control", "landing"])
        gps.send_signal(signal.SIGINT)
        print("Aterrizando")
        rospy.sleep(20)
        land.send_signal(signal.SIGINT)

    def switch(self):
        if(self.nodo == "waypoints" and self.waypoint == 2):
            aruco_pub.publish("Activada")
            self.nodo = "vs"
        if(self.nodo == "vs" and self.aruco == "Centrado"):
            aruco_pub.publish("Desactivada")
            self.nodo = "waypoints"
    


switch = Control()
rospy.init_node('Operador_py')
rospy.Subscriber("mavros/local_position/pose", PoseStamped, switch.get_pose)
rospy.Subscriber("aruco",String,switch.get_aruco)
aruco_pub = rospy.Publisher("ibvs/camera_state",String,queue_size=10)
switch.simulation()

rospy.spin()


#!/usr/bin/env python3

import rospy
import subprocess
import signal
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from std_msgs.msg import String

class Control:
    def __init__(self):
        self.estado = "pick"
        self.aruco = ""
        self.despegue = False
        self.aterrizaje = False
    
    def get_pose(self, msg):
        print(msg.pose.position.z)
        if(msg.pose.position.z < 1.55 and msg.pose.position.z > 1.5):
            self.despegue = True
        if(msg.pose.position.z < .05 and msg.pose.position.z > 0):
            self.aterrizaje = True
    
    def get_aruco(self,msg):
        self.aruco=msg

    def simulation(self):
        gps = subprocess.Popen(["rosrun","uav_control","offboard"])
        print("Llendo al objetivo")
        while(not self.despegue):
            print("Despegando")
        vs = subprocess.Popen(["rosrun","uav_ibvs","arucos.py"]) 
        gps.send_signal(signal.SIGINT)
        print("Detectando el aruco")
        while(self.aruco != "Centrado"):
            print("Centrando aruco")
        land = subprocess.Popen(["rosrun", "uav_control", "landing"])
        vs.send_signal(signal.SIGINT)
        while(not self.aterrizaje):
            print("Aterrizando")
        gps = subprocess.Popen(["rosrun","uav_control","offboard"])
        land.send_signal(signal.SIGINT)
        print("Volviendo a la base")
        rospy.sleep(25)
        land = subprocess.Popen(["rosrun", "uav_control", "landing"])
        gps.send_signal(signal.SIGINT)
        print("Aterrizando")
        rospy.sleep(20)
        land.send_signal(signal.SIGINT)


switch = Control()
rospy.init_node('Operador_py')
rospy.Subscriber("mavros/local_position/pose", PoseStamped, switch.get_pose)
rospy.Subscriber("aruco",String,switch.get_aruco)
switch.simulation()

rospy.spin()


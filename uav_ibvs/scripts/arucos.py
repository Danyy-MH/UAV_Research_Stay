#!/usr/bin/env python3
#Autor: Mizael Beltran Romero
#Email: elmizabeltran@gmail.com
#Resumen: Codigo para deteccion de arucos
import cv2
import rospy
import time
import sys
import numpy as np
import tf_conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from std_msgs.msg import String
import geometry_msgs.msg
import cv2.aruco as aruco
import turtlesim.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class PID:

    def __init__(self):
        self.last_ex = 0
        self.last_ey = 0
        self.last_ez = 0
        self.total_ex = 0
        self.total_ey = 0
        self.total_ez = 0
        self.kp = 0.1
        self.ki = 0.05
        self.kd = 0.1
    
    def control(self,ex,ey,ez):
        d_ex = (ex-self.last_ex)*0.01
        d_ey = (ex-self.last_ey)*0.01

        total_ex = self.total_ex + ex/0.01
        total_ey = self.total_ey + ey/0.01

        velocidades = Twist()
        velocidades.linear.y = self.kp*ex + self.kd*d_ex + self.ki*total_ex
        velocidades.linear.x = self.kp*ey + self.kd*d_ey + self.ki*total_ey


        #print('Velocidad x:{}'.format(velocidades.linear.x))
        #print('Velocidad y:{}'.format(velocidades.linear.y))

        vel.publish(velocidades)

        self.last_ex = ex
        self.last_ey = ey


class Camera:
    def __init__(self):
        self.camara = "Apagada"

    
    def aruco_camera(self,msg):
        self.camara = msg.data

    def IBVS(self,msg):
        if(self.camara == "Activada"):
            img = bridge.imgmsg_to_cv2(msg,"bgr8")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #Cambiar direccion hacia donde estan los archivos
            k = np.load("/home/mizil/catkin_ws/src/UAV_Research_Stay/uav_ibvs/visual servoing/calibration_matrix.npy")
            d = np.load("/home/mizil/catkin_ws/src/UAV_Research_Stay/uav_ibvs/visual servoing/distortion_coefficients.npy")
            arucoDict = aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            arucoParams = aruco.DetectorParameters_create()
            (corners, ids, rejected) = aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
            if len(corners) > 0:
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
                    errorx = (tvec[0][0][0]) 
                    errory = (tvec[0][0][1]) 
                    errorz = (tvec[0][0][2])
                    #print('Translacion x: {}'.format(tvec[0][0][0]))
                    #print('Translacion y: {}'.format(tvec[0][0][1]))
                    #print('Error Translacion x: {}'.format(errorx))
                    #print('Error Translacion y: {}'.format(errory))
                    #print('Error Rotacion x: {}'.format(rvec[0][0][0]))
                    #print('Error Rotacion y: {}'.format(rvec[0][0][1]))
                    #print('Error Rotacion z: {}'.format(rvec[0][0][2]))
                    trans.publish(tvec[0][0][0],tvec[0][0][1],tvec[0][0][2])
                    rot.publish(rvec[0][0][0],rvec[0][0][1],rvec[0][0][2])
                    if errorx < 0.01 and errorx > -0.01 and errory < 0.01 and errory > -0.01: #and errorz <0.1:
                        print("Estable")
                        errorx = 0
                        errory = 0
                        errorz = 0
                        pid.control(errorx,errory,errorz)
                        aviso.publish("Centrado")
                    error.publish(errory,errorx,errorz)
                    pid.control(errorx,errory,errorz)
                    img = aruco.drawDetectedMarkers(img, corners, ids)
                    cv2.drawFrameAxes(img, k, d, rvec, tvec, 0.01) 
            cv2.imshow('Aruco',img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

def get_pose(msg):
    pose.pose.position.x = msg.pose.position.x
    pose.pose.position.y = msg.pose.position.y
    pose.pose.position.z = 0.5
    pos_pub.publish(pose)


pid = PID()
camera = Camera()
bridge = CvBridge()
rospy.init_node('IBVS_py')
trans = rospy.Publisher('posicion/translacion', Point, queue_size=10)
rot = rospy.Publisher("posicion/rotacion", Point, queue_size=10)
error = rospy.Publisher("error", Vector3, queue_size=10)
ref = rospy.Publisher("ref",Twist, queue_size=10)
aviso = rospy.Publisher("aruco",String, queue_size=10)
vel = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
rospy.Subscriber("/iris/usb_cam/image_raw",Image,camera.IBVS)
#rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,IBVS)
rospy.Subscriber("mavros/local_position/pose", PoseStamped, get_pose)
rospy.Subscriber("ibvs/camera_state", String, camera.aruco_camera)

pose = PoseStamped()


rospy.spin()
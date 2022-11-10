#!/usr/bin/env python3
#Autor: Mizael Beltran Romero
#Email: elmizabeltran@gmail.com
#Resumen: Codigo para deteccion de arucos
import cv2
import rospy
import sys
import numpy as np
import tf_conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist, Vector3
import geometry_msgs.msg
import cv2.aruco as aruco
import turtlesim.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''
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
        d_ez = (ex-self.last_ez)*0.01
        total_ex = self.total_ex + ex/0.01
        total_ey = self.total_ey + ey/0.01
        total_ez = self.total_ez + ez/0.01

        vel_x = self.kp*ex + self.kd*d_ex + self.ki*total_ex
        vel_y = self.kp*ey + self.kd*d_ey + self.ki*total_ey
        vel_z = self.kp*ez + self.kd*d_ez + self.ki*total_ez

        print('Velocidad x:{}'.format(vel_x))
        print('Velocidad y:{}'.format(vel_y))
        print('Velocidad z:{}'.format(vel_z))

        self.last_ex = ex
        self.last_ey = ey
        self.last_ez = ez
'''

def IBVS(msg):
    img = bridge.imgmsg_to_cv2(msg,"bgr8")
    img = cv2.circle(img,(400,400),0,(255, 0, 255),thickness=10)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    k = np.load("visual servoing/calibration_matrix.npy")
    d = np.load("visual servoing/distortion_coefficients.npy")
    arucoDict = aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = aruco.DetectorParameters_create()
    (corners, ids, rejected) = aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
    #print('detected: {}'.format(ids))
    if len(corners) > 0:
        for i in range(0, len(ids)):
            #print('ID: {}; Corners: {}'.format(i, corners[i]))
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
            errorx = 10*(0.01-tvec[0][0][0]) #Error Gzaebo 0.012370023907240402 <- 0
            errory = 10*(0.03-tvec[0][0][1]) #Error Gazebo 0.035540042577308875 <- 0
            errorz = 10*(0.03-tvec[0][0][2])
            error_r = 3-rvec[0][0][0]
            error_p = 0.1-rvec[0][0][1]
            error_y = 0.1-rvec[0][0][2]
            print('Translacion x: {}'.format(tvec[0][0][0]))
            print('Translacion y: {}'.format(tvec[0][0][1]))
            print('Translacion z: {}'.format(tvec[0][0][2]))
            print('Error Translacion x: {}'.format(errorx))
            print('Error Translacion y: {}'.format(errory))
            print('Error Translacion z: {}'.format(errorz))
            #print('Rotacion x: {}'.format(rvec[0][0][0]))
            #print('Rotacion y: {}'.format(rvec[0][0][1]))
            #print('Rotacion z: {}'.format(rvec[0][0][2]))
            #print('Error Rotacion x: {}'.format(error_r))
            #print('Error Rotacion y: {}'.format(error_p))
            #print('Error Rotacion z: {}'.format(error_y))
            trans.publish(tvec[0][0][0],tvec[0][0][1],tvec[0][0][2])
            rot.publish(rvec[0][0][0],rvec[0][0][1],rvec[0][0][2])
            orientado = False
            if corners[0][0][0][0] < corners[0][0][2][0]:
                if corners[0][0][0][1] < corners[0][0][2][1]:
                    print("Aruco bien orientado")
                    orientado = True
                else:
                    print("Punto abajo")
            else:
                if corners[0][0][0][1] < corners[0][0][2][1]:
                    print("Punto derecha")
                else:
                    print("Punto abajo/derecha")
            if orientado:
                if True or error_r < 0.15 and error_r > -0.15 and error_p < 0.15 and error_p > -0.15 and error_y <0.15 and error_y > -0.15:
                    print("Rotacion correcta")
                    if errorx < 0.1 and errorx > -0.1 and errory < 0.1 and errory > -0.1 and errorz <0.1:
                        print("Estable")
                        errorx = 0
                        errory = 0
                        errorz = 0
            error.publish(errorx,errory,errorz)
            #pid.control(errorx,errory,errorz)
            #print(corners)
            img = aruco.drawDetectedMarkers(img, corners, ids)
            cv2.rectangle(img,(340,340),(460,460),(255,0,255),thickness=2)
            cv2.drawFrameAxes(img, k, d, rvec, tvec, 0.01) 
    cv2.imshow('Aruco',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()


bridge = CvBridge()
rospy.init_node('IBVS_py')
trans = rospy.Publisher('posicion/translacion', Point, queue_size=10)
rot = rospy.Publisher("posicion/rotacion", Point, queue_size=10)
error = rospy.Publisher("error", Vector3, queue_size=10)
ref = rospy.Publisher("/ref", Twist, queue_size=10)
rospy.Subscriber("/camera/image_raw",Image,IBVS)



#pid = PID()
rospy.spin()

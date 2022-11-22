#!/usr/bin/env python3
import matplotlib.pyplot as plt
from itertools import count
import rospy
from geometry_msgs.msg import Point, Twist

def rotacion(msg):
    rotx.append(msg.x)
    roty.append(msg.y)
    rotz.append(msg.z)
    r.append(next(index2))


def translacion(msg):
    transx.append(msg.x)
    transy.append(msg.y)
    transz.append(msg.z)
    t.append(next(index))

def velocidad(msg):
    velx.append(msg.linear.x)
    v.append(next(index3))


rospy.init_node('Analisis_py')
index = count()
index2 = count()
index3 = count()
t = []
r = []
v = []
transx = []
transy = []
transz = []
velx = []
rotx = []
roty = []
rotz = []
rospy.Subscriber("posicion/rotacion", Point, rotacion, queue_size=10) 
rospy.Subscriber("posicion/translacion", Point, translacion, queue_size=10) 
rospy.Subscriber("mavros/setpoint_velocity/cmd_vel", Twist, velocidad)

while not rospy.is_shutdown(): # run the node until Ctrl-C is pressed
    '''fig, (ax1,ax2) = plt.subplots(1,2)
    fig.suptitle("Translacion/Rotacion")
    ax1.plot(t,transx, label = "X")
    ax1.plot(t,transy, label = "Y")
    ax1.plot(t,transz, label = "Z")
    ax1.legend()
    ax1.set_title("Translacion")
    ax2.plot(r,rotx, label = "X")
    ax2.plot(r,roty, label = "Y")
    ax2.plot(r,rotz, label = "Z")
    ax2.legend()
    ax2.set_title("Rotacion")'''
    plt.plot(v,velx,label = "X")
    plt.show()
	
	#rate.sleep() # This makes the loop to iterate at 10Hz i.e., 10 times a sec
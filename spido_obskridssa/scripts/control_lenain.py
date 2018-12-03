#!/usr/bin/env python2
# -*- coding: utf-8 -*-

################################################################################
# enregistrer des courbes différents Kp et Kd
# Adaptative control for car like vehicles guidance relying on RTK GPS:
#                      rejection of sliding effects in agricultural applications
#                       R. Lenain, B. Thuilot, C. Cariou and P. Martinet
################################################################################
import rospy
from nav_msgs.msg import Odometry
from spido_pure_interface.msg import cmd_car
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Point
import tf

from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round

from matplotlib import pyplot
import numpy
from math import *

from Parameters import *
import numpy as np


import sys


###############################################
from spido_pure_interface.msg import cmd_drive
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Imu, NavSatFix


##############################################

#############  Contoller Function ############################################
def deltaFunction(Kp, Kd, y, theta_tilde, c, dc_ds):
    A = dc_ds*y*tan(theta_tilde)
    B = -Kd*(1-c*y)*tan(theta_tilde)
    C = -Kp*y
    D = c*(1-c*y)*pow(tan(theta_tilde), 2)
    E = c*cos(theta_tilde)/(1-c*y)
    F = pow(cos(theta_tilde),3)/pow((1-c*y),2)
    G = axles_dist*(F*(A+B+C+D)+E)
    return atan(G)

#############  Contoller Function ############################################
################################################################################
def ImuCallback(odom_message):
	#rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.steering_angle)
	global Qx, Qy, Qz, Qw, Xp,Yp, Psip,Psi
	Qx=odom_message.pose.pose.orientation.x
	Qy=odom_message.pose.pose.orientation.y
	Qz=odom_message.pose.pose.orientation.z
	Qw=odom_message.pose.pose.orientation.w
	Xp=odom_message.twist.twist.linear.x
	Yp=odom_message.twist.twist.linear.y
   	Psip=odom_message.twist.twist.angular.z
	Psi=atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz))

################################################################################
def retour_gps (msg):
	global longitude,latitude, X ,Y
	longitude= msg.longitude;
	latitude= msg.latitude;
	X=Rt*(longitude - longitude_init)*(pi/180)*cos((pi/180)*latitude_init)
	Y=Rt*(latitude - latitude_init)*(pi/180)

################################################################################

def curvature():

    Traj=np.loadtxt('/home/summit/Spido_ws/src/spido_riding2/scripts/mat.txt')

    tarjT = Traj[:,0]
    trajX = Traj[:,1]
    trajY = Traj[:,2]
    Yaw = Traj[:,3]
    Psip= Traj[:,3]
    cpt = trajX.size

    s_vect = np.zeros(cpt)
    theta_s = np.zeros(cpt)
    c = np.zeros(cpt)

# s
    for i in range(1, cpt):
        s_vect[i] = s_vect[i-1] + hypot(trajX[i]-trajX[i-1],trajY[i]-trajY[i-1])


# theta_s
    for i in range(cpt):
        if i<cpt-1:
            angle = atan2(trajY[i+1]-trajY[i],trajX[i+1]-trajX[i])
            if angle < 0 :
                angle += 2*pi
                theta_s[i] = angle
            else:
                angle = theta_s[cpt-2]
                if angle < 0 :
                    angle += 2*pi
                    theta_s[i] = angle

# c
    for i in range(cpt):
        if i<cpt-1:
                c[i] = (theta_s[i+1]-theta_s[i])/(s_vect[i+1]-s_vect[i])
        elif i == cpt-1:
                c[i] = c[cpt-2]


    #TrajRe=np.array([[tarjT],[trajX],[trajY],[Yaw],[s_vect,c]])
    return tarjT,trajX,trajY,Yaw,Psip,s_vect,c

######################################################################################

def talker():
	cmd_publisher= rospy.Publisher('cmd_drive', cmd_drive,queue_size=10)
	data_pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
	rospy.Subscriber("/IMU", Odometry, ImuCallback)
	rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
	rospy.init_node('LQR', anonymous=True)
	r = rospy.Rate(200) # 10hz
	simul_time = rospy.get_param('~simulation_time', '10')


    Traj=np.loadtxt('/home/summit/Spido_ws/src/spido_lqr/scripts/TraRef.txt')

    tarjT = Traj[:,0]
    trajX = Traj[:,1]
    trajY = Traj[:,2]
    Yaw = Traj[:,3]
    Psip= Traj[:,3]
    cpt = trajX.size

    s_vect = np.zeros(cpt)
    theta_s = np.zeros(cpt)
    c = np.zeros(cpt)

# s
    for i in range(1, cpt):
        s_vect[i] = s_vect[i-1] + hypot(trajX[i]-trajX[i-1],trajY[i]-trajY[i-1])


# theta_s
    for i in range(cpt):
        if i<cpt-1:
            angle = atan2(trajY[i+1]-trajY[i],trajX[i+1]-trajX[i])
            if angle < 0 :
                angle += 2*pi
                theta_s[i] = angle
            else:
                angle = theta_s[cpt-2]
                if angle < 0 :
                    angle += 2*pi
                    theta_s[i] = angle

# c
    for i in range(cpt):
        if i<cpt-1:
                c[i] = (theta_s[i+1]-theta_s[i])/(s_vect[i+1]-s_vect[i])
        elif i == cpt-1:
                c[i] = c[cpt-2]


    print c

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

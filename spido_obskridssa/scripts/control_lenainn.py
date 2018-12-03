#!/usr/bin/env python2
# -*- coding: utf-8 -*-

################################################################################
# enregistrer des courbes différents Kp et Kd
# Model predictive control for vehicle guidance in presence of sliding: 
#application to farm vehicles path tracking  2005
#                       R. Lenain, B. Thuilot, C. Cariou and P. Martinet
################################################################################
from __future__ import division

import rospy
import numpy as np
import numpy.linalg as alg
import matplotlib.pyplot as plt
import pylab as pl
from pylab import *
#from scipy import *
from scipy.linalg import solve_continuous_are
import time
from nav_msgs.msg import Odometry
from spido_pure_interface.msg import cmd_drive
from std_msgs.msg import Float64, String, Int32
from sensor_msgs.msg import Imu, NavSatFix
from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
from scipy.signal import *
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
import subprocess
import rospy
import numpy.linalg as alg
import pylab as pl
from pylab import *
from scipy import *
from scipy.linalg import solve_continuous_are
import time
from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection
import shapely.geometry as geom
from scipy import spatial
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

#from functions import *



from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Point
import tf

from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round

from matplotlib import pyplot
import numpy
from math import *

#from Parameters import *
import numpy as np


import sys

# == Paramètres du modèle ======================================================

ww = float('nan')


kkk=pi/180
Rt=6371000;
# latitude_init=48.80564077; //sur le terrain réel
# longitude_init=2.07643527;
latitude_init=39.5080331#39.5080322117 #sur gazebo
longitude_init=-0.4619816#-0.46198057533;
vitesse=10
Cf      = 22000   #rigidité de dérive du train avant
Cr      = 22000    # rigidité de dérive du train arrière
masse       = 880
moment      = 86.7
a       = 0.85
b       = 0.85
d       = 0.5

Xp=0
Yp=0
Psip=0
Psi=0
X=0
Y=0

################  IMU  Topic   #################################################
def ImuCallback_F(imu_data):
    global Qx, Qy, Qz, Qw, Xp,Yp, Psip,Psi,ay_F
    Qx=imu_data.orientation.x
    Qy=imu_data.orientation.y
    Qz=imu_data.orientation.z
    Qw=imu_data.orientation.w
#    Xp=odom_message.twist.twist.linear.x
#    Yp=odom_message.twist.twist.linear.y
    ay_F=imu_data.linear_acceleration.y
    Psip=imu_data.angular_velocity.z
    Psi=math.atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz))
 
##################  GPS  Data   ################################################
def retour_gps(msg):
    global longitude,latitude, X ,Y
    longitude= msg.longitude;
    latitude= msg.latitude;
    X=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init)
    Y=Rt*(latitude - latitude_init)*kkk

#############  adaptative contoller Function ############################################
def deltaFunction_adaptative(y, theta_tilde, c,betaF,betaR):
    Kd=0.6
    Kp=Kd**2/4
    axles_dist=1.5
    alpha= 1-c*y
    B = -Kd*alpha*tan(theta_tilde)
    C = -Kp*y
    D = c*alpha*pow(tan(theta_tilde), 2)
    A=B+C+D
    E = c*cos(theta_tilde)/alpha
    F = A*pow(cos(theta_tilde),3)/pow(alpha,2)
    G = (axles_dist/cos(betaR))*(E+F)+tan(betaR)
    deltaF=atan(G)-betaF
    return deltaF

#############  Predictive contoller Function ############################################
def deltaFunction_Predictive(y, theta_tilde, c,betaF,betaR):
    Kd=0.6
    Kp=Kd**2/4
    axles_dist=1.5
    alpha= 1-c*y
    
    B = -Kd*alpha*tan(theta_tilde)
    C = -Kp*y
    D = c*alpha*pow(tan(theta_tilde), 2)
    A=B+C+D
    u=(axles_dist/cos(betaR))*c*(cos(theta_tilde)/alpha)
    v=(axles_dist/cos(betaR))*A*(pow(cos(theta_tilde),3)/pow(alpha,2))+tan(betaR)
    delta_Traj=atan(u)
    delta_deviation=atan(u/(1+u*v+u**2))-betaF
    deltaF=delta_Traj+delta_deviation
    return deltaF  
    
################################################################################
def SSA_ObserverNL(y,theta_tilde,c,v,Theta,dt,dtheta_ref) :
    L1=0.5*cos(theta_tilde)**2+sin(theta_tilde)**2
    L2=-0.5*cos(theta_tilde)*sin(theta_tilde)
    L3=-0.5*cos(theta_tilde)*sin(theta_tilde)
    L4=0.5*sin(theta_tilde)**2+cos(theta_tilde)**2
    L=np.array([[L1,L2],[L3,L4]])  #Observer gain matrix 
    ##########
    p1=-sin(theta_tilde)*((1/c)-y)
    p2=cos(theta_tilde)*((1/c)-y)
    p=np.array([[p1],[p2]])  #p matrix
    ##########
    B1=-(v*cos(theta_tilde))/(2*((1/c)-y))
    B2=-(v*sin(theta_tilde))/(2*((1/c)-y))
    B3=v*sin(theta_tilde)
    B4=-v*cos(theta_tilde)
    B=np.array([[B1,B2],[B3,B4]])  #B matrix
    ##########
    f1=(1/2)*dtheta_ref
    f2=0
    f=np.array([[f1],[f2]])
    ##########
    Theta=Theta+dt*(-L.dot(Theta)-L.dot(np.linalg.inv(B).dot(f)-p))
    omega_e=Theta+p
    beta_f=acos(omega_e[0,0])
    beta_r=asin(-omega_e[0,1])
    print beta_f
    return beta_f,beta_r,Theta

  
##########   Main function #######################################################
def talker():
    cmd_publisher= rospy.Publisher('cmd_drive', cmd_drive,queue_size=10)
    data_pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.Subscriber("/IMU_F", Imu, ImuCallback_F)
    #rospy.Subscriber("/IMU_R", Imu, ImuCallback_R)
    #rospy.Subscriber("/odom", Odometry, OdomCallback)
    rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
    rospy.init_node('SSA', anonymous=True)
    r = rospy.Rate(100) # 100hz
    simul_time = rospy.get_param('~simulation_time', '10')

    
    
    # Trajectoire de reference     
    ref=np.loadtxt('/home/summit/Spidoo_ws/src/LMPC/spido_lmpc/scripts/matS.txt')
    ref[:,1]=ref[:,1]-ref[0,1]
    ref[:,2]=ref[:,2]-ref[0,2]
    dpsi=Psi-ref[0,3]
    ref[:,3]=ref[:,3]+dpsi
    
    RR=np.array([[cos(dpsi),-sin(dpsi),X],[sin(dpsi) ,cos(dpsi) ,Y],[0 ,0 ,1]])
    FF=np.array([np.transpose(ref[:,1]),np.transpose(ref[:,2]),np.ones((1,len(ref)))])
    A=RR.dot(FF)
    ref[:,1:2]=np.transpose(A[0])
    ref[:,2:3]=np.transpose(A[1])
    
    
    line = geom.LineString(ref[:,1:3])
    
    Theta=0
    
    vitesse=4
    
    
    t0=  rospy.get_time()
    cmd=cmd_drive()
    cmd.linear_speed=vitesse
    
    while ((rospy.get_time()-t0<=simul_time)):
        #subprocess.check_call("rosservice call /gazebo/pause_physics", shell=True)
        point = geom.Point(X,Y)
        nearest_pt=line.interpolate(line.project(point))
        distance,index = spatial.KDTree(ref[:,1:3]).query(nearest_pt)
        xref=ref[index,1]
        yref=ref[index,2]
        Psiref=ref[index,3]
        c=ref[index,7]
        vyref=0
        Psipref=0
        y=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)   ## Lateral error
        theta_tilde=(Psi-Psiref)       ## angular error 
        
       
        
        ###  Side-slip angles ###########
        
        betaF=0
        betaR=0
        
        dt=0.1
        
        if c!=0 :
            [kk,yy,Theta]=SSA_ObserverNL(y,theta_tilde,c,vitesse,Theta,dt,Psipref)
        ###   Adaptative controller 
        
        u=deltaFunction_adaptative( y, theta_tilde, c,betaF,betaR)
                

        
        
        
        cmd.steering_angle_front=u#[0,0]
        cmd.steering_angle_rear=0#u[1,0]
        cmd_publisher.publish(cmd)
        
 
        
        posture = np.array([X,Y,Psi,y,theta_tilde,Psip,u,xref,yref,Psiref], dtype=np.float32)
        data_pub.publish(posture)

        r.sleep()

    cmd.steering_angle_front=0#u[0,0]
    cmd.steering_angle_rear=0#u[0,0]
    cmd.linear_speed=0
    cmd_publisher.publish(cmd)



	
	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

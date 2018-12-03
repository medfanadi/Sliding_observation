#!/usr/bin/env python2
# -*- coding: utf-8 -*-
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
from gazebo_msgs.msg import ModelStates
from contact_republisher.msg import contacts_msg
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
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial


import subprocess
import rospy
import numpy.linalg as alg
#import matpbandlotlib.pyplot as plt
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
from std_msgs.msg import String

from functions import*
from cardraw import draw_car



gpsx = 0.0
gpsy = 0.0
kkk=pi/180
Rt=6371000
latitude_init=39.5080331
longitude_init=-0.4619816

ey=0
epsi=0
Psip=0
Psi=0
X=0
Y=0
vy=0
bf=0
br=0
Xref=0
Yref=0
Psiref=0
vpsih=0
eyyy=0
epsih=0
Cf=0
Cr=0
betaf=0
betar=0
Fyf_obs=0
Fyr_obs=0
phi_r=0
theta_r=0



################################################################################
def Callback(message):
    global X,Y,Psi,ey,epsi,vy,Psip,bf,br,Xref,Yref,Psiref,vpsih,eyy,epsih,Cf,Cr,betaf,betar,Fyf_obs,Fyr_obs,phi_r,theta_r
    X=message.data[0]
    Y=message.data[1]
    Psi=message.data[2]
    ey=message.data[3]
    epsi=message.data[4]
    vy=message.data[5]
    Psip=message.data[6]
    bf=message.data[7]
    br=message.data[8]
    Xref=message.data[9]
    Yref=message.data[10]
    Psiref=message.data[11]
    vpsih=message.data[12]
    eyy=message.data[13]
    epsih=message.data[14]
    Cf=message.data[15]
    Cr=message.data[16]
    betaf=message.data[17]
    betar=message.data[18]
    Fyf_obs=message.data[19]
    Fyr_obs=message.data[20]
    phi_r=message.data[21]
    theta_r=message.data[22]


################################################################################
def retour_gps (msg):
    global gpsx
    global gpsy
    global longitude
    global latitude
    longitude= msg.longitude;
    latitude= msg.latitude;
    gpsx=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init)
    gpsy=Rt*(latitude - latitude_init)*kkk

################################################################################
################################################################################
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
################################################################################
def traceStif():
    global gpsx,Cf,Cr,Psi,bf,br,theta,alpha
    global gpsy
    real_trajX = []
    real_trajY = []
    real_Psi = []
    Cff=[]
    Crr=[]
    time=[]
    eyyy=[]
    betaff=[]
    betarr=[]
    Fyf=[]
    Fyr=[]
    Theta=[]
    Alpha=[]
    
    lf=0.7
    lr=0.7
    tr=0.5
    rw=0.2
    dx=0.2
    dy=0.2
    zoom=4
    
    rospy.init_node('trace_gsp')
    
    rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
    
    rospy.Subscriber('floats', numpy_msg(Floats), Callback)
    
    simul_time = rospy.get_param('~simulation_time', '10')
    
    r = rospy.Rate(200) # 10hz
    
    mat=np.loadtxt('/home/summit/Spidoo_ws/src/stiffness_observer/scripts/mat.txt')
    
    mat[:,1]=mat[:,1]-mat[0,1]
    mat[:,2]=mat[:,2]-mat[0,2]
    Psi=1.5
    dpsi=Psi-mat[0,3]
    
    mat[:,3]=mat[:,3]+dpsi
    RR=np.array([[cos(dpsi),-sin(dpsi),gpsx],[sin(dpsi) ,cos(dpsi) ,gpsy],[0 ,0 ,1]])
    FF=np.array([np.transpose(mat[:,1]),np.transpose(mat[:,2]),np.ones((1,len(mat)))])
    
    A=RR.dot(FF)
    mat[:,1:2]=np.transpose(A[0])
    mat[:,2:3]=np.transpose(A[1])
    t0=  rospy.get_time()
    while ((rospy.get_time()-t0<=simul_time)) :
        t=rospy.get_time()-t0-2
        real_trajX.append(gpsx)
        real_trajY.append(gpsy)
        real_Psi.append(Psi)
        Cff.append(Cf)
        Crr.append(Cr)
        time.append(t)
        eyyy.append(ey)
        betaff.append((betaf*(180/pi))/1.2)
        betarr.append(betar*(180/pi))
        Fyf.append(Fyf_obs)
        Fyr.append(Fyr_obs)
        Theta.append(phi_r*(180/pi))
        Alpha.append(theta_r*(180/pi))
        
        plt.figure(1)
        plt.plot(mat[:,1],mat[:,2], 'g', linewidth=1)
        plt.plot(real_trajX, real_trajY, 'k--', linewidth=5)
        plt.xlim(-40, 45)     # set the xlim to xmin, xmax
        plt.ylim(-15,85)     # set the xlim to xmin, xmax
        plt.axis('equal')
        plt.yticks( color='k', size=22)
        plt.xticks( color='k', size=22)
        plt.xlabel('x [m]', fontsize=30)
        plt.ylabel('y [m]', fontsize=30)
        if (t%3<0.19):
            x=np.array([gpsx,gpsy,Psi,bf,br])
            draw_car(x,lf,lr,tr,dx,dy,rw,zoom)
        plt.grid(True)
        plt.legend(["Reference path"],loc='upper right',fontsize=35)

        figure(2)
        plt.plot(time,Theta, 'b', linewidth=1)
        plt.plot(time,Alpha, 'r', linewidth=1)
        plt.xlim(0, 25)     # set the xlim to xmin, xmax
        plt.yticks( color='k', size=22)
        plt.xticks( color='k', size=22)
        plt.xlabel('Time [s]', fontsize=30)
        plt.ylabel('Angles [degree]', fontsize=30)
        plt.grid(True)
        plt.legend(["Bank Angle","Slope Angle"],loc='upper right',fontsize=35)
        
#        figure(2)
#        plt.subplot(222)
#        plt.plot(time,Fyf, 'b', linewidth=1)
#        plt.plot(time, Fyr, 'r', linewidth=3)
#        plt.xlim(0.5, 22)     # set the xlim to xmin, xmax
#        plt.axis('equal')
#        plt.yticks( color='k', size=13)
#        plt.xticks( color='k', size=13)
#        plt.ylabel('Lateral forces [$N.rad^{-1}$]', fontsize=15)
#        plt.xlabel('Time [s]', fontsize=15)
#        plt.grid(True)
#        plt.legend(["Front lateral force"," Rear lateral force"],loc='center left',fontsize=20)
#        
#        plt.subplot(223)
#        plt.plot(time,eyyy, 'k', linewidth=3)
#        #plt.plot(time,Crr, 'b', linewidth=3)
#        #plt.legend(["$\\hat{C}_f$"," $\\hat{C}_r$"],loc='upper right',fontsize=20)
#        plt.xlim(0.5, 22)     # set the xlim to xmin, xmax
#        plt.yticks( color='k', size=13)
#        plt.xticks( color='k', size=13)
#        plt.xlabel('Time [s]', fontsize=15)
#        plt.ylabel('Lateral error [$m$]', fontsize=15)
#        plt.grid(True)
        
#        plt.subplot(211)
#        plt.plot(time,Cff, 'r', linewidth=3)
#        plt.plot(time,Crr, 'b', linewidth=3)
#        plt.legend(["$\\hat{C}_f$","$\\hat{C}_r$"],loc='upper right',fontsize=28)
#        plt.xlim(0, 16)    # set the xlim to xmin, xmax
#        plt.ylim(0, 55000) 
#        plt.yticks( color='k', size=16)
#        plt.xticks( color='k', size=16)
#        plt.xlabel('Time [s]', fontsize=25)
#        plt.ylabel('Cornering stiffness [$N.rad^{-1}$]', fontsize=21)
#        plt.grid(True)
#        
#        plt.subplot(212)
#        plt.plot(time,betaff, 'b', linewidth=3)
#        plt.plot(time,betarr, 'b--', linewidth=3)
#        plt.legend(["$\\beta_f$","$\\beta_r$"],loc='upper right',fontsize=28)
#        plt.xlim(0, 16)     # set the xlim to xmin, xmax
#        plt.ylim(-7, 7) 
#        plt.yticks( color='k', size=16)
#        plt.xticks( color='k', size=16)
#        plt.xlabel('Time [s]', fontsize=25)
#        plt.ylabel('Slip angles [$deg$]', fontsize=25)
#        plt.grid(True)
        plt.draw()
        plt.pause(1/150)
        r.sleep()
################################################################################
if __name__ == '__main__':
    try:
        traceStif()
    except rospy.ROSInterruptException: pass

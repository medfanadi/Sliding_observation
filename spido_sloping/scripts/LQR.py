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
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
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
from std_msgs.msg import String

from functions import*


# == Paramètres du modèle ======================================================
kkk=pi/180
Rt=6371000;
# latitude_init=48.80564077; //sur le terrain réel
# longitude_init=2.07643527;
latitude_init=39.5080331#39.5080322117 #sur gazebo
longitude_init=-0.4619816#-0.46198057533;
vit=5
#Cf      = 15000   #rigidité de dérive du train avant
#Cr      = 15000    # rigidité de dérive du train arrière
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

################################################################################
def ImuCallback_F(imu_data):
    global Qx, Qy, Qz, Qw, Xp,Yp, Psip,ay,ax,az,wx,wy,wz,Psi,theta,phi
    Qx=imu_data.orientation.x
    Qy=imu_data.orientation.y
    Qz=imu_data.orientation.z
    Qw=imu_data.orientation.w
    ax=imu_data.linear_acceleration.x
    ay=imu_data.linear_acceleration.y
    az=imu_data.linear_acceleration.z
    wx=imu_data.angular_velocity.x
    wy=imu_data.angular_velocity.y
    wz=imu_data.angular_velocity.z
    Psip=imu_data.angular_velocity.z
    Psi=math.atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz))
    phi=math.atan2(2.0 * (Qw * Qx + Qy * Qz),1.0 - 2.0 * (Qx * Qx + Qy * Qy))

    sinp = +2.0 * (Qw * Qy - Qz * Qx)
    if (fabs(sinp) >= 1):
        theta= math.copysign(pi / 2, sinp)
    else:
        theta= math.asin(sinp)

    return ax,ay,az,wx,wy,wz,Psi,theta,phi
 
 
################################################################################

def callback(data):
    message = pygazebo.msg.contacts_pb2.Contacts.FromString(data)
    print(message)
################################################################################
def Gazebo_model(model_gaz):
    global Vx_Gaz,Vy_Gaz,Vpsi_Gaz
    Vx_Gaz=model_gaz.twist
#    Vx_Gaz=model_gaz.twist.linear.y
#    Vpsi_Gaz=model_gaz.twist.angular.x

################################################################################
def contactss(forcee):
    global Cont_Forces
    Cont_Forces=forcee.contacts
#    Vx_Gaz=model_gaz.twist.linear.y
#    Vpsi_Gaz=model_gaz.twist.angular.x
    
################################################################################
 
def retour_gps (msg):
	global longitude,latitude, X ,Y
	longitude= msg.longitude;
	latitude= msg.latitude;
	X=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init)
	Y=Rt*(latitude - latitude_init)*kkk

################################################################################
def talker():
    cmd_publisher= rospy.Publisher('cmd_drive', cmd_drive,queue_size=10)
    data_pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    gazebo_pub = rospy.Publisher('gazeboo', String, queue_size=10)
    rospy.Subscriber("/IMU_F", Imu, ImuCallback_F)
    rospy.Subscriber("/GPS/fix",NavSatFix,retour_gps)
    rospy.Subscriber("/gazebo/model_states",ModelStates,Gazebo_model)
#    rospy.Subscriber("/forces",contacts_msg,contactss)
    rospy.init_node('LQR', anonymous=True)
    r = rospy.Rate(200) # 10hz
    simul_time = rospy.get_param('~simulation_time', '10')
    
    
    
    C=np.array([   [0, 1,0, 0],
                        [0,0, 1, 0],
                        [0,0,0, 1]])
                        
    R=np.array([   [10**8,0],
                        [0,10**8]])

    Q=np.array([   [10**7, 0,0, 0],
                        [0,10**7, 0, 0],
                        [0,0,10**7, 0],
                        [0,0,0,10**7]])


#    t00=  rospy.get_time()
#    while ((rospy.get_time()-t00<0.1)) :
#        vy=0
#        eyy=0
#        epsih=0
#        vpsih=0
#        cmd=cmd_drive()
#        cmd.linear_speed=vit
#        cmd.steering_angle_front=0
#        cmd.steering_angle_rear=0
#        cmd_publisher.publish(cmd)
#        r.sleep()

# == Observer  ======================================================


    dt=1/1000  #periode d'échantionnage
    
    Tsamp=0.12
    
    Gammax=dt*eye(4,4)
    
    Gammaalpha=dt*0.0000001*eye(4,4)
    Gammabeta=dt**2*eye(3,3)*0.0000001
    
    mat=np.loadtxt('/home/summit/Spidoo_ws/src/spido_sloping/scripts/mat.txt')
    
    mat[:,1]=mat[:,1]-mat[0,1]
    mat[:,2]=mat[:,2]-mat[0,2]
    dpsi=Psi-mat[0,3]
    
    mat[:,3]=mat[:,3]+dpsi
    RR=np.array([[cos(dpsi),-sin(dpsi),X],[sin(dpsi) ,cos(dpsi) ,Y],[0 ,0 ,1]])
    FF=np.array([np.transpose(mat[:,1]),np.transpose(mat[:,2]),np.ones((1,len(mat)))])
    
    A=RR.dot(FF)
    mat[:,1:2]=np.transpose(A[0])
    mat[:,2:3]=np.transpose(A[1])
    
    Psiref=mat[0,3]
    Psipref=0
    xref=mat[0,1]
    yref=mat[0,2]
    ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
    epsi=(Psi-Psiref)
    xhat=np.array([[0],[0],[0],[0]])
    
    
    
    delta=np.array([[0],[0]])
    Gamma=np.array([[40000],[40000]])
    
    Td=1/200
    g=9.81
    
    
    angle_mesu=Slopes_Mesur(ax,ay,az,0,vit,0)
    

    x_slope=np.array([[0],[0]])
    Gamma_slop=dt*np.eye(2,2)
    
    

    
    line = geom.LineString(mat[:,1:3])
    t0=  rospy.get_time()
    
    cmd=cmd_drive()
    cmd.linear_speed=vit
    
    cmd.steering_angle_front=0
    cmd.steering_angle_rear=0
    cmd_publisher.publish(cmd)
    
#    file2 = open("/home/summit/Spidoo_ws/src/spido_sloping/scripts/Files/Gazebo_model_V8.txt","w")
#    file4 = open("/home/summit/Spidoo_ws/src/spido_sloping/scripts/Files/Traj_V8.txt","w")
    
    
    while ((rospy.get_time()-t0<=simul_time)) :
        
         point = geom.Point(X,Y)
         nearest_pt=line.interpolate(line.project(point))
         distance,index = spatial.KDTree(mat[:,1:3]).query(nearest_pt)
         xref=mat[index,1]
         yref=mat[index,2]
         Psiref=mat[index,3]
         k=mat[index,7]
         vyref=0#mat[index,5]
         Psipref=0#vit*k#mat[index,6]
         ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)
         epsi=(Psi-Psiref)


         vy=xhat[0,0]
         eyy=xhat[2,0]
         epsih=xhat[3,0]
         
         vpsih=xhat[1,0]
         x=np.array([[vy],[Psip],[ey],[epsi]])
         
         yd=np.array([[vyref],[Psipref],[0]])
         
         y=C.dot(x)


         
         theta_r=0
         phi_r=0
         
         
         
         ##  Non Linear Observer 
         deltaf=delta[0,0]
         deltar=delta[1,0]
         
         Cff,Crr,Gamma=SlipModel2(vit, vy,vpsih,deltaf,deltar,k,Gamma,Tsamp)  #rigidité de dérive du train avant et arrière
         
         print('Cf =  ',Cff)
         print('Cr =  ',Crr)
         if (abs(k)>30):
             A=get_model_A_matrix(k,15000,15000,vit) 
             B=get_model_B_matrix(15000,15000)
             Cff,Crr=15000,15000
         else:
             A=get_model_A_matrix(k,Cff,Crr,vit) 
             B=get_model_B_matrix(Cff,Crr)
             
       
        ## SideSlip angles and tire forces
         
         betaf,betar= SlipAngles(vit,vy,vpsih,deltaf,deltar)
         
         Fyf_obs,Fyr_obs=ForceLater_observed(betaf,betar,Cff,Crr,k) 
         
         
         ## Slop and Bank angles
         
         # x_slope,Gamma_slop,uk,zk,Gammaalpha_slop,Gammabeta_slop,Ak,Ck=Model_slope(ax,ay,az,vit,x_slope,wx,wy,wz,Psi,phi,theta)
         # xxw=Derivative(wx,wy,wz,Psi,phi,theta)
         # theta_r=xxw[0,0]
         # phi_r=xxw[1,0]
         # x_slope=np.array([[vy],[vpsih],[theta_r],[phi_r]])
         
         # theta_r,phi_r=Slopes(ax,ay,az,vy,vit,vpsih)

         # x_mesure=Slopes_Mesur(ax,ay,az,vy,vit,vpsih)
         # xdot_r=Derivative(wx,wy,wz,Psi,phi,theta)


         # x_slope=Bank_slope(x_slope,Td,wx,wy,wz,Psi,phi,theta,ax,ay,az,vy,vit,vpsih)
         # phi_r=x_slope[2,0]
         # theta_r=x_slope[3,0]
         # print('xslope = ',x_slope)
         # print('slope=',phi_r*(180/pi))
         # print('bank=',theta_r*(180/pi))


         ## Steady state vectors
         
         Vyss=(k*vit*(A[(1,1)]*(B[(0,0)]-B[(0,1)])-A[(0,1)]*(B[(1,0)]-B[(1,1)]))-g*cos(phi_r)*sin(theta_r)*(B[(1,0)]-B[(1,1)]))/(A[(0,0)]*(B[(1,0)]-B[(1,1)])-A[(1,0)]*(B[(0,0)]-B[(0,1)]))
         Vpsiss=vit*k
         eyss=0;
         epsiss=-Vyss/vit;
         
         bfss=(k*vit*(A[(0,0)]*A[(1,1)]-A[(1,0)]*A[(0,1)]) + A[(1,0)]*g*cos(phi_r)*sin(theta_r))/(-A[(0,0)]*(B[(1,0)]-B[(1,1)])+A[(1,0)]*(B[(0,0)]-B[(0,1)]))
         brss=-bfss;
         
         xsss=np.array([[Vyss],[Vpsiss],[eyss],[epsiss]])
         usss=np.array([[bfss],[brss]])
         
         
         
         
         ## LQR controller
         P=solve_continuous_are(A, B, Q, R)
         
         G=np.linalg.inv(R).dot(np.transpose(B)).dot(P)
         
         M = pinv(C.dot(pinv(A-B.dot(G))).dot(B))
         
         u=M.dot(yd)-G.dot(x-xsss)+usss
                  
         xhat,Gammax=kalman(xhat,Gammax,dt*B.dot(u),y,Gammaalpha,Gammabeta,eye(4,4)+dt*A,C)
         
         delta=u
#Model_slopee(vy,vpsih,ax,ay,az,vit,x_slope,Gamma_slop,wx,wy,wz,Psi,phi,theta)#

         # Gamma_slop=dt*0.0000001*np.eye(4,4)    
         # uk,zk,Gammaalpha1,Gammabeta1,Ak,C_lin=Model_slope_New(vy,vpsih,ax,ay,az,vit,x_slope,Gamma_slop,wx,wy,wz,Psi,phi,theta,delta)
         x_slope=Luenberger(vy,vpsih,ax,ay,az,vit,x_slope,Gamma_slop,wx,wy,wz,Psi,phi,theta,delta)#Model_slopee(vy,vpsih,ax,ay,az,vit,x_slope,Gamma_slop,wx,wy,wz,Psi,phi,theta)
         theta_r=x_slope[0,0]
         phi_r=x_slope[1,0]

         angle_mesur=Slopes_Mesur(ax,ay,az,vy,vit,vpsih)
         theta_rMesu=angle_mesur[0,0]
         phi_rMesu=angle_mesur[1,0]


         # print('theta_r = ', theta*(180/pi))
         # print('phi_r = ', phi*(180/pi))

         # print('xslope = ', x_slope*(180/pi))


         cmd.steering_angle_front=u[0,0]
         cmd.steering_angle_rear=u[1,0]
         cmd_publisher.publish(cmd)
         
         posture = np.array([X,Y,Psi,ey,epsi,vy,Psip,u[0,0],u[1,0],xref,yref,Psiref,vpsih,eyy,epsih,Cff,Crr,betaf,betar,Fyf_obs,Fyr_obs,phi_r,theta_r], dtype=np.float32)
         data_pub.publish(posture)
         gazebo_pub.publish(Vx_Gaz)
#         gazebo_pub.publish(Cont_Forces)
#         file2.write(str(Vx_Gaz) +'\n')
#         file4.write(' '.join((str(rospy.get_time()-t0), str(X),str(Y),str(Psi)))+'\n')
         
         r.sleep()
    
#    file2.close()
#    file4.close()
    
    cmd.steering_angle_front=0
    cmd.steering_angle_rear=0
    cmd.linear_speed=0
    cmd_publisher.publish(cmd)
 
#     cmd.steering_angle_front=0
#     cmd.steering_angle_rear=0
#     aa=vit
#     while (aa>0):
#         cmd=cmd_drive()
#         aa=aa-0.1
#         if aa<0:
#             aa=0
#        cmd.linear_speed=aa
#        cmd_publisher.publish(cmd)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

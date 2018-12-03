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
a       = 0.85
b       = 0.85
d       = 0.5

Xp=0
Yp=0
Psip=0
Psi=0
X=0
Y=0
LR=b
LF=a
L=LR+LF
Tsamp=0.15 #Sample Time derrivation
Kd=0.75
G=np.array([[-6,-3],[1,-11]])
G1=np.array([[11,0],[0,9]])
VITESSE=6


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

############# No Linear observer function ######################################
def NLObserverBFBr(y,theta_tilde,DeltaF,DeltaR,vitesse,C,Epsi,DCapR,theta_tilde_dot):
    global Tsamp,LR,LF
    if vitesse<=0.5:
        Epsi=np.array([[0],[0]])
        PX=np.array([[0],[0]])
    else: 
        if C==0:
            C=0.000001;
        alphaob=(1/C-y);
        a1=vitesse/(2*alphaob);  
        b1=vitesse;
        pres=np.array([[1],[1]]);
        fx=np.array([[DCapR/2],[0]]);
        PX=pres*np.array([[-sin(DeltaR+theta_tilde)*alphaob],[cos(DeltaR+theta_tilde)*alphaob]])
        B=np.array([[-a1*cos(DeltaR+theta_tilde),-a1*sin(DeltaR+theta_tilde)],[b1*sin(DeltaR+theta_tilde),-b1*cos(DeltaR+theta_tilde)]])
        Cb=pres*np.array([[-cos(DeltaR+theta_tilde)*alphaob,sin(DeltaR+theta_tilde)],[-sin(DeltaR+theta_tilde)*alphaob,-cos(DeltaR+theta_tilde)]])
        CM=Cb.dot(B)
        Epsi= Epsi+ Tsamp*(-CM.dot(np.linalg.pinv(B)).dot(fx) - CM.dot(Epsi-PX))
    We=Epsi+PX
    We1=We[0,0]
    We2=We[1,0]
    if (abs(We1)==0 or abs(We2)==0):
        betaR=0
        betaF=0
    else:
        betaR=atan(-We2/We1)+(pi/2)
        
        if abs(betaR)>1.5*pi/2:
            betaR=betaR-pi;
        
        Beta=atan(tan(DeltaR-betaR)+ LR*theta_tilde_dot/(vitesse*cos(DeltaR-betaR)))
        
        if abs(Beta) > 1.5*pi/2: 
            Beta=Beta-pi
        
        betaF=(atan(tan(Beta)+LF*theta_tilde_dot/(vitesse*cos(DeltaR-betaR)))+ DeltaF)
        
        if abs(betaF) > 1.5*pi/2: 
            betaF=betaF-pi;
            
    return betaF,betaR,Epsi
    
    
############# No Linear observer function ######################################
def NLObserver_Linear_BFBr(y,theta_tilde,DeltaF,DeltaR,vitesse,C,Epsi,DCapR,theta_tilde_dot):
    global Tsamp,LR,LF,L
    if vitesse<=0.5:
        Epsi=np.array([[0],[0]])
        PX=np.array([[0],[0]])
    else: 
        if C==0:
            C=0.000001;
        alphaob=(1/C-y);
        a1=vitesse/(2*alphaob);  
        b1=vitesse;
        pres=np.array([[1],[1]]);
        
        B12=0;
        B11=vitesse*cos(theta_tilde);  
        B22=vitesse/(L*cos(DeltaF)**2);        
        B21= vitesse*C*sin(theta_tilde)/(1-C*y)-vitesse/L;  
        B=np.array([[B11,B12],[B21,B22]])
        
        fFB001=vitesse*sin(theta_tilde)
        fFB002=vitesse*(((tan(DeltaF)/L-C*cos(theta_tilde)/(1-C*y))))
        fx=np.array([[fFB001],[fFB002]])
    
        PX=pres*np.array([[(1/3)*y**3],[y+L*theta_tilde]])
        Cb=pres*np.array([[y**2,0],[1,L]])
        CM=Cb.dot(B)
        print CM
        Epsi= Epsi+ Tsamp*(-CM.dot(np.linalg.pinv(B)).dot(fx) - CM.dot(Epsi+PX))
    We=Epsi+PX
   
    betaF=We[0,0]
    betaR=We[1,0]
            
    return betaF,betaR,Epsi
 

###############  Observer state ################################################

def KinmaticModelOLd(betaF,betaR,y,theta_tilde,Xm,v,DeltaF,C):
    global Tsamp,LR,LF,L
    f1=v*sin(theta_tilde+betaR)
    f2=v*(cos(betaR)*((tan(DeltaF+betaF)-tan(betaR))/L-C*cos(theta_tilde+betaR)/(1-C*y)))
    f=np.array([[f1],[f2]])
    Xm=Xm+Tsamp*f
    y_obs=Xm[0,0]
    theta_tilde_obs=Xm[1,0]
    return  y_obs,theta_tilde_obs,Xm

###############  Kinematic model ################################################

def KinmaticModel(betaF,betaR,y,theta_tilde,v,DeltaF,C):
    global Tsamp,LR,LF,L
    fF1=v*sin(theta_tilde+betaR)
    fF2=v*(cos(betaR)*(tan(DeltaF+betaF)-tan(betaR))/L-C*cos(theta_tilde+betaR)/(1-C*y))
    fF=np.array([[fF1],[fF2]])
#    Xm=Xm+Tsamp*f
#    y_obs=Xm[0,0]
#    theta_tilde_obs=Xm[1,0]
#    return  y_obs,theta_tilde_obs,Xm
    return fF
###############  Linear Kinematic model ################################################

def LinearKinmaticModel(y_obs,theta_tilde_obs,Xm,v,DeltaF,C,betaF,betaR):
    global Tsamp,LR,LF,L
    
    if C==0:
            C=0.0001;
            
    fFB001=v*sin(theta_tilde_obs)
    fFB002=v*(((tan(DeltaF)/L-C*cos(theta_tilde_obs)/(1-C*y_obs))))
    fFB00=np.array([[fFB001],[fFB002]])
        
    Bf11=0;
    Bf21=v*cos(theta_tilde_obs);  
    Bf12=v/(L*cos(DeltaF)**2);        
    Bf22= v*C*sin(theta_tilde_obs)/(1-C*y_obs)-v/L;  
    Bf=np.array([[Bf11,Bf21],[Bf12,Bf22]])
    
    u1=betaF
    u2=betaR
    uf=np.array([[u1],[u2]])
    
    fFobs=fFB00+Bf.dot(uf)
#    print fFB00

    Xm=Xm+Tsamp*fFobs
    
    y_obs=Xm[0,0]
    theta_tilde_obs=Xm[1,0]*pi/180
    if abs(theta_tilde_obs) > 1.5*pi/2: 
        theta_tilde_obs=theta_tilde_obs-pi;
        
    return  y_obs,theta_tilde_obs,Xm,fFB00,Bf



    
############# Linear observer function Lenain #########################################

def LenainObsvBFBr(y,theta_tilde,y_obs,theta_tilde_obs,DeltaF,DeltaR,v,C,theta_tilde_dot,y_dot,fFB00,fF,Bf):
    global Tsamp,LR,LF,L,betaRold,betaFold,G,G1
    if C==0:
            C=0.0001;
#    B11=0;
#    B21=v*cos(theta_tilde_obs);  
#    B12=v/(L*cos(DeltaF)**2);        
#    B22= v*C*sin(theta_tilde_obs)/(1-C*y_obs)-v/L;  
#    B=np.array([[B11,B21],[B12,B22]])
#    G=np.array([[-2.8,0],[0,-0.8]])


#    f1=v*sin(theta_tilde_obs)
#    f2=v*((tan(DeltaF))/L-C*cos(theta_tilde_obs)/(1-C*y_obs))
#    f=np.array([[f1],[f2]])
#    
    
    X_dot=np.array([[y_dot],[theta_tilde_dot]])
    X_stat=np.array([[y],[theta_tilde]])
    X_obs=np.array([[y_obs],[theta_tilde_obs]])
    epsilon=X_obs-X_stat
    
#    We=np.linalg.pinv(B).dot(G.dot(epsilon)-f+X_dot)
    We=G1.dot(np.linalg.inv(Bf).dot(G.dot(epsilon)-fFB00+fF)*pi/180)#+pi/2
    betaF=We[0,0]
    betaR=We[1,0]
    
    
    
#    if abs(betaF) > 1.5*pi/2: 
#        betaF=betaF-pi;
#    if abs(betaR)>1.5*pi/2:
#        betaR=betaR-pi;
#    if abs(betaF) > 1*pi/36: 
#            betaF=1*sign(betaF)/36
#    
#    if abs(betaR) > 1*pi/36: 
#            betaR=1*sign(betaR)/36
            
    betaRold=betaR
    betaFold=betaF
    
    return betaF,betaR
              
       
    
############# Linear observer function #########################################

def LenainObsvBFBrOld(y,theta_tilde,y_obs,theta_tilde_obs,DeltaF,DeltaR,v,C,theta_tilde_dot,y_dot):
    global Tsamp,LR,LF,L
    B11=v*cos(theta_tilde_obs);  
    B21=0;
    B12= v*C*sin(theta_tilde_obs)/(1-C*y_obs)-v/L;  
    B22=v/L*(1+tan(DeltaF)**2);        
    B=np.array([[B11,B21],[B12,B22]])
    G=np.array([[-100,0],[0,-100]])#np.array([[-2.8,0],[0,-0.8]])
    f1=v*sin(theta_tilde_obs)
    f2=v*((tan(DeltaF))/L-C*cos(theta_tilde_obs)/(1-C*y_obs))
    f=np.array([[f1],[f2]])
    X_dot=np.array([[y_dot],[theta_tilde_dot]])
    X_stat=np.array([[y],[theta_tilde]])    
    X_obs=np.array([[y_obs],[theta_tilde_obs]])
    epsilon=X_obs-X_stat
    We=np.linalg.pinv(B).dot(G.dot(epsilon)-f+X_dot)
    betaR=We[0,0]
    betaF=We[1,0]
    return betaF,betaR
              
    
    
############# Linear observer function #########################################

def LinearObsvBFBr(y,theta_tilde,DeltaF,DeltaR,vitesse,C,Epsi,DCapR,theta_tilde_dot):
    global Tsamp,LR,LF,L
    if vitesse<=0.5:
        Epsi=np.array([[0],[0]])
        PX=np.array([[0],[0]])
    else: 
        if C==0:
            C=0.00000001
        alphaob=(1/C-y)
        b1=vitesse;
        pres=1/vitesse         
        fx=np.array([[b1*sin(theta_tilde)],[b1*((tan(DeltaF)/L)-(cos(theta_tilde)/alphaob))]])         
        PX=pres*np.array([[sin(theta_tilde)-y],[sin(theta_tilde)+y]])         
        P11=-1;  
        P21=cos(theta_tilde);
        P12= 1;  
        P22=cos(theta_tilde);        
        CM=pres*np.array([[P11,P21],[P12,P22]])         
        B=np.array([[0,cos(theta_tilde)],[1/(L*(pow(cos(DeltaF),2))),(sin(DeltaR+theta_tilde)/alphaob)-1/L]])      
        Epsi= Epsi+ Tsamp*(-CM.dot(fx) - CM.dot(B).dot(Epsi+PX))
    We=Epsi+PX
    betaR=We[0,0]
    betaF=-We[1,0]             
    return betaF,betaR,Epsi
           


#############  Contoller Function ###############################################
def deltaFunction(y, theta_tilde, c,betaF,betaR):
    global Kd
    Kp=Kd**2/4
    axles_dist=1.7
    alpha= 1-c*y
    theta_tilde1=theta_tilde+betaR
    B = -Kd*alpha*tan(theta_tilde1)
    C = -Kp*y
    D = c*alpha*pow(tan(theta_tilde1), 2)
    A=B+C+D
    E = c*cos(theta_tilde1)/alpha
    F = A*pow(cos(theta_tilde1),3)/pow(alpha,2)
    G = (axles_dist/cos(betaR))*(E+F)+tan(betaR)
    deltaF=atan(G)-betaF
    return deltaF
    
    
################################################################################
def talker():
    global u,VITESSE
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
    
    vitesse=VITESSE
    Epsi=np.array([[0],[0]])
    Xm=np.array([[0],[0]])
    Psiref_old=Psi
    theta_tilde_old=0
    ey_old=0
    DeltaR=0
    DeltaF=0
    betaF=0 
    betaR=0
    y_obs=0
    theta_tilde_obs=0
    
    
    
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
        
        ### Calculate DCapR
        
        DCapR=(Psiref-Psiref_old)/Tsamp
        Psiref_old=Psiref
        
        ### theta_tilde_dot
        ey=-sin(Psiref)*(X-xref)+cos(Psiref)*(Y-yref)   ## Lateral error
        ey_dot=(ey-ey_old)/Tsamp
        ey_old=ey
        theta_tilde=(Psi-Psiref)       ## angular error 
        theta_tilde_dot=(theta_tilde-theta_tilde_old)/Tsamp
        theta_tilde_old=theta_tilde
        

        ###  Side-slip angles Observer 
#        betaF=0 
#        betaR=0
        #betaF,betaR,Epsi=LinearObsvBFBr(ey,theta_tilde,DeltaF,DeltaR,vitesse,c,Epsi,DCapR,theta_tilde_dot)   #  Linear Observer
        
        ###  No Linear Observer   
#        betaF,betaR,Epsi=NLObserverBFBr(ey,theta_tilde,DeltaF,DeltaR,vitesse,c,Epsi,DCapR,theta_tilde_dot)   # NL  Linear Observer
        
         ### For Roland 2017
        
#        fF=KinmaticModel(betaF,betaR,ey,theta_tilde,vitesse,DeltaF,c)
#        y_obs,theta_tilde_obs,Xm,fFB00,B=LinearKinmaticModel(y_obs,theta_tilde_obs,Xm,vitesse,DeltaF,c,betaF,betaR)
#        betaF,betaR=LenainObsvBFBr(ey,theta_tilde,y_obs,theta_tilde_obs,DeltaF,DeltaR,vitesse,c,theta_tilde_dot,ey_dot,fFB00,fF,B)
        
        
        ####  Linear Model EMAD MAHROUS  NLObserver_Linear_BFBr
        betaF,betaR,Epsi=NLObserver_Linear_BFBr(ey,theta_tilde,DeltaF,DeltaR,vitesse,c,Epsi,DCapR,theta_tilde_dot) 
        
##      

#        print('y_obs=',y_obs)
#        print('y=',ey)
#        
        
        if (betaF<-0.087266463):
            betaF=-0.087266463
            
        if (betaR<-0.087266463):
            betaR=-0.087266463
            
        if (betaR>0.087266463):
            betaR=0.087266463
            
        if (betaF>0.087266463):
            betaF=0.087266463
        
            
            
        aa=betaF*(180/pi)
        bb=betaR*(180/pi)
        
        
#        y_obs,theta_tilde_obs,Xm=KinmaticModel(betaF,betaR,ey,-theta_tilde,Xm,vitesse,DeltaF,c)
#        betaF,betaR=LenainObsvBFBr(ey,-theta_tilde,y_obs,theta_tilde_obs,DeltaF,DeltaR,vitesse,c,theta_tilde_dot,ey_dot)
#        
#        print('y_obs=\n',y_obs)
#        print('y=\n',ey)
        print('betaF=',aa)
        print('betaR=',bb)
        
        ###   Adaptative controller 
        u=deltaFunction(ey,theta_tilde,c,betaF,betaR)
        
        
        if abs(u)>20*(pi/180):
            u=20*(pi/180)*sign(u)
            
            
        DeltaF=u
        
        

        
        
        
        cmd.steering_angle_front=u#[0,0]
        cmd.steering_angle_rear=DeltaR#u[1,0]
        cmd_publisher.publish(cmd)
        
 
        
        posture = np.array([X,Y,Psi,ey,theta_tilde,Psip,u,xref,yref,Psiref,betaF,betaR], dtype=np.float32)
        data_pub.publish(posture)
        
        #subprocess.check_call("rosservice call /gazebo/unpause_physics", shell=True)
        #unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        r.sleep()

    cmd.steering_angle_front=0#u[0,0]
    cmd.steering_angle_rear=0#u[0,0]
    cmd.linear_speed=0
    cmd_publisher.publish(cmd)



	
	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

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

######
#k=1/20  # à calculer à chaque pas de calcul ???? .txt
#######"
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



################################################################################
def Callback(message):
    global X,Y,Psi,ey,epsi,vy,Psip,bf,br,Xref,Yref,Psiref,vpsih,eyy,epsih,Cf,Cr,betaf,betar,Fyf_obs,Fyr_obs
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
def recorderr():
    rospy.Subscriber("/forces",contacts_msg,contactss)
    rospy.init_node('record2', anonymous=True)
    qq = rospy.Rate(460) # 10hz
    simul_time = rospy.get_param('~simulation_time', '10')

    file3 = open("/home/summit/Spidoo_ws/src/stiffness_observer/scripts/Gazebo_data/Gazebo_Forces_V8.txt","w")
    

    t0=rospy.get_time()

    while (rospy.get_time()-t0<=simul_time):
        file3.write(str(Cont_Forces) +'\n')
        qq.sleep()
    file3.close()
########
if __name__ == '__main__':
    try:
        #recorderGazebo()
        recorderr()
    except rospy.ROSInterruptException: pass

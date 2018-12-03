#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
import numpy as np

from std_msgs.msg import Float64, String, Int32
#from scipy.signal import place_poles
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round



from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round

from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
from scipy import fftpack

import subprocess
import rospy
import numpy.linalg as alg
import matplotlib.pyplot as plt
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


######
#k=1/20  # à calculer à chaque pas de calcul ???? .txt
#######"
y=0
theta_tilde=0
Psip=0
Psi=0
X=0
Y=0
vy=0
u=0
Xref=0
Yref=0
Psiref=0
betaF=0
betaR=0





################################################################################
def Callback(message) :
    global X,Y,Psi,y,theta_tilde,Psip,u,Xref,Yref,Psiref,betaF,betaR
    X=message.data[0]
    Y=message.data[1]
    Psi=message.data[2]	
    y=message.data[3]
    theta_tilde=message.data[4]
    Psip=message.data[5]	
    u=message.data[6]	
    Xref=message.data[7]	
    Yref=message.data[8]
    Psiref=message.data[9]
    betaF=message.data[10]
    betaR=message.data[11]
   

################################################################################

################################################################################

################################################################################
def recorder():
    
    file = open("/home/summit/Spidoo_ws/src/spido_obskridssa/scripts/Plot/NewLinear/New_Line_V6.txt","w") #Without_contr_4  NLObser_6  LinObser_4

    rospy.Subscriber('floats', numpy_msg(Floats), Callback)

    rospy.init_node('record', anonymous=True)

    r = rospy.Rate(200) # 10hz

    simul_time = rospy.get_param('~simulation_time', '10')

    while(abs(X)<0.001):
		i=0
    t0=rospy.get_time()

    while (rospy.get_time()-t0<=simul_time):
		file.write(' '.join((str(rospy.get_time()-t0), str(X),str(Y), str(Psi),str(y),str(theta_tilde),str(Psip),str(u),str(Xref),str(Yref), str(Psiref),str(betaF),str(betaR))) +'\n')
		r.sleep()
    file.close()
if __name__ == '__main__':
    try:
        recorder()
    except rospy.ROSInterruptException: pass

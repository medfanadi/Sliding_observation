#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
import numpy as np
import numpy.linalg as alg
import matplotlib.pyplot as plt
import pylab as pl
from pylab import *
from scipy.linalg import solve_continuous_are
import time
from nav_msgs.msg import Odometry
from spido_pure_interface.msg import cmd_drive
from std_msgs.msg import Float64, String, Int32
from sensor_msgs.msg import Imu, NavSatFix
from scipy.linalg import sqrtm,expm,norm,block_diag
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
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
import subprocess
import rospy
import numpy.linalg as alg
import pylab as pl
from pylab import *
from scipy import *

import numpy as np


from scipy.linalg import solve_continuous_are
import time

from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc

from matplotlib.collections import PatchCollection
import shapely.geometry as geom
from scipy import spatial


from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats




#################################################################################################################################################


#################################################################################
# Extended Kalman Filter Functions
#################################################################################

def kalman(x0,Gamma0,u,y,Gammaalpha,Gammabeta,A,C):
    S = C.dot(Gamma0) .dot(np.transpose(C)) + Gammabeta
    Kal = Gamma0 .dot(np.transpose(C)) .dot(inv(S) )
    ytilde = y - C .dot(x0 )
    Gup = (eye(len(x0))-Kal .dot(C) ).dot(Gamma0)
    xup = x0 + Kal.dot(ytilde)
    Gamma1 = A .dot(Gup) .dot(np.transpose(A)) + Gammaalpha
    x1 = A .dot(xup) + u
    return(x1,Gamma1)


#################################################################################
# Extended kinematic model
#################################################################################
def B(C,v,betaR,y,teta,delatR) :

    a11=-v*cos(teta+delatR)/(2*((1/C)-y))
    a12=-v*sin(teta+delatR)/(2*((1/C)-y))
    a21=v*sin(teta+delatR)
    a22=-v*cos(teta+delatR)


    return np.array([   [a11,a12],
                        [a21,a22]])

################################################################################
def f(tetaRp) :

   b11=(1/2)*tetaRp
   b21=0


   return np.array([[b11],
                        [b21]])


#################################################################################
# Observer matrix
#################################################################################
def c(C,v,betaR,y,teta,delatR) :

    c11=-cos(teta+delatR)*(((1/C)-y))
    c12=-sin(teta+delatR)
    c21=-sin(teta+delatR)*(((1/C)-y))
    c22=-cos(teta+delatR)


    return np.array([   [c11,c12],
                        [c21,c22]])

################################################################################
def gain(B,c) :

    L=B.dot(c)
    return L
################################################################################

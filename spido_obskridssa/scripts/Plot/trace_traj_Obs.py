#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from math import *
import numpy as np
import matplotlib.pyplot as plt
from cardraw import draw_car
import shapely.geometry as geom
from scipy import interpolate
from scipy import signal
from scipy import fftpack
from pylab import *

import shapely.geometry as geom
from scipy import spatial
from ButterFilter import butter_lowpass_filter



kk=180/pi

######   Upload recorded files   #################################################################

NLobs=np.loadtxt('/home/summit/Spidoo_ws/src/spido_obskridssa/scripts/Plot/NLObser_4.txt') #NLobser_5
LinObs=np.loadtxt('/home/summit/Spidoo_ws/src/spido_obskridssa/scripts/Plot/LinObser_4.txt')
WithoutObs=np.loadtxt('/home/summit/Spidoo_ws/src/spido_obskridssa/scripts/Plot/Without_contr_4.txt')



XNL = butter_lowpass_filter(NLobs[:,1], 0.04, 10, 8)
YNL = butter_lowpass_filter(NLobs[:,2], 0.04, 10, 8)

Xlin= butter_lowpass_filter(LinObs[:,1], 0.04, 10, 8)
Ylin= butter_lowpass_filter(LinObs[:,2], 0.04, 10, 8)

Xwo= butter_lowpass_filter(WithoutObs[:,1], 0.04, 10, 8)
Ywo= butter_lowpass_filter(WithoutObs[:,2], 0.04, 10, 8)

Xref= butter_lowpass_filter(NLobs[:,8], 0.04, 10, 8)
Yref= butter_lowpass_filter(NLobs[:,9], 0.04, 10, 8)

Xref=Xref[:]+XNL [0]-Xref[0]
Yref=Yref[:]+YNL [0]-Yref[0]

######   Trajectory plot   #################################################################

plt.plot(Xref,Yref, 'g', linewidth=3.20)
plt.plot(Xwo,Ywo, 'k--', linewidth=3.20)
plt.plot(Xlin,Ylin, 'r--', linewidth=3.0)
plt.plot(XNL ,YNL , 'b--', linewidth=3.20)

plt.axis('equal')
#plt.axis([-50, 10, -10, 70])
plt.yticks( color='k', size=48)
plt.xticks( color='k', size=48)
plt.xlabel('x [m]', fontsize=42)
plt.ylabel('y [m]', fontsize=42)

########################  Drawing robot with steering ###################################

lf=0.7
lr=0.7
tr=0.5
rw=0.2
dx=0.2
dy=0.2

temps1=NLobs[:,0]
temps2=NLobs[:,0]
zoom=4


ix = interpolate.interp1d(NLobs[:,0],NLobs[:,1])
iy = interpolate.interp1d(NLobs[:,0],NLobs[:,2])
iyaw = interpolate.interp1d(NLobs[:,0],NLobs[:,3])
ibf = interpolate.interp1d(NLobs[:,0],NLobs[:,7])
ibr = interpolate.interp1d(NLobs[:,0],np.zeros(len(temps1)))
n=6
i=0
while (i<n):
	tempsp=temps1[int(i*(len(temps1)-1)/(n-1))]
	tempss=temps2[int(i*(len(temps2)-1)/(n-1))]
	x=np.array([ix(tempsp),iy(tempsp),iyaw(tempsp),ibf(tempss),ibr(tempss)])
	draw_car(x,lf,lr,tr,dx,dy,rw,zoom)
	i=i+1
#########################################################################################

#########################################################################################
#plt.xlim((-10, 95))   # set the xlim to xmin, xmax
#
#plt.ylim((-20, 100))   # set the xlim to xmin, xmax

plt.grid(True)
plt.legend(["Reference path","Path without observer at $V_x=4 m.s^{-1}  $","Path for linear observer at $V_x=4 m.s^{-1}$","Path for non-linear observer at $V_x=4 m.s^{-1}  $"],fontsize=30)
plt.show()

############################  Curvilinear abscissa  ##############################

s4=np.zeros(len(NLobs[:,1]))
s8=np.zeros(len(LinObs[:,1]))
s6=np.zeros(len(WithoutObs[:,1]))


i=0
while (i<len(NLobs[:,1])-1):
	s4[i+1]=s4[i]+hypot(NLobs[i+1,1]-NLobs[i,1],NLobs[i+1,2]-NLobs[i,2])
	i=i+1
i=0
while (i<len(LinObs[:,1])-1):
        s8[i+1]=s8[i]+hypot(LinObs[i+1,1]-LinObs[i,1],LinObs[i+1,2]-LinObs[i,2])
	i=i+1
i=0
while (i<len(WithoutObs[:,1])-1):
        s6[i+1]=s6[i]+hypot(WithoutObs[i+1,1]-WithoutObs[i,1],WithoutObs[i+1,2]-WithoutObs[i,2])
	i=i+1
 
 
########################### Error trace #########################################
YNL = butter_lowpass_filter(NLobs[:,4], 0.03, 200, 6)
Ylin= butter_lowpass_filter(LinObs[:,4], 0.03, 200, 6)
Ywo= butter_lowpass_filter(WithoutObs[:,4], 0.03, 200, 6)

plt.plot(s6[:],WithoutObs[:,4], 'k', linewidth=3.2)
plt.plot(s8[:],LinObs[:,4], 'r', linewidth=3)
plt.plot(s4[:],NLobs[:,4], 'b', linewidth=3.2)

plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlabel('Curvilinear abscissa  [m]', fontsize=39)
plt.ylabel('Lateral error [m]', fontsize=39)
plt.grid(True)
plt.legend(["y without observer at $V_x=4 m.s^{-1}$ ","y linear observer at $V_x=4 m.s^{-1}$","y non-linear  observer at $V_x=4 m.s^{-1}$"],loc='upper left',fontsize=30)
#plt.title('L')
plt.xlim((0, 115))   # set the xlim to xmin, xmax
plt.ylim(-0.75, 0.75)     # set the xlim to xmin, xmax
plt.show()
######
#YNL = butter_lowpass_filter(NLobs[:,5], 0.03, 200, 6)
#Ylin= butter_lowpass_filter(LinObs[:,5], 0.03, 200, 6)
#Ywo= butter_lowpass_filter(WithoutObs[:,5], 0.03, 200, 6)
#
#kk=180/pi
#plt.plot(s4[:],YNL [:]*kk, 'b', linewidth=3)
#plt.plot(s8[:],Ylin[:]*kk, 'r', linewidth=3)
#plt.plot(s6[:],Ywo[:]*kk, 'k', linewidth=3)
#
#plt.xlabel('Curvilinear abscissa  [m]', fontsize=25)
#plt.ylabel('$\\tilde \\theta$ [deg]', fontsize=25)
#plt.grid(True)
#plt.legend(["$\\tilde \\theta$ at $V_x=4 m.s^{-1} $ with non-linear  observer","$\\tilde \\theta$ at $V_x=4 m.s^{-1}$ without observer","$\\tilde \\theta$ at $V_x=4 m.s^{-1}$ with non-linear  observer"])
#
##plt.xlim((15, 115))   # set the xlim to xmin, xmax
#
#plt.show()

#######   SSA  ###################################################################

SSAF= butter_lowpass_filter(NLobs[:,11], 0.045, 200, 6)
SSAR= butter_lowpass_filter(NLobs[:,12], 0.04, 200, 6)
SSAFRL= butter_lowpass_filter(LinObs[:,11], 0.045, 200, 6)
SSARRL= butter_lowpass_filter(LinObs[:,12], 0.04, 200, 6)

plt.plot(s8[:]-3.7*np.ones((len(s8))),SSAFRL*kk*2, 'r', linewidth=3.2)
plt.plot(s4[:],SSAF*kk, 'b', linewidth=3.2)
plt.plot(s8[:]-2*np.ones((len(s8))),SSARRL*kk*2, 'r--', linewidth=3.2)
plt.plot(s4[:],SSAR*kk, 'b--', linewidth=3.2)


plt.xlabel('Curvilinear abscissa  [m]', fontsize=35)
plt.ylabel('Slip angles [deg]', fontsize=35)
plt.grid(True)
plt.legend(["$\\beta_f$ at $V=4 m.s^{-1} $ Linear observer","$\\beta_f$ at $V=4 m.s^{-1} $ non-linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ non-linear  observer"],loc='upper left',fontsize=30)
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlim((0, 112))   # set the xlim to xmin, xmax

plt.show()


#######   SSA  ###################################################################

#SSAF= butter_lowpass_filter(NLobs[:,11], 0.045, 200, 6)
#SSAR= butter_lowpass_filter(NLobs[:,12], 0.04, 200, 6)
#SSAFRL= butter_lowpass_filter(LinObs[:,11], 0.045, 200, 6)
#SSARRL= butter_lowpass_filter(LinObs[:,12], 0.04, 200, 6)
#
#plt.plot(LinObs[:,0],SSAFRL*kk, 'r', linewidth=3.2)
#plt.plot(NLobs[:,0],SSAF*kk, 'b', linewidth=3.2)
#plt.plot(LinObs[:,0],SSARRL*kk, 'r--', linewidth=3.2)
#plt.plot(NLobs[:,0],SSAR*kk, 'b--', linewidth=3.2)
#
#
#plt.xlabel('Curvilinear abscissa  [m]', fontsize=35)
#plt.ylabel('Slip angles [deg]', fontsize=35)
#plt.grid(True)
#plt.legend(["$\\beta_f$ at $V=4 m.s^{-1} $ Linear observer","$\\beta_f$ at $V=4 m.s^{-1} $ non-linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ non-linear  observer"],loc='upper left',fontsize=30)
#plt.yticks( color='k', size=42)
#plt.xticks( color='k', size=42)
#
#plt.show()
##### Braquage ################################################################################

yf4= butter_lowpass_filter(NLobs[:,7], 0.045, 200, 6)
yf8= butter_lowpass_filter(LinObs[:,7], 0.03, 200, 6)
yf6= butter_lowpass_filter(WithoutObs[:,7], 0.04, 200, 6)




plt.plot(s6[:],180/pi*yf6, 'k', linewidth=3)
plt.plot(s8[:],180/pi*yf8, 'r', linewidth=3)
plt.plot(s4[:],180/pi*yf4, 'b', linewidth=3)


plt.xlabel('Curvilinear abscissa  [m]', fontsize=35)
plt.ylabel('Steering angles [deg]', fontsize=35)
plt.grid(True)
plt.legend(["$\\delta_f$ at $V=4 m.s^{-1} $ without observer","$\\delta_f$ at $V=4 m.s^{-1} $ linear observer","$\\delta_f$ at $V=4 m.s^{-1} $ non-linear observer"],loc='upper left',fontsize=30)

plt.xlim((15, 115))   # set the xlim to xmin, xmax
plt.ylim((-15, 15))   # set the xlim to xmin, xmax
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlim((3, 115))   # set the xlim to xmin, xmax

plt.show()








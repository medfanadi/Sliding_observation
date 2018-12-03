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

NLobs=np.loadtxt('/home/summit/Spidoo_ws/src/spido_sloping/scripts/Files/Slops_Cor6.txt') #NLobser_5
WithoutObs=np.loadtxt('/home/summit/Spidoo_ws/src/spido_sloping/scripts/Files/Without_Slops.txt') #NLobser_5


XNL = butter_lowpass_filter(NLobs[0:len(NLobs)-2600,1], 0.04, 10, 8)
YNL = butter_lowpass_filter(NLobs[0:len(NLobs)-2600,2], 0.04, 10, 8)

XWobs = butter_lowpass_filter(WithoutObs[:,1], 0.04, 10, 8)
YWobs = butter_lowpass_filter(WithoutObs[:,2], 0.04, 10, 8)

Xref= butter_lowpass_filter(NLobs[:,10], 0.04, 10, 8)
Yref= butter_lowpass_filter(NLobs[:,11], 0.04, 10, 8)

Xref=Xref[:]+XNL[0]-Xref[0]
Yref=Yref[:]+YNL[0]-Yref[0]

######   Trajectory plot   #################################################################

plt.plot(Xref,Yref, 'g', linewidth=3.20)

plt.plot(XNL ,YNL , 'b--', linewidth=3.20)

plt.plot(XWobs ,YWobs , 'r--', linewidth=3.20)

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

temps1=WithoutObs[:,0]
temps2=WithoutObs[:,0]
zoom=4


ix = interpolate.interp1d(WithoutObs[:,0],WithoutObs[:,1])
iy = interpolate.interp1d(WithoutObs[:,0],WithoutObs[:,2])
iyaw = interpolate.interp1d(WithoutObs[:,0],WithoutObs[:,3])
ibf = interpolate.interp1d(WithoutObs[:,0],WithoutObs[:,8]*4.3)
ibr = interpolate.interp1d(WithoutObs[:,0],WithoutObs[:,9]*4.3)#np.zeros(len(temps1)))
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
plt.xlim((-10, 95))   # set the xlim to xmin, xmax
#
#plt.ylim((-20, 100))   # set the xlim to xmin, xmax

plt.grid(True)
plt.legend(["Reference path","Path with observer","Path without observer"],fontsize=30)
plt.show()


############################  Curvilinear abscissa  ##############################

s4=np.zeros(len(NLobs[:,1]))
#s8=np.zeros(len(LinObs[:,1]))
s6=np.zeros(len(WithoutObs[:,1]))


i=0
while (i<len(NLobs[:,1])-1):
	s4[i+1]=s4[i]+hypot(NLobs[i+1,1]-NLobs[i,1],NLobs[i+1,2]-NLobs[i,2])
	i=i+1

#while (i<len(LinObs[:,1])-1):
 #       s8[i+1]=s8[i]+hypot(LinObs[i+1,1]-LinObs[i,1],LinObs[i+1,2]-LinObs[i,2])
#	i=i+1
i=0
while (i<len(WithoutObs[:,1])-1):
        s6[i+1]=s6[i]+hypot(WithoutObs[i+1,1]-WithoutObs[i,1],WithoutObs[i+1,2]-WithoutObs[i,2])
	i=i+1
############################  Slopping ##############################
bank = butter_lowpass_filter(NLobs[:,23], 0.008, 10, 8)
slope = butter_lowpass_filter(NLobs[:,22], 0.008, 10, 8)

plt.plot(s4,-slope*(180/pi), 'b', linewidth=3.2)

plt.plot(s4,bank*(180/pi), 'r', linewidth=3.2)

plt.xlim((8.5, 119.7))
plt.ylim((-25, 25))
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlabel('Curvilinear abscissa  [m]', fontsize=39)
plt.ylabel('Road angles [degree]', fontsize=39)
plt.grid(True)
plt.legend(["Slope angle at $V=4 m.s^{-1} $","Bank angle at $V=4 m.s^{-1} $"],loc='upper left',fontsize=32)

plt.show()




############################  Curvilinear abscissa  ##############################
eyObs = butter_lowpass_filter(NLobs[:,4],0.02, 200, 6)
epsiObse = butter_lowpass_filter(NLobs[:,5], 0.005, 200, 6)
eyWObs = butter_lowpass_filter(WithoutObs[:,4],0.02, 200, 6)
epsiWObse = butter_lowpass_filter(WithoutObs[:,5], 0.005, 200, 6)

plt.plot(s4,eyObs*1.5, 'r', linewidth=3.2)
plt.plot(s6,eyWObs*2, 'b', linewidth=3.2)




plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlabel('Curvilinear abscissa  [m]', fontsize=39)
plt.ylabel('Lateral error [m]', fontsize=39)
plt.grid(True)
plt.legend(["$e_y$ With observer","$e_y$ Without observer"],loc='upper left',fontsize=32)
plt.xlim((0, 115))
plt.ylim((-0.15, 0.15))
plt.show()


######
plt.plot(s4,epsiObse*(180/pi)*1.5, 'r', linewidth=3.2)
plt.plot(s6,epsiWObse*(180/pi)*2, 'b', linewidth=3.2)


plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlabel('Curvilinear abscissa  [m]', fontsize=39)
plt.ylabel('Angular error [deg]', fontsize=39)
plt.grid(True)
plt.legend(["$e_\\psi$ With observer","$e_\\psi$ Without observer"],loc='upper left',fontsize=32)
plt.xlim((0, 112))
plt.ylim((-2, 2))
plt.show()


############################  Steering angles  ##############################
deltafObs = butter_lowpass_filter(NLobs[:,8],0.025, 200, 6)
deltarObse = butter_lowpass_filter(NLobs[:,9], 0.025, 200, 6)
deltafWObs = butter_lowpass_filter(WithoutObs[:,8],0.025, 200, 6)
deltarWObse = butter_lowpass_filter(WithoutObs[:,9], 0.025, 200, 6)

plt.plot(s4,deltafObs*kk, 'r', linewidth=3.2)
plt.plot(s6,deltafWObs*kk, 'b', linewidth=3.2)

plt.plot(s4,deltarObse*kk, 'r--', linewidth=3.2)


plt.plot(s6,deltarWObse*kk, 'b--', linewidth=3.2)


plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
plt.xlabel('Curvilinear abscissa  [m]', fontsize=35)
plt.ylabel('Steering angles [deg]', fontsize=35)
plt.grid(True)
plt.legend(["$\\delta_f$  with observer","$\\delta_f$  without observer","$\\delta_r$  with observer","$\\delta_r$  without observer"],loc='upper left',fontsize=30)
plt.xlim((2, 113))
plt.ylim((-6, 9))
plt.show()

#at $V=4 m.s^{-1} $

#######   SSA  ###################################################################

SSAF= butter_lowpass_filter(NLobs[:,18], 0.02, 200, 6)
SSAR= butter_lowpass_filter(NLobs[:,19], 0.02, 200, 6)
SSAFRL= butter_lowpass_filter(WithoutObs[:,18], 0.02, 200, 6)
SSARRL= butter_lowpass_filter(WithoutObs[:,19], 0.02, 200, 6)


plt.plot(s4[:],SSAF*kk, 'b', linewidth=3.2)
plt.plot(s6[:],SSAFRL*kk*2, 'r', linewidth=3.2)

plt.plot(s4[:],SSAR*kk, 'b--', linewidth=3.2)


plt.plot(s6[:],SSARRL*kk*2, 'r--', linewidth=3.2)




plt.xlabel('Curvilinear abscissa  [m]', fontsize=35)
plt.ylabel('Slip angles [deg]', fontsize=35)
plt.grid(True)
plt.legend(["$\\beta_f$ with observer","$\\beta_f$ without observer","$\\beta_r$ with observer","$\\beta_r$ without observer"],loc='upper right',fontsize=30)
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
#plt.xlim((0, 112))   # set the xlim to xmin, xmax
plt.xlim((4.5, 115.5))
plt.ylim((-4, 4))
plt.show()


#######   forces  ###################################################################

SSAF= butter_lowpass_filter(NLobs[:,20], 0.02, 200, 6)
SSAR= butter_lowpass_filter(NLobs[:,21], 0.02, 200, 6)
SSAFRL= butter_lowpass_filter(WithoutObs[:,20], 0.02, 200, 6)
SSARRL= butter_lowpass_filter(WithoutObs[:,21], 0.02, 200, 6)

plt.plot(s6[:],SSAFRL*kk*1.5, 'r', linewidth=3.2)
plt.plot(s4[:],SSAF*kk, 'b', linewidth=3.2)
plt.plot(s6[:],SSARRL*kk*1.5, 'r--', linewidth=3.2)
plt.plot(s4[:],SSAR*kk, 'b--', linewidth=3.2)


plt.xlabel('Curvilinear abscissa  [m]', fontsize=35)
plt.ylabel('Slip angles [deg]', fontsize=35)
plt.grid(True)
plt.legend(["$\\beta_f$ at $V=4 m.s^{-1} $ Linear observer","$\\beta_f$ at $V=4 m.s^{-1} $ non-linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ non-linear  observer"],loc='upper left',fontsize=30)
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
#plt.xlim((0, 112))   # set the xlim to xmin, xmax
plt.xlim((0, 119))
#plt.ylim((-7, 7))
plt.show()



#######   cf cr  ###################################################################

SSAF= butter_lowpass_filter(NLobs[:,16], 0.04, 200, 6)
SSAR= butter_lowpass_filter(NLobs[:,17], 0.04, 200, 6)


plt.plot(s4[:]-10*np.ones((len(s4))),NLobs[:,16]-16200, 'r', linewidth=3.2)
plt.plot(s4[:]-10*np.ones((len(s4))),NLobs[:,17]-12000, 'b', linewidth=3.2)



plt.xlabel('Curvilinear abscissa  [m]', fontsize=45)
plt.ylabel('Cornering stiffness $[N.rad^{-1}]$', fontsize=45)
plt.grid(True)
plt.legend(["$\\hat{C}_f$ at $V_x=4m.s^{-1}$"," $\\hat{C}_r$ at $V_x=4m.s^{-1}$"],fontsize=45)


#plt.legend(["$\\beta_f$ at $V=4 m.s^{-1} $ Linear observer","$\\beta_f$ at $V=4 m.s^{-1} $ non-linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ non-linear  observer"],loc='upper left',fontsize=30)
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
#plt.xlim((0, 112))   # set the xlim to xmin, xmax
plt.xlim((0, 118))
#plt.ylim((-7, 7))
plt.show()


#######   Vpsi  ###################################################################

Vpsi_mes= butter_lowpass_filter(NLobs[:,7], 0.01, 200, 6)
Vpsi_obs= butter_lowpass_filter(NLobs[:,13], 0.01, 200, 6)


plt.plot(s4[:],Vpsi_mes+0.01, 'r', linewidth=3.2)
plt.plot(s4[:],Vpsi_obs, 'b--', linewidth=3.2)



plt.xlabel('Curvilinear abscissa  [m]', fontsize=45)
plt.ylabel('Yaw rate $[rad.s^{-1}]$', fontsize=45)
plt.grid(True)
plt.legend(["$V_\\psi$ at $V_x=8m.s^{-1}$"," $\\hat{V}_\\psi$ at $V_x=8m.s^{-1}$"],loc='upper left',fontsize=32)


#plt.legend(["$\\beta_f$ at $V=4 m.s^{-1} $ Linear observer","$\\beta_f$ at $V=4 m.s^{-1} $ non-linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ non-linear  observer"],loc='upper left',fontsize=30)
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
#plt.xlim((0, 112))   # set the xlim to xmin, xmax
plt.xlim((0, 118))
#plt.ylim((-7, 7))
plt.show()


#######   Vy  ###################################################################

Vy_mes= butter_lowpass_filter(NLobs[:,6], 0.01, 200, 6)



plt.plot(s4[:],Vy_mes, 'r', linewidth=3.2)




plt.xlabel('Curvilinear abscissa  [m]', fontsize=45)
plt.ylabel('Lateral velocity $[m.s^{-1}]$', fontsize=45)
plt.grid(True)



#plt.legend(["$\\beta_f$ at $V=4 m.s^{-1} $ Linear observer","$\\beta_f$ at $V=4 m.s^{-1} $ non-linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ linear observer","$\\beta_r$ at $V=4 m.s^{-1} $ non-linear  observer"],loc='upper left',fontsize=30)
plt.yticks( color='k', size=42)
plt.xticks( color='k', size=42)
#plt.xlim((0, 112))   # set the xlim to xmin, xmax
plt.xlim((5, 115))
plt.ylim((-0.2, 0.2))
plt.show()




#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from math import cos,sin

def draw_car(x,lf,lr,tr,dx,dy,rw,zoom):
	
	lf=zoom*lf	
	lr=zoom*lr	
	tr=zoom*tr	
	dx=zoom*dx	
	dy=zoom*dy	
	rw=zoom*rw	
	M=np.array([
	[ -lr-dx, lf, lf+dx, lf+dx, lf, -lr-dx, -lr-dx, -lr,-lr, -lr, -lr,   lf,    lf, lf],
	[ -tr+dy, -tr+dy, -tr/3, tr/3,  tr-dy, tr-dy,  -tr+dy, -tr+dy, -tr, tr,  tr-dy, tr-dy, tr, -tr],   [1,1,1,1,1,1,1,1,1,1,1,1,1,1]])
	W=np.array([[-rw, rw],[0, 0],[1, 1]])  # wheel
	R=np.array([[cos(x[2]),-sin(x[2]),x[0]],[sin(x[2]),cos(x[2]),x[1]],[0, 0 ,1]])
	M=R.dot(M)    
	Ravg=R.dot(np.array([[cos(x[3]),-sin(x[3]), lf],[sin(x[3]),cos(x[3]),  tr],[0, 0,1]]).dot(W))
	Ravd=R.dot(np.array([[cos(x[3]),-sin(x[3]), lf],[sin(x[3]),cos(x[3]), -tr],[0, 0,1]]).dot(W))
	Rarg=R.dot(np.array([[cos(x[4]),-sin(x[4]), -lr],[sin(x[4]),cos(x[4]),  tr],[0, 0,1]]).dot(W))
	Rard=R.dot(np.array([[cos(x[4]),-sin(x[4]), -lr],[sin(x[4]),cos(x[4]), -tr],[0, 0,1]]).dot(W))
	plt.plot(M[0,:],M[1,:],'k')           
	plt.plot(Ravd[0,:],Ravd[1,:],'r') 
	plt.plot(Ravg[0,:],Ravg[1,:],'r') 
	plt.plot(Rard[0,:],Rard[1,:],'r') 
	plt.plot(Rarg[0,:],Rarg[1,:],'r') 

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from math import *


Rt=6371000;  # earth radius in meters
latitude_init=39.5080331  #fix point GPS: sur gazebo
longitude_init=-0.4619816

# == Paramètres du modèle ======================================================

vit=2 # Car speed

#==   kinematic Parameters ============================

axles_dist = 1.5 # distance between axles

target_curve = 3  # if dist<target_curve, the vehicle is close to the curve


#==   Dynamic Parameters ============================
Cf      = 15000   #rigidité de dérive du train avant
Cr      = 15000    # rigidité de dérive du train arrière
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

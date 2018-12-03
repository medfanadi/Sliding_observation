
import numpy as np
from scipy.signal import butter, lfilter, freqz, firwin
import matplotlib.pyplot as plt
from scipy import fftpack
from matplotlib import pyplot as plt
from math import *
import shapely.geometry as geom
from scipy import interpolate
from scipy import signal

import shapely.geometry as geom
from scipy import spatial
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round


def butter_lowpass_filter(data, cutOff, fs, order=6):
    b, a = signal.butter(order, cutOff)
    y =signal.filtfilt(b, a, data)
    return y

def firwin_filter(data, cutOff, fs, order=6):
    numtaps = 50
    a = signal.firwin(numtaps, cutOff, window = "hamming")
    b = [1.0]
    y =signal.filtfilt(b, a, data)
    return y

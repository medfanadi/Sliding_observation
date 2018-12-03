#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from nav_msgs.msg import Odometry
import rospy
from matplotlib import pyplot

gpsx = 0.0
gpsy = 0.0

################################################################################
def tracegpscallback(data):
    global gpsx
    global gpsy
    gpsx = data.pose.pose.position.x
    gpsy = data.pose.pose.position.y
################################################################################
def tracegps():
    global Cff
    global Crr
    real_trajX = []
    real_trajY = []
    
    rospy.init_node('trace_cor')
    
    rospy.Subscriber("cor_data", Odometry, tracegpscallback)
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        real_trajX.append(gpsx)
        real_trajY.append(gpsy)
        pyplot.plot(real_trajX, real_trajY, 'b+')
        pyplot.draw()
        pyplot.pause(0.01)
        r.sleep()
################################################################################
if __name__ == '__main__':
    try:
        tracegps()
    except rospy.ROSInterruptException: pass

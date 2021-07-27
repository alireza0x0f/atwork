#!/usr/bin/env python

import roslib
import rospy
import geometry_msgs.msg
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

#from fsm import Fsm
#from fsm_manager import FsmManager
import copy
import math
from atwork_ros_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion


         
        
def laser_callback( data):
    global ranges
    ranges = data.ranges
    
    '''for i in range(360):
        if ranges[i] > 1:
            print str(i) + "  " + str(ranges[i])'''
    print str(ranges[180]) +"   " + str(data.intensities[180])



if __name__ == '__main__':

    rospy.init_node('meter_get', anonymous=True)
    rospy.Subscriber('scan', LaserScan , laser_callback)
    
    rospy.spin()
#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from iauk.srv import *
import motor
from sensor_msgs.msg import JointState
import time


joint0 = 50
joint1 = 51
joint2 = 52
joint3 = 53
joint4 = 54
gripper = 55


if __name__ == '__main__':
    rospy.init_node('armfault',anonymous=True)

    motor.ser = motor.openSerial_servo_arm()
    motor.test()

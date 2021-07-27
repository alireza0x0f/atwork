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

joint0_val = 3447
joint1_val = 3490
joint2_val = 1702
joint3_val = 3200 #3658
joint4_val = 3175


def joint_callback(data):
    global joint0_val
    global joint1_val
    global joint2_val
    global joint3_val
    global joint4_val


    joint0_val = int(6721 - ((data.position[0] - 1.4112) * 2139.83) + 34)
    joint1_val = int((data.position[1]  * 2675.83) + 3610) #3610
    joint2_val = int(((data.position[2] + 0.03) * 1714.08) + 1702)
    joint3_val = int(4712 - ((data.position[3] - 0.2333) * 1514.96) + 1700) # the last number is position of motor whene joint is in 3.3436
    joint4_val = int(3431 - ((data.position[4] - 1.3344) * 1069.24) + 1497)


if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('armdyna',anonymous=True)

    motor.ser = motor.openSerial_servo_arm()

    print( motor.check_id(joint0))
    print( motor.check_id(joint1))
    print( motor.check_id(joint2))
    print( motor.check_id(joint3))
    print( motor.check_id(joint4))

    motor.writeParameter(joint0,[73,10])
    motor.writeParameter(joint1,[73,10])
    motor.writeParameter(joint2,[73,10])
    motor.writeParameter(joint3,[73,10])
    motor.writeParameter(joint4,[73,10])

    print motor.getPos(joint0)
    print motor.getPos(joint1)
    print motor.getPos(joint2)
    print motor.getPos(joint3)
    print motor.getPos(joint4)

    motor.setPos(joint0,joint0_val,200)
    motor.setPos(joint1,joint1_val,200)
    motor.setPos(joint2,joint2_val,200)
    motor.setPos(joint3,joint3_val,200)
    motor.setPos(joint4,joint4_val,200)


    rospy.set_param('/armGripperOpen',False)
    rospy.set_param('/armGripperClose',False)
    rospy.set_param('/armCanGo',False)

    rospy.Subscriber("/joint_states", JointState , joint_callback)

    while not rospy.is_shutdown():

        if rospy.get_param('/armGripperOpen') == True:
            motor.setPos(gripper,24000,100)
            t = motor.getTrq(gripper)
            print t
            while t < 1250:
                t = motor.getTrq(gripper)

            p =  motor.getPos(gripper) - 300
            motor.setPos(gripper,p,200)
            rospy.set_param('/armGripperOpen',False)

        if rospy.get_param('/armGripperClose') == True:
            gripperStartPos = motor.getPos(gripper)
            motor.setPos(gripper,-24000,100)
            t = motor.getTrq(gripper)
            #print t
            while t < 500:
                t = motor.getTrq(gripper)
                #print t
            p =  motor.getPos(gripper)
            motor.setPos(gripper,p,100)
            rospy.set_param('/armGripperOffset',(gripperStartPos-p))
            rospy.set_param('/armGripperClose',False)


        elif rospy.get_param('/armCanGo', False) == True:
            motor.setPos(joint0,joint0_val,400)
            motor.setPos(joint1,joint1_val,400)
            motor.setPos(joint2,joint2_val,400)
            motor.setPos(joint3,joint3_val,400)
            motor.setPos(joint4,joint4_val,400)

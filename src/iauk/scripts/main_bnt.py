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
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import *





'''anager =  FsmManager()

task= Fsm('dellay')
task.dellay = 3
manager.fsms.append(task)
task = copy.deepcopy(task)'''

robot_x_position = 0
robot_y_position = 0
robot_last_location = 'start'
task_index = 0
is_nav_finished = True
navigation_status = 0

client = None


getFirstTime = True
velocity_publisher = None
arm_publisher = None
ranges= []


isArmMeterEnable = True



GOAL_POSE ={'start'         : [ 0 , 0],
            'Workstation 1' : [ 3.284518 , 0.416930],
            'Workstation 2' : [ 4.432668 , 0.997913],
            'Workstation 3' : [ 4.527204 , -1.089611],
            'Workstation 4' : [ 4.422811 , -2.198312],
            'Workstation 5' : [ 1.734271 , -4.488999],
            'Workstation 6' : [ 3.091582 , -4.392856],
            'Workstation 7' : [ 2.236971 , -3.302536],
            'Workstation 8' : [ 2.236971 , -3.302536],
            'Workstation 9' : [ 0.190655 , -3.302536],
            'Workstation 10' : [ 2.623505 , -0.751156],
            'Workstation 11' : [ 0.218311 , -1.621838],
            'Waypoint 1' : [ 1.925135 , -0.548746],
            'Waypoint 2' : [ 3.006380 , 0.230177],
            'Waypoint 3' : [ 4.581704 , 0.078143],
            'Waypoint 4' : [ 4.150191 , -1.357078],
            'Waypoint 5' : [ 3.192821 , -2.105676],
            'Waypoint 6' : [ 3.854783 , -3.405527],
            'Waypoint 7' : [ 3.459734 , -4.235692],
            'Waypoint 8' : [ 2.238890 , -4.189229],
            'Waypoint 9' : [ 0.962362 , -4.538254],
            'Waypoint 10' : [ -0.069348 , -3.855102],
            'Waypoint 11' : [ 0.648591 , -2.802625],
            'Waypoint 12' : [ 0.546828 , -2.064440],
            'Waypoint 13' : [ 1.251734 , -2.137473],
            'Waypoint 14' : [ 2.277984 , -2.112924],
            'Waypoint 15' : [ 0.691037 , -0.913565],
            'Precision Platform' : [ 0.616584 , -4.219434],
            'Shelf 1' : [ 4.994752 , -3.571197],
            'Shelf 2' : [ 0.190655 , -2.889803],
            'Rotating Table' : [ 1.647644 , 0.317350],
            'Conveyor Belt' : [ 1.647644 , 0.317350],
            'finish' : [ 5.308210 , -4.957252],
            }

GOAL_ROTA = {'start'         :  0 ,
            'Workstation 1' :  1.57 ,
            'Workstation 2' :  0 ,
            'Workstation 3' :  0 ,
            'Workstation 4' :  0 ,
            'Workstation 5' :  -1.57 ,
            'Workstation 6' :  -1.57 ,
            'Workstation 7' :  0 ,
            'Workstation 8' :  3.14 ,
            'Workstation 9' :  0 ,
            'Workstation 10' :  -1.57 ,
            'Workstation 11' :  3.14 ,
            'Waypoint 1' :  0 ,
            'Waypoint 2' :  0 ,
            'Waypoint 3' :  0 ,
            'Waypoint 4' :  0 ,
            'Waypoint 5' :  0 ,
            'Waypoint 6' :  0 ,
            'Waypoint 7' :  0 ,
            'Waypoint 8' :  0 ,
            'Waypoint 9' :  0 ,
            'Waypoint 10' :  0 ,
            'Waypoint 11' :  0 ,
            'Waypoint 12' :  0 ,
            'Waypoint 13' :  0 ,
            'Waypoint 14' :  0 ,
            'Waypoint 15' :  0 ,
            'Precision Platform' : -1.57,
            'Shelf 1' : 1.57,
            'Shelf 2' : 3.14,
            'Rotating Table' : 1.57,
            'Conveyor Belt' : 1.57,
            'finish' :  0 ,
            }



ENTER_ACTION ={ 'start'         :[["none"           ,0      ,0]],

                'Workstation 1' :[["direction"     ,3    ,0.00],
                                  ["moveForward"   ,0.30     ,0.1]],

                'Workstation 2':[["direction"      ,1    ,0],
                                 ["wallForward"    ,0.35 ,0.00]],

                'Workstation 3':[["direction"      ,1    ,0],
                                 ["wallForward"    ,0.35 ,0.00]],

                'Workstation 4':[["direction"      ,1    ,0],
                                 ["moveForward"    ,0.20 ,0.05]],

                'Workstation 5':[["direction"      ,2    ,0],
                                 ["wallForward"    ,0.35 ,0.00]],

                'Workstation 6':[["direction"      ,2    ,0],
                                 ["wallForward"    ,0.35 ,0.00]],

                'Workstation 7':[["direction"      ,1    ,0],
                                 ["wallBackward"   ,1.05 ,0.00]],

                'Workstation 8':[["direction"      ,4    ,0],
                                 ["wallForward"    ,0.35 ,0.00]],

                'Workstation 9':[["direction"      ,1    ,0],
                                 ["wallForward"    ,0.35 ,0.00]],

                'Workstation 10':[["direction"      ,2    ,0],
                                  ["wallForward"    ,0.35 ,0.00],
                                  ["wallLeft"       ,0.95 ,0.00]],

                'Workstation 11':[["direction"      ,4    ,0],
                                  ["wallForward"    ,0.35 ,0.00]],

                'Precision Platform':[["direction"      ,2    ,0],
                                      ["rotateRight"    ,0.78 ,0.5],
                                      ["wallForward"    ,0.35 ,0.00]],

                'Shelf 1'       :[["direction"      ,3    ,0],
                                  ["angleBackward"  ,0.0  ,0.00]],

                'Shelf 2'       :[["direction"      ,4    ,0]],

                'Rotating Table':[["direction"      ,3    ,0],
                                  ["wallBackward"   ,2.20 ,0.00]],

                'Conveyor Belt' :[["direction"      ,3    ,0],
                                  ["wallBackward"   ,2.20 ,0.00]],

                'Waypoint 1'    :[["none"           ,0      ,0]],
                'Waypoint 2'    :[["none"           ,0      ,0]],
                'Waypoint 3'    :[["none"           ,0      ,0]],
                'Waypoint 4'    :[["none"           ,0      ,0]],
                'Waypoint 5'    :[["none"           ,0      ,0]],
                'Waypoint 6'    :[["none"           ,0      ,0]],
                'Waypoint 7'    :[["none"           ,0      ,0]],
                'Waypoint 8'    :[["none"           ,0      ,0]],
                'Waypoint 9'    :[["none"           ,0      ,0]],
                'Waypoint 10'    :[["none"           ,0      ,0]],
                'Waypoint 11'    :[["none"           ,0      ,0]],
                'Waypoint 12'    :[["none"           ,0      ,0]],
                'Waypoint 13'    :[["none"           ,0      ,0]],
                'Waypoint 14'    :[["none"           ,0      ,0]],
                'Waypoint 15'    :[["none"           ,0      ,0]],

                'finish'        :[["none"           ,0      ,0]]}

EXIT_ACTION ={  'start'         :[["moveForward"   ,1.6  ,0.2]],

                'Workstation 1' :[["direction"      ,3    ,0],
                                  ["moveBackward"  ,0.25 ,0.1],
                                  ["rotateRight"   ,1.57 ,0.7]],

                'Workstation 2':[["direction"      ,1    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,1.57 ,0.7]],

                'Workstation 3':[["direction"      ,1    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,1.57 ,0.7]],

                'Workstation 4':[["direction"      ,1    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,3.14 ,0.7]],

                'Workstation 5':[["direction"      ,2    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,3.14 ,0.7]],

                'Workstation 6':[["direction"      ,2    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,3.14 ,0.7]],

                'Workstation 7':[["direction"      ,1    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,1.57 ,0.7]],

                'Workstation 8':[["direction"      ,4    ,0],
                                 ["moveBackward"   ,0.40 ,0.1],
                                 ["rotateLeft"     ,1.57 ,0.7]],

                'Workstation 9':[["direction"      ,1    ,0],
                                 ["moveBackward"   ,0.40 ,0.1]],

                'Workstation 10':[["direction"      ,2    ,0],
                                 ["moveBackward"   ,0.40 ,0.1]],

                'Workstation 11':[["direction"      ,4    ,0],
                                  ["moveBackward"   ,0.40 ,0.1],
                                  ["rotateRight"    ,3.14 ,0.7]],

                'Precision Platform':[["direction"      ,2    ,0],
                                      ["moveBackward"   ,0.40 ,0.1],
                                      ["rotateRight"    ,3.14 ,0.5]],

                'Shelf 1'       :[["direction"      ,3    ,0],
                                  ["moveLeft"       ,0.40 ,0.1],
                                  ["rotateLeft"     ,1.57 ,0.7]],

                'Shelf 2'       :[["direction"      ,4    ,0],
                                  ["moveBackward"   ,0.20 ,0.1],
                                  ["rotateRight"    ,3.14 ,0.7]],

                'Rotating Table':[["direction"      ,3    ,0],
                                  ["moveBackward"   ,0.40 ,0.1],
                                  ["rotateRight"    ,3.14 ,0.7]],

                'Conveyor Belt' :[["direction"      ,3    ,0],
                                  ["moveBackward"   ,0.40 ,0.1],
                                  ["rotateLeft"    ,1.57 ,0.7]],

                'Waypoint 1'    :[["none"           ,0      ,0]],
                'Waypoint 2'    :[["none"           ,0      ,0]],
                'Waypoint 3'    :[["none"           ,0      ,0]],
                'Waypoint 4'    :[["none"           ,0      ,0]],
                'Waypoint 5'    :[["none"           ,0      ,0]],
                'Waypoint 6'    :[["none"           ,0      ,0]],
                'Waypoint 7'    :[["none"           ,0      ,0]],
                'Waypoint 8'    :[["none"           ,0      ,0]],
                'Waypoint 9'    :[["none"           ,0      ,0]],
                'Waypoint 10'    :[["none"           ,0      ,0]],
                'Waypoint 11'    :[["none"           ,0      ,0]],
                'Waypoint 12'    :[["none"           ,0      ,0]],
                'Waypoint 13'    :[["none"           ,0      ,0]],
                'Waypoint 14'    :[["none"           ,0      ,0]],
                'Waypoint 15'    :[["none"           ,0      ,0]],

                'finish'        :[["none"           ,0      ,0]]}

TASK_LIST = []





def subAction(action,distance=None,speed=None):
    global velocity_publisher

    if action=="moveForward":
        vel_msg = Twist()
        vel_msg.linear.x = speed
        velocity_publisher.publish(vel_msg)
        rospy.sleep(distance/speed)
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.5)
    elif action=="moveBackward":
        vel_msg = Twist()
        vel_msg.linear.x = speed * -1
        velocity_publisher.publish(vel_msg)
        rospy.sleep(distance/speed)
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.5)
    elif action=="moveRight":
        vel_msg = Twist()
        vel_msg.linear.y = speed * -1
        velocity_publisher.publish(vel_msg)
        rospy.sleep(distance/speed)
        vel_msg.linear.y = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.5)
    elif action=="moveLeft":
        vel_msg = Twist()
        vel_msg.linear.y = speed
        velocity_publisher.publish(vel_msg)
        rospy.sleep(distance/speed)
        vel_msg.linear.y = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.5)
    elif action=="rotateLeft":
        vel_msg = Twist()
        vel_msg.angular.z = speed
        velocity_publisher.publish(vel_msg)
        rospy.sleep(distance/speed)
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.5)
    elif action=="rotateRight":
        vel_msg = Twist()
        vel_msg.angular.z = speed * -1
        velocity_publisher.publish(vel_msg)
        rospy.sleep(distance/speed)
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.5)
    elif action=="wallForward":
        shouldBreak = False
	msg = Twist()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and not math.isinf(ranges[182]):
                dis  = float(ranges[182])
                if dis > (distance+0.005):
                    msg.linear.x = abs(dis-distance)/float(2) #0.02
                    velocity_publisher.publish(msg)
                elif dis < (distance-0.005) :
                    msg.linear.x = -1 * abs(dis-distance)/float(2) #-0.02
                    velocity_publisher.publish(msg)
                else:
                    msg.linear.x = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="wallBackward":
        shouldBreak = False
	msg = Twist()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and not math.isinf(ranges[2]):
                dis  = float(ranges[2])
                if dis > (distance+0.005):
                    msg.linear.x = -1 * abs(dis-distance)/float(2) #0.02
                    velocity_publisher.publish(msg)
                elif dis < (distance-0.005) :
                    msg.linear.x =  abs(dis-distance)/float(2) #-0.02
                    velocity_publisher.publish(msg)
                else:
                    msg.linear.x = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="wallRight":
        shouldBreak = False
	msg = Twist()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and not math.isinf(ranges[92]):
                dis  = float(ranges[92])
                if dis > (distance+0.005):
                    msg.linear.y = -1 * abs(dis-distance)/float(2) #0.02
                    velocity_publisher.publish(msg)
                elif dis < (distance-0.005) :
                    msg.linear.y = abs(dis-distance)/float(2) #-0.02
                    velocity_publisher.publish(msg)
                else:
                    msg.linear.y = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="wallLeft":
        shouldBreak = False
	msg = Twist()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and not math.isinf(ranges[272]):
                dis  = float(ranges[272])
                if dis > (distance+0.005):
                    msg.linear.y = abs(dis-distance)/float(2) #0.02
                    velocity_publisher.publish(msg)
                elif dis < (distance-0.005) :
                    msg.linear.y = -1 * abs(dis-distance)/float(2) #-0.02
                    velocity_publisher.publish(msg)
                else:
                    msg.linear.y = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="angleForward":
        msg = Twist()
        rate = rospy.Rate(1)
        shouldBreak = False
        while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and ranges[167]!='inf' and ranges[197]!='inf':
                aSide , bSide = float(ranges[197]),float(ranges[167]);
                cSide = math.sqrt(abs((aSide**2)+(bSide**2)) - (2*aSide*bSide*math.cos(0.5235)))
                beta = math.acos(((cSide**2)+(aSide**2)-(bSide**2)) / float(2*aSide*cSide))
                teta = 1000
                teta = 1.57079 - (beta+0.26179939)
                if teta > 0.02:
                    msg.angular.z = -1* abs(teta/3)
                    velocity_publisher.publish(msg)
                elif teta < -0.02 :
                    msg.angular.z =  abs(teta/3)
                    velocity_publisher.publish(msg)
                elif not math.isnan(teta) :
                    msg.angular.z = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="angleBackward":
        msg = Twist()
        rate = rospy.Rate(1)
        shouldBreak = False
        while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and ranges[346]!='inf' and ranges[17]!='inf':
                aSide , bSide = float(ranges[17]),float(ranges[346]);
                cSide = math.sqrt(abs((aSide**2)+(bSide**2)) - (2*aSide*bSide*math.cos(0.5235)))
                beta = math.acos(((cSide**2)+(aSide**2)-(bSide**2)) / float(2*aSide*cSide))
                teta = 1000
                teta = 1.57079 - (beta+0.26179939)
                if teta > 0.02:
                    msg.angular.z = -1* abs(teta/3)
                    velocity_publisher.publish(msg)
                elif teta < -0.02 :
                    msg.angular.z =  abs(teta/3)
                    velocity_publisher.publish(msg)
                elif not math.isnan(teta) :
                    msg.angular.z = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="angleRight":
        msg = Twist()
        rate = rospy.Rate(1)
        shouldBreak = False
        while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and ranges[77]!='inf' and ranges[107]!='inf':
                aSide , bSide = float(ranges[107]),float(ranges[77]);
                cSide = math.sqrt(abs((aSide**2)+(bSide**2)) - (2*aSide*bSide*math.cos(0.5235)))
                beta = math.acos(((cSide**2)+(aSide**2)-(bSide**2)) / float(2*aSide*cSide))
                teta = 1000
                teta = 1.57079 - (beta+0.26179939)
                if teta > 0.02:
                    msg.angular.z = -1* abs(teta/3)
                    velocity_publisher.publish(msg)
                elif teta < -0.02 :
                    msg.angular.z =  abs(teta/3)
                    velocity_publisher.publish(msg)
                elif not math.isnan(teta) :
                    msg.angular.z = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="angleLeft":
        msg = Twist()
        rate = rospy.Rate(1)
        shouldBreak = False
        while not rospy.is_shutdown() and ( not shouldBreak):
            if len(ranges)==360 and ranges[257]!='inf' and ranges[287]!='inf':
                aSide , bSide = float(ranges[287]),float(ranges[257]);
                cSide = math.sqrt(abs((aSide**2)+(bSide**2)) - (2*aSide*bSide*math.cos(0.5235)))
                beta = math.acos(((cSide**2)+(aSide**2)-(bSide**2)) / float(2*aSide*cSide))
                teta = 1000
                teta = 1.57079 - (beta+0.26179939)
                if teta > 0.02:
                    msg.angular.z = -1* abs(teta/3)
                    velocity_publisher.publish(msg)
                elif teta < -0.02 :
                    msg.angular.z =  abs(teta/3)
                    velocity_publisher.publish(msg)
                elif not math.isnan(teta) :
                    msg.angular.z = 0
                    velocity_publisher.publish(msg)
                    shouldBreak = True
            rate.sleep()
    elif action=="direction":
        msg = Twist()
        listener = tf.TransformListener()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                if distance==1: #North
                    angle = euler_from_quaternion(rot)[2]
                elif distance==2: #east
                    angle = euler_from_quaternion(rot)[2] + 1.57
                elif distance==3: #west
                    angle = euler_from_quaternion(rot)[2] - 1.57
                elif distance==4: #south
                    angle = euler_from_quaternion(rot)[2]
                    if angle < 0:
                        angle += 3.14
                    elif angle > 0:
                        angle -=3.14
                    else:
                        angle = 0

                if angle<-0.08:
                    vel_msg = Twist()
                    vel_msg.angular.z = 0.5
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(abs(angle)/0.5)
                    vel_msg.angular.z = 0
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(0.5)
                elif angle > 0.08 :
                    vel_msg = Twist()
                    vel_msg.angular.z = -0.5
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(abs(angle)/0.5)
                    vel_msg.angular.z = 0
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(0.5)
                break

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
    elif action=="sleep":
        rospy.sleep(distance)



def do_enter_sub_actions(name):
    global ENTER_ACTION
    print name
    for i in ENTER_ACTION[name]:
        subAction(i[0],i[1],i[2])

def do_exit_sub_actions(name):
    global EXIT_ACTION
    for i in EXIT_ACTION[name]:
        subAction(i[0],i[1],i[2])


def laser_callback( data):
    global ranges
    ranges = data.ranges

def active_cb():
    print 'active'

def feedback_cb(feedback):
    global robot_x_position
    global robot_y_position
    robot_x_position = feedback.base_position.pose.position.x
    robot_y_position = feedback.base_position.pose.position.y

def cancel_the_goal():
    global client
    client.cancel_goal()

def done_cb(status, result):
    global is_nav_finished
    global navigation_status



    is_nav_finished = True
    navigation_status = status


def do_task():
    global task_index
    global robot_x_position
    global robot_y_position
    global robot_last_location
    global task_index
    global GOAL_POSE
    global GOAL_ROTA
    global ENTER_ACTION
    global EXIT_ACTION
    global TASK_LIST
    global client
    global is_nav_finished
    global navigation_status

    do_exit_sub_actions(robot_last_location)

    is_nav_finished = False
    navigation_status = 0

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    workstation = TASK_LIST[task_index][0]
    pose = GOAL_POSE[workstation]
    rot = tf.transformations.quaternion_from_euler(0, 0, GOAL_ROTA[workstation])

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose[0]
    goal.target_pose.pose.position.y = pose[1]
    goal.target_pose.pose.orientation.z = rot[2]
    goal.target_pose.pose.orientation.w = rot[3]

    '''rospy.wait_for_service('/move_base/clear_costmaps')
    clearCostmap = rospy.ServiceProxy('/move_base/clear_costmaps' , Empty)
    clearCostmap()'''

    while not (navigation_status ==3):
        is_nav_finished = False
        navigation_status = 0
        if isArmMeterEnable == True:
            rospy.set_param('/vision/meterEnable',True)
        client.send_goal(goal,done_cb,active_cb,feedback_cb)
        while not is_nav_finished and not rospy.is_shutdown(): pass

    '''rospy.wait_for_service('/move_base/clear_costmaps')
    clearCostmap = rospy.ServiceProxy('/move_base/clear_costmaps' , Empty)
    clearCostmap()'''
    ##### robot is goal position
    rospy.set_param('/vision/meterEnable',False)
    do_enter_sub_actions(workstation)
    robot_last_location = workstation

    dir = TASK_LIST[task_index][1]
    subAction('direction',dir)

    subAction('sleep',3,0)


    task_index +=1

    do_task()











def refCallBack_BNT1(data):

    global TASK_LIST
    global getFirstTime

    if getFirstTime == False :
        return

    getFirstTime = False


    for t in data.tasks:
        station = t.navigation_task.location.description.data
        direction = t.navigation_task.orientation.data

        dest = str(station)+'_'+str(direction)
        print dest

        newTask = []

        newTask.append(station)

        if direction==1: #north
            newTask.append(3) # our west
        elif direction==2: # east
            newTask.append(1) # our north
        elif direction==3: #south
            newTask.append(2) #our east
        elif direction==4: #west
            newTask.append(4) #our south

        TASK_LIST.append(newTask)

    finishTask = ['finish' , 1]
    TASK_LIST.append(finishTask)


    do_task()




if __name__ == '__main__':

    rospy.init_node('bnt_main_node', anonymous=True)
    rospy.Subscriber("/robot_example_ros/task_info", TaskInfo, refCallBack_BNT1)
    rospy.Subscriber('scan', LaserScan , laser_callback)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    arm_publisher = rospy.Publisher('/arm_go', String, queue_size=10)
    rospy.set_param('/vision/meterEnable',False)

    getFirstTime = True

    rospy.sleep(1)

    while rospy.get_param('/armCanGo', False) == False:
        rospy.sleep(0.2)


    if isArmMeterEnable == True:
        arm_publisher.publish("move_pos")
        rospy.set_param('/armBussy',True)
        while rospy.get_param('/armBussy', True) == True: pass



    rospy.sleep(1)


    rospy.spin()
    print 'finished'

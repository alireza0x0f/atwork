#!/usr/bin/env python

import roslib
import rospy
import geometry_msgs.msg
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import copy
import math
from atwork_ros_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import *
from std_srvs.srv import *
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from goto import with_goto


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
vision_publisher = None
shift_status = 0
shift_status_put = 0
ranges= []
is_all_tasks_try = False
stack= ['none','none','none']
table_delay = 0

'''Workstation 1
Waypoint 7
Rotating Table
Precision Platform
Waypoint 2
Workstation 5
Waypoint 9
Waypoint 8
Waypoint 4'''

'''['Distance Tube', 'Bearing', 'Large Nut', 'Small Black Alu. Profile', 'Small Nut']
'''

GOAL_POSE ={'start'         : [ 0 , 0],
            'Workstation 1' : [ 3.284518 , 0.416930],
            'Workstation 2' : [ 4.432668 , 0.997913],
            'Workstation 3' : [ 4.527204 , -1.089611],
            'Workstation 4' : [ 4.422811 , -2.198312],
            'Workstation 5' : [ 1.734271 , -4.488999],
            'Workstation 6' : [ 3.091582 , -4.392856],
            'Workstation 7' : [ 2.236971 , -3.302536],
            'Workstation 8' : [ 2.236971 , -3.302536],
            'Workstation 9' : [ 0.190655 , -3.402536],
            'Workstation 10' : [ 2.623505 , -0.751156],
            'Workstation 11' : [ 0.218311 , -1.621838],
            'Precision Platform' : [ 0.676584 , -4.219434],
            'Shelf 1' : [ 4.994752 , -3.571197],
            'Shelf 2' : [ 0.190655 , -2.889803],
            'Rotating Table' : [ 1.747644 , 0.317350],
            'Conveyor Belt' : [ 1.747644 , 0.317350],
            'finish' : [ 5.308210 , -4.957252]
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
            'Precision Platform' : -1.57,
            'Shelf 1' : 1.57,
            'Shelf 2' : 3.14,
            'Rotating Table' : 1.57,
            'Conveyor Belt' : 1.57,
            'finish' :  0
            }



ENTER_ACTION ={ 'start'         :[["none"           ,0      ,0]],

                'Workstation 1' :[["direction"     ,3    ,0.00],
                                  ["moveLeft"      ,0.05 ,0.05],
                                  ["moveForward"   ,0.35     ,0.05]],

                'Workstation 2':[["direction"      ,1    ,0],
                                 ["wallForward"    ,0.24 ,0.00],
                                 ["angleForward"   ,0.0  ,0.00],
                                 ["wallLeft"       ,0.40 ,0.00]],

                'Workstation 3':[["direction"      ,1    ,0],
                                 ["wallForward"    ,0.24 ,0.00],
                                 ["wallLeft"       ,0.40 ,0.00],
                                 ["angleForward"   ,0.0  ,0.00]],

                'Workstation 4':[["direction"      ,1    ,0],
                                 ["moveForward"    ,0.30 ,0.05],
                                 ["wallLeft"       ,1.67 ,0.00]],

                'Workstation 5':[["direction"      ,2    ,0],
                                 ["wallForward"    ,0.24 ,0.00],
                                 ["angleForward"   ,0.0  ,0.00]],

                'Workstation 6':[["direction"      ,2    ,0],
                                 ["wallForward"    ,0.24 ,0.00],
                                 ["angleForward"   ,0.0  ,0.00]],

                'Workstation 7':[["direction"      ,1    ,0],
                                 ["wallLeft"       ,0.85 ,0.00],
                                 ["wallBackward"   ,1.10 ,0.00]],

                'Workstation 8':[["direction"      ,4    ,0],
                                 ["wallRight"      ,0.80 ,0.00],
                                 ["wallForward"    ,0.24 ,0.00],
                                 ["angleForward"   ,0.0  ,0.00]],

                'Workstation 9':[["direction"      ,1    ,0],
                                 ["wallForward"    ,0.24 ,0.00],
                                 ["angleForward"   ,0.0  ,0.00]],

                'Workstation 10':[["direction"      ,2    ,0],
                                  ["wallForward"    ,0.24 ,0.00],
                                  ["wallLeft"       ,0.95 ,0.00],
                                  ["angleForward"   ,0.0  ,0.00]],

                'Workstation 11':[["direction"      ,4    ,0],
                                  ["wallForward"    ,0.24 ,0.00],
                                  ["angleForward"   ,0.0  ,0.00],
                                  ["wallRight"      ,1.22 ,0.00]],

                'Precision Platform':[["direction"      ,2    ,0],
                                      ["rotateRight"    ,0.78 ,0.5],
                                      ["wallForward"    ,0.24 ,0.00],
                                      ["angleForward"   ,0.0  ,0.00]],

                'Shelf 1'       :[["direction"      ,3    ,0],
                                  ["angleBackward"  ,0.0  ,0.00]],

                'Shelf 2'       :[["direction"      ,4    ,0],
                                  ["moveRight"      ,0.05 ,0.05]],

                'Rotating Table':[["direction"      ,3    ,0],
                                  ["wallBackward"   ,2.20 ,0.00]],

                'Conveyor Belt' :[["direction"      ,3    ,0],
                                  ["wallBackward"   ,2.20 ,0.00]],

                'finish'        :[["none"           ,0      ,0]]}

#'start'         :[["moveForward"   ,1.4  ,0.2]],
EXIT_ACTION ={  'start'         :[["moveForward"   ,1.4  ,0.2]],

                'Workstation 1' :[["moveBackward"  ,0.25 ,0.1],
                                  ["rotateRight"   ,1.57 ,0.5]],

                'Workstation 2':[["moveBackward"   ,0.40 ,0.1],
                                 ["moveRight"      ,0.40 ,0.1],
                                 ["rotateRight"    ,1.57 ,0.5]],

                'Workstation 3':[["moveBackward"   ,0.40 ,0.1],
                                 ["moveRight"      ,0.40 ,0.1],
                                 ["rotateRight"    ,1.57 ,0.5]],

                'Workstation 4':[["moveBackward"   ,0.40 ,0.1],
                                 ["moveLeft"       ,0.40 ,0.1],
                                 ["rotateRight"    ,3.14 ,0.5]],

                'Workstation 5':[["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,3.14 ,0.5]],

                'Workstation 6':[["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,3.14 ,0.5]],

                'Workstation 7':[["moveBackward"   ,0.40 ,0.1],
                                 ["rotateRight"    ,1.57 ,0.5]],

                'Workstation 8':[["moveBackward"   ,0.40 ,0.1],
                                 ["rotateLeft"     ,1.57 ,0.5]],

                'Workstation 9':[["moveBackward"   ,0.40 ,0.1]],

                'Workstation 10':[["moveBackward"   ,0.40 ,0.1]],

                'Workstation 11':[["moveBackward"   ,0.40 ,0.1],
                                  ["rotateRight"    ,3.14 ,0.5]],

                'Precision Platform':[["moveBackward"   ,0.40 ,0.1],
                                      ["rotateRight"    ,3.14 ,0.5]],

                'Shelf 1'       :[["moveLeft"       ,0.40 ,0.1],
                                  ["rotateLeft"     ,1.57 ,0.5]],

                'Shelf 2'       :[["moveBackward"   ,0.20 ,0.1],
                                  ["rotateRight"    ,3.14 ,0.5]],

                'Rotating Table':[["moveBackward"   ,0.40 ,0.1],
                                  ["rotateRight"    ,3.14 ,0.5]],

                'Conveyor Belt' :[["moveBackward"   ,0.40 ,0.1],
                                  ["rotateLeft"    ,1.57 ,0.5]],


                'finish'        :[["none"           ,0      ,0]]}

AREA_HEIGHT ={  'start'        :0,
                'Workstation 1'    :0.0,
                'Workstation 2'    :0.15,
                'Workstation 3'    :0.05,
                'Workstation 4'    :0.0,
                'Workstation 5'    :0.1,
                'Workstation 6'    :0.1,
                'Workstation 7'    :0.0,
                'Workstation 8'    :0.1,
                'Workstation 9'    :0.1,
                'Workstation 10'    :0.05,
                'Workstation 11'    :0.1,
                'Precision Platform'    :0.115,
                'Shelf 1'    :0.1,
                'Shelf 2'    :0.1,
                'Rotating Table'    :0.11,
                'Conveyor Belt'    :0.11,
                'finish'    :0.0}

TASK_LIST = []
################################################################################ subAction

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

################################################################################ arm go to
def arm_go_to(teta,v,h,teta4,teta5):
    global arm_publisher

    arm_publisher.publish(str(teta)+','+str(v)+','+str(h)+','+str(teta4)+','+str(teta5))
    rospy.set_param('/armBussy',True)
    while rospy.get_param('/armBussy', True) == True: pass

################################################################################ arm go to pos
def arm_go_to_pos(name , wait = True):
    global arm_publisher

    while rospy.get_param('/armBussy', True) == True: pass
    arm_publisher.publish(name)
    rospy.set_param('/armBussy',True)
    if wait == True:
        while rospy.get_param('/armBussy', True) == True: pass

################################################################################ enter sub antion
def do_enter_sub_actions(name):
    global ENTER_ACTION
    print name
    for i in ENTER_ACTION[name]:
        subAction(i[0],i[1],i[2])

################################################################################ exit sub antion
def do_exit_sub_actions(name):
    global EXIT_ACTION
    for i in EXIT_ACTION[name]:
        subAction(i[0],i[1],i[2])
################################################################################ get laser
def laser_callback( data):
    global ranges
    ranges = data.ranges

################################################################################ action roslib
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

    print status, result
################################################################################ get path
def get_plan (x,y):
    global robot_x_position
    global robot_y_position

    startPos = PoseStamped()
    startPos.header.frame_id = 'map'
    startPos.header.stamp = rospy.Time.now()
    startPos.pose.position.x = robot_x_position
    startPos.pose.position.y = robot_y_position

    endPos = PoseStamped()
    endPos.header.frame_id = 'map'
    endPos.header.stamp = rospy.Time.now()
    endPos.pose.position.x = x
    endPos.pose.position.y = y

    rospy.wait_for_service('/move_base/make_plan')
    planService = rospy.ServiceProxy('/move_base/make_plan' , GetPlan)
    result = planService(startPos,endPos,0.4)

    return len(result.plan.poses)

################################################################################ sort by first element
def takeFirstElement(elem):
    return elem[0]
################################################################################ do mission
@with_goto
def do_task():
    global task_index
    global robot_x_position
    global robot_y_position
    global robot_last_location
    global ranges
    global arm_publisher
    global velocity_publisher
    global GOAL_POSE
    global GOAL_ROTA
    global ENTER_ACTION
    global EXIT_ACTION
    global AREA_HEIGHT
    global TASK_LIST
    global client
    global is_nav_finished
    global navigation_status
    global shift_status
    global shift_status_put
    global stack
    global is_all_tasks_try
    global table_delay

    label .nexttask
    ############################
    print "start loop"
    if len(TASK_LIST)>0:
        canFinishTask = True
        # choose one task

        # check if all high priority task are done, then convert low priority tasks to high
        isAnyHighPriorityLeft = False
        for task in TASK_LIST:
            if task[4]==1:
                isAnyHighPriorityLeft = True
                break
        if isAnyHighPriorityLeft == False:
            for task in TASK_LIST: task[4]=1
            print "convert all task to high priority"

        # is there any task to current workstation?
        task_index = -1
        for i in range(len(TASK_LIST)):
            if TASK_LIST[i][0] == "Bolt"  or TASK_LIST[i][0] == "Large Nut":
                #heavy objecys
                if TASK_LIST[i][1] == robot_last_location and TASK_LIST[i][2]=='pick' and stack[2] == 'none' and TASK_LIST[i][4]== 1 :
                    task_index = i
                    print "there is a pick up task in current workstation " , robot_last_location
                    break
                if TASK_LIST[i][1] == robot_last_location and TASK_LIST[i][2]== 'put' and TASK_LIST[i][0] in stack and TASK_LIST[i][4]== 1 :
                    task_index = i
                    print "there is a put task in current workstation " , robot_last_location
                    break
            else:
                # light objects
                if TASK_LIST[i][1] == robot_last_location and TASK_LIST[i][2]=='pick' and 'none' in stack and TASK_LIST[i][4]== 1 :
                    task_index = i
                    print "there is a pick up task in current workstation " , robot_last_location
                    break
                if TASK_LIST[i][1] == robot_last_location and TASK_LIST[i][2]== 'put' and TASK_LIST[i][0] in stack and TASK_LIST[i][4]== 1 :
                    task_index = i
                    print "there is a put task in current workstation " , robot_last_location
                    break
        # search for nearest workstation
        if task_index == -1 :
            print "there is no task with this workstation"
            pathList= []
            for task in range(len(TASK_LIST)):
                if TASK_LIST[task][1] == robot_last_location:
                    pathList.append([-1,task])
                else:
                    pathList.append([get_plan(GOAL_POSE[TASK_LIST[task][1]][0],GOAL_POSE[TASK_LIST[task][1]][1]),task])
            print "create list of paths :" ,  pathList
            maxPath = 1000000
            for p in range(len(pathList)):
                if pathList[p][0] == -1 :  pathList[p][0] = maxPath+1
                elif  pathList[p][0] == 0 :  pathList[p][0] = maxPath
            print "converted path list is : " , pathList
            pathList.sort(key=takeFirstElement)
            print "sorted path list is : " , pathList
            for p in pathList:
                tempIndex = p[1]
                print "checking path list - index : " , tempIndex
                if TASK_LIST[tempIndex][0] == "Bolt" or TASK_LIST[tempIndex][0] == "Large Nut":
                    #heavy objects
                    if  TASK_LIST[tempIndex][2]=='pick' and stack[2]=='none' and TASK_LIST[tempIndex][4]== 1 :
                        task_index = tempIndex
                        print "new pick task is found index : " ,tempIndex
                        break
                    if TASK_LIST[tempIndex][2]== 'put' and TASK_LIST[tempIndex][0] in stack and TASK_LIST[tempIndex][4]== 1 :
                        task_index = tempIndex
                        print "new put task is found index : " ,tempIndex
                        break
                else:
                    # light objects
                    if  TASK_LIST[tempIndex][2]=='pick' and 'none' in stack and TASK_LIST[tempIndex][4]== 1 :
                        task_index = tempIndex
                        print "new pick task is found index : " ,tempIndex
                        break
                    if TASK_LIST[tempIndex][2]== 'put' and TASK_LIST[tempIndex][0] in stack and TASK_LIST[tempIndex][4]== 1 :
                        task_index = tempIndex
                        print "new put task is found index : " ,tempIndex
                        break

        if task_index == -1 :
            if is_all_tasks_try==False:
                for task in TASK_LIST: task[4]=1
                is_all_tasks_try = True
            else :
                TASK_LIST = []
            goto .nexttask
        ####################################################################
        #now do this task
        print "selected index is " , task_index
        print "task is " ,TASK_LIST[task_index]
        #arm is in safe
        table_heigh = AREA_HEIGHT[TASK_LIST[task_index][1]]
        if TASK_LIST[task_index][1] != robot_last_location:
            arm_go_to_pos('move_pos',False)
            do_exit_sub_actions(robot_last_location)

            is_nav_finished = False
            navigation_status = 0

            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            client.wait_for_server()
            workstation = TASK_LIST[task_index][1]
            pose = GOAL_POSE[workstation]
            rot = tf.transformations.quaternion_from_euler(0,0,GOAL_ROTA[workstation])

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
                while rospy.get_param('/armBussy', True) == True: pass
                arm_go_to_pos('move_pos',True)
                rospy.set_param('/vision/meterEnable',True)
                client.send_goal(goal,done_cb,active_cb,feedback_cb)
                while not is_nav_finished and not rospy.is_shutdown(): pass

            '''rospy.wait_for_service('/move_base/clear_costmaps')
            clearCostmap = rospy.ServiceProxy('/move_base/clear_costmaps' , Empty)
            clearCostmap()'''

            rospy.set_param('/vision/meterEnable',False)
            do_enter_sub_actions(TASK_LIST[task_index][1])

        if TASK_LIST[task_index][2] == 'put':
            put_teta = 2.9491
            put_teta2 = 2.9229
            put_x = 0.28
            put_y_delta = 0

            if TASK_LIST[task_index][3] == 'Blue Container' or  TASK_LIST[task_index][3] == 'Red Container' or TASK_LIST[task_index][1] == 'Precision Platform':
                while rospy.get_param('/armBussy', True) == True: pass
                arm_go_to(2.9491,0.09, table_heigh + 0.03 ,1.78845,2.9229)
                #search
                label .puttryagain
                if TASK_LIST[task_index][3] == 'Blue Container' or  TASK_LIST[task_index][3] == 'Red Container':
                    vision_publisher.publish( TASK_LIST[task_index][3])
                    put_y_delta = 0.07
                    put_teta = 2.4291 # left -> - 0.52
                    put_x = 0.28
                else :
                    vision_publisher.publish( TASK_LIST[task_index][0] + '_place')
                    put_y_delta = -0.03
                rospy.set_param('/vision/object_status',"search")
                while rospy.get_param('/vision/object_status', "search") == "search" : pass


                if ( rospy.get_param('/vision/object_status', "search") == "found" ):
                    print "object found " + TASK_LIST[task_index][0]
                    x =  rospy.get_param('/vision/object_x', 0 )
                    y =  rospy.get_param('/vision/object_y', 0 )
                    a = rospy.get_param('/vision/object_a', 0 )
                    W = 0.37
                    H = 0.28
                    DELATA_Y = 0.09
                    DELATA_Y2 = 0.035
                    x = ((x - 320) * W) / float(640)
                    y = (((480-y) *H ) / float(480) ) + DELATA_Y

                    if TASK_LIST[task_index][3] == 'Blue Container' or  TASK_LIST[task_index][3] == 'Red Container':
                        y += 0.1

                    v = math.sqrt( x**2 + y**2)
                    if x > 0 :
                        teta = math.acos( float(y) / float(v)) + 2.9491
                    else :
                        teta = 2.9491 -  math.acos( float(y) / float(v) )
                    teta2 = a + ( 2.9491 - teta)
                    v = v - DELATA_Y2

                    put_teta = teta
                    put_x = v
                    put_teta2 = teta2

                else:
                    if shift_status_put == 0:
                        subAction("moveRight",0.1,0.05)
                        shift_status_put = 1
                        goto .puttryagain
                    elif shift_status_put == 1:
                        subAction("moveRight",0.1,0.05)
                        shift_status_put = 2
                        goto .puttryagain
                    elif shift_status_put == 2:
                        subAction("moveLeft",0.3,0.05)
                        shift_status_put = 3
                        goto .puttryagain
                    elif shift_status_put == 3:
                        subAction("moveLeft",0.1,0.05)
                        shift_status_put = 4
                        goto .puttryagain
                    elif shift_status_put == 4:
                        subAction("moveRight",0.2,0.05)
                        shift_status_put = 0

            while rospy.get_param('/armBussy', True) == True: pass

            #now arm is in pick3

            rospy.set_param('/armGripperOpen',True)
            while rospy.get_param('/armGripperOpen', True) == True: pass

            if TASK_LIST[task_index][0]== 'Bolt' or TASK_LIST[task_index][0] == 'Large Nut':
                arm_go_to_pos('heavy_pic_pos',True)
                stack[2] = 'none'
                rospy.set_param('/armGripperClose',True)
                while rospy.get_param('/armGripperClose', True) == True: pass
                arm_go_to_pos('heavy_unpic_pos',True)
            else:
                arm_go_to_pos('pick3_pos',True)#totalpick_pos
                stack_index = stack.index(TASK_LIST[task_index][0])
                if stack_index == 0:
                    arm_go_to_pos('stack0_from_pick3_pos',True)
                    stack[0] = 'none'
                    rospy.set_param('/armGripperClose',True)
                    while rospy.get_param('/armGripperClose', True) == True: pass
                    arm_go_to_pos('pick3_from_stack0_pos',True)
                elif stack_index == 1:
                    arm_go_to_pos('stack1_from_pick3_pos',True)
                    stack[1] = 'none'
                    rospy.set_param('/armGripperClose',True)
                    while rospy.get_param('/armGripperClose', True) == True: pass
                    arm_go_to_pos('pick3_from_stack1_pos',True)
                elif stack_index == 2:
                    arm_go_to_pos('stack2_from_pick3_pos',True)
                    stack[2] = 'none'
                    rospy.set_param('/armGripperClose',True)
                    while rospy.get_param('/armGripperClose', True) == True: pass
                    arm_go_to_pos('pick3_from_stack2_pos',True)

                arm_go_to_pos('totalunpick_pos',True)

            if TASK_LIST[task_index][1] == 'Shelf 1' or TASK_LIST[task_index][1] == 'Shelf 2':
                subAction('wallForward',0.6,0.05)
                arm_go_to_pos('heavy_pic1_pos',True)
                arm_go_to_pos('shelf_totalput_pos',True)
                subAction('wallForward',0.35,0.05)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                subAction('wallForward',0.6,0.05)
                arm_go_to_pos('safe_pos',True)

            else :
                if TASK_LIST[task_index][3] == 'Blue Container' or  TASK_LIST[task_index][3] == 'Red Container':
                    put_teta2 = 4.3
                elif TASK_LIST[task_index][1] == 'Precision Platform':
                    #put_x = 0.28
                    pass
                arm_go_to(put_teta ,put_x, table_heigh - 0.03 + put_y_delta,3.4,put_teta2)
                rospy.sleep(1)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos('safe_pos',True)

            if shift_status_put == 1:
                subAction("moveLeft",0.1,0.05)
                shift_status_put = 0
            elif shift_status_put == 2:
                subAction("moveLeft",0.2,0.05)
                shift_status_put = 0
            elif shift_status_put == 3:
                subAction("moveRight",0.1,0.05)
                shift_status_put = 0
            elif shift_status_put == 4:
                subAction("moveRight",0.2,0.05)
                shift_status_put = 0


        elif TASK_LIST[task_index][2] == 'pick':
            if TASK_LIST[task_index][1] == 'Shelf 1' or TASK_LIST[task_index][1] == 'Shelf 2':
                pass #do_to
            elif TASK_LIST[task_index][1] == 'Rotating Table' or TASK_LIST[task_index][1] == 'Conveyor Belt':
                pass #do_to
            else:
                #scan pos
                while rospy.get_param('/armBussy', True) == True: pass
                arm_go_to(2.9491,0.09, table_heigh + 0.03 ,1.78845,2.9229)
                #search
                label .tryagain
                vision_publisher.publish(TASK_LIST[task_index][0])
                rospy.set_param('/vision/object_status',"search")
                while rospy.get_param('/vision/object_status', "search") == "search" : pass


                if ( rospy.get_param('/vision/object_status', "search") == "found" ):
                    print "object found " + TASK_LIST[task_index][0]
                    x =  rospy.get_param('/vision/object_x', 0 )
                    y =  rospy.get_param('/vision/object_y', 0 )
                    a = rospy.get_param('/vision/object_a', 0 )
                    W = 0.37
                    H = 0.28
                    DELATA_Y = 0.09
                    DELATA_Y2 = 0.035
                    x = ((x - 320) * W) / float(640)
                    y = (((480-y) *H ) / float(480) ) + DELATA_Y
                    v = math.sqrt( x**2 + y**2)
                    if x > 0 :
                        teta = math.acos( float(y) / float(v)) + 2.9491
                    else :
                        teta = 2.9491 -  math.acos( float(y) / float(v) )
                    teta2 = a + ( 2.9491 - teta)
                    v = v - DELATA_Y2

                    rospy.set_param('/armGripperOpen',True)
                    while rospy.get_param('/armGripperOpen', True) == True: pass


                    arm_go_to(teta,v, table_heigh + 0.03 , 3.35, teta2)
                    if table_delay != 0 :
                        rospy.sleep(table_delay)
                    arm_go_to(teta,v, table_heigh - 0.065, 3.35 ,teta2)
                    rospy.set_param('/armGripperClose',True)
                    while rospy.get_param('/armGripperClose', True) == True: pass

                    deltaGripper = rospy.get_param('/armGripperOffset', 0)
                    if deltaGripper > 1100 :
                        newTask = TASK_LIST[task_index]
                        newTask[4] = 0
                        TASK_LIST.append(newTask)
                        arm_go_to_pos("safe_pos",False)
                        rospy.set_param('/armGripperOpen',True)

                    else:
                        if TASK_LIST[task_index][0]=="Bolt" or TASK_LIST[task_index][0]=='Large Nut':
                            arm_go_to_pos("heavy_pic_pos",True)
                            stack[2] = TASK_LIST[task_index][0]
                            rospy.set_param('/armGripperOpen',True)
                            #while rospy.get_param('/armGripperOpen', True) == True: pass
                            arm_go_to_pos('heavy_unpic_pos',False)
                        else:

                            arm_go_to_pos("totalpick_pos",True)

                            #now arm is in pick3
                            stack_index = stack.index('none')
                            if stack_index == 0:
                                arm_go_to_pos('stack0_from_pick3_pos',True)
                                stack[0] = TASK_LIST[task_index][0]
                                rospy.set_param('/armGripperOpen',True)
                                #while rospy.get_param('/armGripperOpen', True) == True: pass
                                arm_go_to_pos('pick3_from_stack0_pos',True)
                            elif stack_index == 1:
                                arm_go_to_pos('stack1_from_pick3_pos',True)
                                stack[1] = TASK_LIST[task_index][0]
                                rospy.set_param('/armGripperOpen',True)
                                #while rospy.get_param('/armGripperOpen', True) == True: pass
                                arm_go_to_pos('pick3_from_stack1_pos',True)
                            elif stack_index == 2:
                                arm_go_to_pos('stack2_from_pick3_pos',True)
                                stack[2] = TASK_LIST[task_index][0]
                                rospy.set_param('/armGripperOpen',True)
                                #while rospy.get_param('/armGripperOpen', True) == True: pass
                                arm_go_to_pos('pick3_from_stack2_pos',True)

                            arm_go_to_pos("totalunpick_pos",False)

                    # come back to first place
                    if shift_status == 1:
                        subAction("moveLeft",0.1,0.05)
                    elif shift_status == 2:
                        subAction("moveLeft",0.2,0.05)
                    elif shift_status == 3:
                        subAction("moveRight",0.1,0.05)
                    elif shift_status == 4:
                        subAction("moveRight",0.2,0.05)
                    shift_status = 0

                    while rospy.get_param('/armBussy', True) == True: pass
                    arm_go_to_pos("safe_pos",False)


                else:
                    if shift_status == 0:
                        subAction("moveRight",0.1,0.05)
                        shift_status = 1
                        goto .tryagain
                    elif shift_status == 1:
                        subAction("moveRight",0.1,0.05)
                        shift_status = 2
                        goto .tryagain
                    elif shift_status == 2:
                        subAction("moveLeft",0.3,0.05)
                        shift_status = 3
                        goto .tryagain
                    elif shift_status == 3:
                        subAction("moveLeft",0.1,0.05)
                        shift_status = 4
                        goto .tryagain
                    elif shift_status == 4:
                        subAction("moveRight",0.2,0.05)
                        shift_status = 0
                        TASK_LIST[task_index][4] = 0
                        canFinishTask = False


        robot_last_location = TASK_LIST[task_index][1]
        if canFinishTask==True: del TASK_LIST[task_index]


    else:
        # finish and go to EXIT
        do_exit_sub_actions(robot_last_location)

        is_nav_finished = False
        navigation_status = 0

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        pose = GOAL_POSE['finish']
        rot = tf.transformations.quaternion_from_euler(0, 0, GOAL_ROTA['finish'])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        goal.target_pose.pose.orientation.z = rot[2]
        goal.target_pose.pose.orientation.w = rot[3]


        while not (navigation_status ==3):
            is_nav_finished = False
            navigation_status = 0
            while rospy.get_param('/armBussy', True) == True: pass
            arm_go_to_pos('move_pos',True)
            rospy.set_param('/vision/meterEnable',True)
            client.send_goal(goal,done_cb,active_cb,feedback_cb)
            while not is_nav_finished and not rospy.is_shutdown(): pass

        rospy.set_param('/vision/meterEnable',False)
        do_enter_sub_actions('finish')
        print "task finished"
        rospy.spin()

    goto .nexttask


################################################################################   ref box
def refCallBack_BMT1(data):

    global TASK_LIST
    global getFirstTime
    global task_index

    if getFirstTime == False :
        return

    getFirstTime = False


    for t in data.tasks:
        newTaskPick = []
        newTaskPut = []

        taskObjectName = t.transportation_task.object.description.data
        taskObjectSource = t.transportation_task.source.description.data
        taskObjectDestination = t.transportation_task.destination.description.data
        taskObjectContainer = t.transportation_task.container.description.data
        taskParity = 1

        '''if  taskObjectName == 'Small Grey Alu. Profile' or taskObjectName == 'Large Grey Alu. Profile' or taskObjectName == 'Axis' or taskObjectName == 'Bearing Box' :
            taskParity = 0'''
        if taskObjectSource == 'Workstation 9' or taskObjectSource == 'Workstation 10' or taskObjectSource == 'Workstation 1' :
            taskParity = 0



        newTaskPick.append(taskObjectName)
        newTaskPick.append(taskObjectSource)
        newTaskPick.append('pick')
        newTaskPick.append(taskObjectContainer)
        newTaskPick.append(taskParity)

        newTaskPut.append(taskObjectName)
        newTaskPut.append(taskObjectDestination)
        newTaskPut.append('put')
        newTaskPut.append(taskObjectContainer)
        newTaskPut.append(taskParity)



        if newTaskPick[1] == 'Shelf 1' or newTaskPick[1] == 'Shelf 2':
            pass
        elif newTaskPick[1] == 'Rotating Table'  or newTaskPick[1] == 'Conveyor Belt':
            pass
        elif (newTaskPut[1] == 'Shelf 1' or newTaskPut[1] == 'Shelf 2' ) and (newTaskPut[0] == 'Bolt' or newTaskPut[0] == 'Large Nut') :
            pass
        else :

            if newTaskPut[1] == 'Workstation 12':
                newTaskPut[1] = 'Workstation 11'

            if newTaskPick[1] == 'Workstation 12':
                newTaskPick[1] = 'Workstation 11'

            TASK_LIST.append(newTaskPick)
            TASK_LIST.append(newTaskPut)

    print TASK_LIST
    #do_task()


if __name__ == '__main__':

    rospy.init_node('btt_main_node', anonymous=True)
    rospy.Subscriber("/robot_example_ros/task_info", TaskInfo, refCallBack_BMT1)
    rospy.Subscriber('scan', LaserScan , laser_callback)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    arm_publisher = rospy.Publisher('/arm_go', String, queue_size=10)
    vision_publisher = rospy.Publisher('/vision/find', String, queue_size=10)
    rospy.set_param('/vision/meterEnable',False)

    getFirstTime = True

    rospy.sleep(1)

    while rospy.get_param('/armCanGo', False) == False:
        rospy.sleep(0.2)


    #arm_go_to_pos("safe1_pos",True)
    arm_go_to_pos("safe_pos", True) #False

    rospy.sleep(2)
    ###################################################################   start the application
    #TASK_LIST = [['Small Black Alu. Profile','Workstation 3','pick','',1],['Small Black Alu. Profile','Precision Platform','put','',1]]
    #do_task()

    #table_delay = 10 - 5.72 #7.465

    #rospy.spin()

    table_heigh = 0

    while rospy.get_param('/armBussy', True) == True: pass
    arm_go_to(2.9491,0.09, table_heigh + 0.03 ,1.78845,2.9229)
    #search


    vision_publisher.publish('Motor')
    rospy.set_param('/vision/object_status',"search")
    while rospy.get_param('/vision/object_status', "search") == "search" : pass


    if ( rospy.get_param('/vision/object_status', "search") == "found" ):

        rospy.loginfo('1')

        x =  rospy.get_param('/vision/object_x', 0 )
        y =  rospy.get_param('/vision/object_y', 0 )
        a = rospy.get_param('/vision/object_a', 0 )
        W = 0.37
        H = 0.28
        DELATA_Y = 0.108
        DELATA_Y2 = 0.035
        x = ((x - 320) * W) / float(640)
        y = (((480-y) *H ) / float(480) ) + DELATA_Y
        v = math.sqrt( x**2 + y**2)
        if x > 0 :
            teta = math.acos( float(y) / float(v)) + 2.9491
        else :
            teta = 2.9491 -  math.acos( float(y) / float(v) )
        teta2 = a + ( 2.9491 - teta)
        v = v - DELATA_Y2

        rospy.set_param('/armGripperOpen',True)
        while rospy.get_param('/armGripperOpen', True) == True: pass

        arm_go_to(teta,v,table_heigh+0.03,3.35,teta2)
        arm_go_to(teta,v, table_heigh - 0.065, 3.35 ,teta2)
        rospy.set_param('/armGripperClose',True)
        while rospy.get_param('/armGripperClose', True) == True: pass


        arm_go_to_pos("totalpick_pos",True)
        arm_go_to_pos('stack0_from_pick3_pos',True)
        rospy.set_param('/armGripperOpen',True)
        while rospy.get_param('/armGripperOpen', True) == True: pass
        arm_go_to_pos('pick3_from_stack0_pos',True)
        arm_go_to_pos("totalunpick_pos",False)




    print 'finished'

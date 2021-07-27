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
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from goto import with_goto
import motor


robot_x_position = 0
robot_y_position = 0
is_nav_finished = True
navigation_status = 0

client = None

task_index = 0
getFirstTime = True
velocity_publisher = None
arm_publisher = None
vision_publisher = None
shift_status = 0
ranges= []
stack= ['none','none','none']

sourceWorkStation = None
destinationWorkStation = 'Precision Platform'



GOAL_POSE ={'start'         : [ 0 , 0],
            'Workstation 5' : [ 1.834271 , -4.488999],
            }

GOAL_ROTA = {'start'         :  0 ,
            'Workstation 5' :  -1.57 ,
            }



ENTER_ACTION ={ 'Workstation 5'        :[["direction"      ,2    ,0],
                                        ["wallForward"    ,0.26 ,0.00],
                                        ["angleForward"   ,0.0  ,0.00]],

                'Workstation 5_from6'             :[["wallForward"  ,0.4    ,0.0],
                                                    ["moveRight"    ,1.2   ,0.1],
                                                    ["wallForward"  ,0.26   ,0.0],
                                                    ["angleForward" ,0.00   ,0.0]],

                'Workstation 6_from5'              :[["wallForward"  ,0.4    ,0.0],
                                                    ["moveLeft"     ,1.2    ,0.1],
                                                    ["wallForward"  ,0.26   ,0.0],
                                                    ["angleForward" ,0.00   ,0.0]],

                'finish_from6'       :[["wallForward"  ,0.4    ,0.0],
                                       ["rotateLeft"   ,1.57    ,0.5],
                                       ["moveForward"  ,2.1    ,0.2]]}

'''['Distance Tube', 'Bearing', 'Large Nut', 'Small Black Alu. Profile', 'Small Nut']
'''
OBJECTS_LIST = []

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

def arm_go_to(teta,v,h,teta4,teta5):
    global arm_publisher

    arm_publisher.publish(str(teta)+','+str(v)+','+str(h)+','+str(teta4)+','+str(teta5))
    rospy.set_param('/armBussy',True)
    while rospy.get_param('/armBussy', True) == True: pass

def arm_go_to_pos(name , wait = True):
    global arm_publisher

    while rospy.get_param('/armBussy', True) == True: pass
    arm_publisher.publish(name)
    rospy.set_param('/armBussy',True)
    if wait == True:
        while rospy.get_param('/armBussy', True) == True: pass

def do_enter_sub_actions(name):
    global ENTER_ACTION
    print name
    for i in ENTER_ACTION[name]:
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


@with_goto
def du_mission():
    global task_index
    global ranges
    global arm_publisher
    global velocity_publisher
    global GOAL_POSE
    global GOAL_ROTA
    global ENTER_ACTION
    global OBJECTS_LIST
    global shift_status
    global is_nav_finished
    global navigation_status
    global robot_x_position
    global robot_y_position
    global client
    global stack
    global sourceWorkStation
    global destinationWorkStation


    doneList = []

    table_heigh = AREA_HEIGHT[
    object_name = ""
    #exit from start
    subAction("moveForward", 1.40 , 0.2)
    subAction("rotateRight", 1.57 , 0.6)
    #   go to workstation5 from start


    is_nav_finished = False
    navigation_status = 0

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    workstation = sourceWorkStation
    pose = GOAL_POSE[workstation]
    rot = tf.transformations.quaternion_from_euler(0, 0, GOAL_ROTA[workstation])

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
        client.send_goal(goal,done_cb,active_cb,feedback_cb)
        while not is_nav_finished and not rospy.is_shutdown(): pass




    do_enter_sub_actions(sourceWorkStation)



    #for i in range(task_index):
    for i in range(3):
        label .tryagain
        #scan pos
        arm_go_to(2.9491,0.09, table_heigh +  0.03 ,1.78845,2.9229)
        #search

        for j in OBJECTS_LIST:

            if stack[2] != 'none' and ( j=='Bolt' or j=='Large Nut'):
                continue


            vision_publisher.publish(j)
            rospy.set_param('/vision/object_status',"search")
            while rospy.get_param('/vision/object_status', "search") == "search" : pass


            if (  rospy.get_param('/vision/object_status', "search") == "found" ):
                object_name = j
                break

        if (  rospy.get_param('/vision/object_status', "search") == "found" ):
            print "object found " + object_name
            x =  rospy.get_param('/vision/object_x', 0 )
            y =  rospy.get_param('/vision/object_y', 0 )
            a = rospy.get_param('/vision/object_a', 0 )

            print 'object x: ' , x , ' y:' , y

            W = 0.37
            H = 0.28
            DELATA_Y = 0.1
            DELATA_Y2 = 0.035
            x = ((x - 320) * W) / float(640)
            y = (((480-y) *H ) / float(480) ) + DELATA_Y
            v = math.sqrt( x**2 + y**2)

            print 'real x: ' , x , ' y:' , y

            if x > 0 :
                teta = math.acos( float(y) / float(v)) + 2.9491
            else :
                teta = 2.9491 -  math.acos( float(y) / float(v) )
            teta2 = a + ( 2.9491 - teta)
            v = v - DELATA_Y2


            print 'v=' , v


            rospy.set_param('/armGripperOpen',True)
            while rospy.get_param('/armGripperOpen', True) == True: pass

            arm_go_to(teta,v,table_heigh+0.03,3.4,teta2)
            arm_go_to(teta,v,table_heigh-0.065,3.4,teta2)

            rospy.set_param('/armGripperClose',True)
            while rospy.get_param('/armGripperClose', True) == True: pass

            deltaGripper = rospy.get_param('/armGripperOffset', 0)
            if deltaGripper > 1100 :
                arm_go_to(2.9491,0.09, table_heigh +  0.03 ,1.78845,2.9229)
                if shift_status == 1:
                    subAction("moveLeft",0.1,0.05)
                elif shift_status == 2:
                    subAction("moveLeft",0.2,0.05)
                elif shift_status == 3:
                    subAction("moveRight",0.1,0.05)
                elif shift_status == 4:
                    subAction("moveRight",0.2,0.05)
                shift_status = 0
                goto .tryagain


            if object_name == 'Bolt' or object_name=='Large Nut':
                arm_go_to_pos("heavy_pic_pos",True)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("heavy_unpic_pos",True)
                stack[2] = object_name
            elif stack[0] == 'none':
                arm_go_to_pos("totalpick_pos",True)
                arm_go_to_pos("stack0_from_pick3_pos",True)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("pick3_from_stack0_pos",True)
                arm_go_to_pos("totalunpick_pos",True)
                stack[0] = object_name
            elif stack[1] == 'none':
                arm_go_to_pos("totalpick_pos",True)
                arm_go_to_pos("stack1_from_pick3_pos",True)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("pick3_from_stack1_pos",True)
                arm_go_to_pos("totalunpick_pos",True)
                stack[1] = object_name
            elif stack[2] == 'none':
                arm_go_to_pos("heavy_pic_pos",True)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("heavy_unpic_pos",True)
                stack[2] = object_name


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

            print 'stack = ' , stack
            print 'done List' , doneList

            if 'none' in stack and (len(doneList) < 3):
                goto .tryagain


            do_enter_sub_actions('Workstation 6_from5')

            if stack[0] != 'none':
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("totalpick_pos",True)
                arm_go_to_pos("stack0_from_pick3_pos",True)
                rospy.set_param('/armGripperClose',True)
                while rospy.get_param('/armGripperClose', True) == True: pass
                arm_go_to_pos("pick3_from_stack0_pos",True)
                arm_go_to_pos("totalunpick_pos",True)
                arm_go_to(2.4 + (i*0.25) ,0.28, table_heigh - 0.03 ,3.5,2.9229)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("safe_pos",True)
                doneList.append(stack[0])
                stack[0] = 'none'
            if stack[1] != 'none':
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("totalpick_pos",True)
                arm_go_to_pos("stack1_from_pick3_pos",True)
                rospy.set_param('/armGripperClose',True)
                while rospy.get_param('/armGripperClose', True) == True: pass
                arm_go_to_pos("pick3_from_stack1_pos",True)
                arm_go_to_pos("totalunpick_pos",True)
                arm_go_to(2.4 + (i*0.25) ,0.28, table_heigh - 0.03 ,3.5,2.9229)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("safe_pos",True)
                doneList.append(stack[1])
                stack[1] = 'none'
            if stack[2] != 'none':
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("heavy_pic_pos",True)
                rospy.set_param('/armGripperClose',True)
                while rospy.get_param('/armGripperClose', True) == True: pass
                arm_go_to_pos("heavy_unpic_pos",True)
                arm_go_to(2.4 + (i*0.25) ,0.28, table_heigh - 0.03 ,3.5,2.9229)
                rospy.set_param('/armGripperOpen',True)
                while rospy.get_param('/armGripperOpen', True) == True: pass
                arm_go_to_pos("safe_pos",True)
                doneList.append(stack[2])
                stack[2] = 'none'


            if len(doneList) >= 5:
                do_enter_sub_actions('finish_from6')
                rospy.spin()


            do_enter_sub_actions('Workstation 5_from6')

            '''if i< (task_index-1) :
                do_enter_sub_actions('Workstation 5')
            else :
                do_enter_sub_actions('finish from_Workstation 6')'''


        else:
            if shift_status == 0:
                subAction("moveRight",0.1,0.05)
                shift_status = 1
            elif shift_status == 1:
                subAction("moveRight",0.1,0.05)
                shift_status = 2
            elif shift_status == 2:
                subAction("moveLeft",0.3,0.05)
                shift_status = 3
            elif shift_status == 3:
                subAction("moveLeft",0.1,0.05)
                shift_status = 4
            elif shift_status == 4:
                subAction("moveRight",0.2,0.05)
                shift_status = 0
            goto .tryagain



def refCallBack_BMT1(data):

    global OBJECTS_LIST
    global getFirstTime
    global task_index
    global sourceWorkStation

    if getFirstTime == False :
        return

    getFirstTime = False


    for t in data.tasks:

        OBJECTS_LIST.append(t.transportation_task.object.description.data)
        sourceWorkStation = t.transportation_task.source.description.data
        task_index +=1

    du_mission()
    print OBJECTS_LIST

if __name__ == '__main__':

    rospy.init_node('ppt_main_node', anonymous=True)
    rospy.Subscriber("/robot_example_ros/task_info", TaskInfo, refCallBack_BMT1)
    rospy.Subscriber('scan', LaserScan , laser_callback)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    arm_publisher = rospy.Publisher('/arm_go', String, queue_size=10)
    vision_publisher = rospy.Publisher('/vision/find', String, queue_size=10)
    rospy.set_param('/vision/meterEnable',False)
    rospy.set_param('/armBussy',False)


    getFirstTime = True

    rospy.sleep(1)

    while rospy.get_param('/armCanGo', False) == False:
        rospy.sleep(0.2)

    arm_go_to_pos("safe1_pos",True)
    arm_go_to_pos("safe_pos", False) # false

    rospy.sleep(1)
    ###################################################################   start the application
    #OBJECTS_LIST=["Small Gray Alu. Profile","Large Gray Alu. Profile","Bolt","Small Nut","Large Nut"]
    #task_index = 5
    #du_mission()

    #arm_go_to(2.9491,0.09, 0.016 +  0.03 ,1.78845,2.9229)

    rospy.spin()
    print 'finished'

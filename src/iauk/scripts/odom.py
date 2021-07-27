#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf
from iauk.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

br = None
odom_pub = None

pos_x = 0 #-0.10
pos_y = 0 #-6.20
pos_w =0.0

v_x = 0.0
v_y = 0.0
v_w = 0.0

currentPos_w1 =0
lastPos_w1 = 0
currentPos_w2 =0
lastPos_w2 = 0
currentPos_w3 =0
lastPos_w3 = 0
currentPos_w4 =0
lastPos_w4 = 0

isRobotMoves = False

dir1=0
lastDir1=0
dir2=0
lastDir2=0
dir3=0
lastDir3=0
dir4=0
lastDir4=0

delta_factor= (math.pi*0.1) / float(4096)

def pos_request(_id):
        try:
            rospy.wait_for_service('motor_request_odom')
            motor_req = rospy.ServiceProxy('motor_request_odom' , MotorRequest)
            result = motor_req(_id,'pos')
	except:
            print("exception")
        else:
            return result.result

def calcDeltaNew (current , last , _dir):
    if current >= last:
        delta0 = current - last
    else :
        delta0 = current + (4096 - last)
    if current <= last:
        delta1 = last - current
    else :
        delta1 = last + (4096 - current)
        
    delta = min(delta0,delta1)
    if delta == delta1:
        delta *= -1
        
    if _dir==1 :
        delta *= -1
    return delta

def calcDelta (current , last,_dir):

	if(_dir == 0):
		if(current < 1000 and last > 3000):
        		delta = current + (4096 - last)
    		else :
        		delta = current - last
	else:
		if(current > 3000 and last < 1000):
        		delta = last + (4096 - current)
    		else :
        		delta = last - current
	return delta


def updateRobotPos():
	global isRobotMoves
	global dir1
	global dir2
	global dir3
	global dir4

	global currentPos_w1
	global lastPos_w1
	global currentPos_w2
	global lastPos_w2
	global currentPos_w3
	global lastPos_w3
	global currentPos_w4
	global lastPos_w4

	global pos_x
    	global pos_y
    	global pos_w
    	global v_w
    	global v_x
    	global v_y

	global delta_factor

	lx = 0.1175
	ly = 0.16
	r = 0.05
	lFactor = 0.2713

	#if isRobotMoves==False:
	#	return
	currentPos_w1 = pos_request(2)
	currentPos_w2 = pos_request(1)
	currentPos_w3 = pos_request(4)
	currentPos_w4 = pos_request(3)

	#delta1=calcDelta(currentPos_w1,lastPos_w1,dir1)
	#delta2=calcDelta(currentPos_w2,lastPos_w2,dir2)
	#delta3=calcDelta(currentPos_w3,lastPos_w3,dir3)
	#delta4=calcDelta(currentPos_w4,lastPos_w4,dir4)

        delta1=calcDeltaNew(currentPos_w1,lastPos_w1,0)
        delta2=calcDeltaNew(currentPos_w2,lastPos_w2,1)
        delta3=calcDeltaNew(currentPos_w3,lastPos_w3,0)
        delta4=calcDeltaNew(currentPos_w4,lastPos_w4,1)
        
        #rospy.loginfo("%04d %04d %04d %04d", delta1 , delta2 , delta3 , delta4)

	delta1 *= delta_factor
	delta2 *= delta_factor
	delta3 *= delta_factor
	delta4 *= delta_factor

	#delta1 = (delta1*-1) if dir1 == 1 else delta1
	#delta2 = (delta2*-1) if dir2 == 0 else delta2
	#delta3 = (delta3*-1) if dir3 == 1 else delta3
	#delta4 = (delta4*-1) if dir4 == 0 else delta4
	

	vx = (delta1 + delta2 + delta3 + delta4) / 4
	vy = (delta3 + delta2 - delta1 - delta4) / 4
	vw = (delta2 + delta4 - delta1 - delta3) / (4 * lFactor)

        #rospy.loginfo("%04f %04f %04f", vx , vy , vw)

	lastPos_w1 = currentPos_w1
	lastPos_w2 = currentPos_w2
	lastPos_w3 = currentPos_w3
	lastPos_w4 = currentPos_w4

	pos_w += vw
	if(pos_w < (-1* math.pi)):
        	pos_w += (2*math.pi)
	if(pos_w > math.pi):
        	pos_w -= (2*math.pi)
	pos_x += ((vx * math.cos(pos_w)) - (vy * math.sin(pos_w)))
	pos_y += ((vx * math.sin(pos_w)) + (vy * math.cos(pos_w)))
	v_x = (vx / 0.15) 
	v_y = (vy / 0.15) 
	v_w = (vw / 0.15) 


def tick(event):
	global br
	global odom_pub

	updateRobotPos()

	br.sendTransform((pos_x, pos_y, 0), tf.transformations.quaternion_from_euler(0, 0, pos_w),rospy.Time.now(),"base_footprint","odom")# odom
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, pos_w)
    	odom = Odometry()
    	odom.header.stamp = rospy.Time.now()
    	odom.header.frame_id = "odom"
	odom.pose.pose = Pose(Point(pos_x, pos_y, 0.0), Quaternion(*odom_quat))
	odom.child_frame_id = "base_footprint"
    	odom.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, v_w))
	odom_pub.publish(odom)

def dir_changed_callback(data):
	data = data.data
	global isRobotMoves
	global dir1
	global lastDir1
	global dir2
	global lastDir2
	global dir3
	global lastDir3
	global dir4
	global lastDir4

	isRobotMoves = False if data=='none' else True
	updateRobotPos()
	if isRobotMoves==True:
		lastDir1,lastDir2,lastDir3,lastDir4 = dir1,dir2,dir3,dir4
		dir1 = 1 if data[0]=='1' else 0
		dir2 = 1 if data[1]=='1' else 0
		dir3 = 1 if data[2]=='1' else 0
		dir4 = 1 if data[3]=='1' else 0

if __name__ == '__main__':

	#global br
	#global currentPos_w1
	#global lastPos_w1
	#global currentPos_w2
	#global lastPos_w2
	#global currentPos_w3
	#global lastPos_w3
	#global currentPos_w4
	#global lastPos_w4
	#global odom_pub

	rospy.init_node('odom', anonymous=True)
	br = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	rospy.Subscriber('robot_new_dir', String , dir_changed_callback)
	rospy.Timer(rospy.Duration(0.15), tick)
	currentPos_w1 = lastPos_w1  = pos_request(2)
	currentPos_w2 = lastPos_w2  = pos_request(1)
	currentPos_w3 = lastPos_w3  = pos_request(4)
	currentPos_w4 = lastPos_w4  = pos_request(3)
	print currentPos_w1

	print 'odom running'
	rospy.spin()

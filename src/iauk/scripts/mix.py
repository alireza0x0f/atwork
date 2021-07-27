#!/usr/bin/env python


import rospy
import math
from geometry_msgs.msg import Twist
import motor
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3


currentTime = 0
lastTime = 0


br = None
odom_pub = None

rad_s_to_motor_factor = 0
delta_factor = (math.pi*0.1) / float(4096)

currentPos_w1 =0
lastPos_w1 = 0
currentPos_w2 =0
lastPos_w2 = 0
currentPos_w3 =0
lastPos_w3 = 0
currentPos_w4 =0
lastPos_w4 = 0

global_dir1=2
global_dir2=2
global_dir3=2
global_dir4=2

global_speed1=0
global_speed2=0
global_speed3=0
global_speed4=0

isNewSpeed=False

pos_x = 0 #-0.10
pos_y = 0 #-6.20
pos_w =0.0

v_x = 0.0
v_y = 0.0
v_w = 0.0

def calcDelta (current , last , _dir):
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



def updateRobotPos():

        global br
	global odom_pub

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

    	global currentTime
        global lastTime

	global delta_factor

	lx = 0.1175
	ly = 0.16
	r = 0.05
	lFactor = 0.2713

	currentPos_w1 = motor.getPos(2)
	currentPos_w2 = motor.getPos(1)
	currentPos_w3 = motor.getPos(4)
	currentPos_w4 = motor.getPos(3)

        delta1=calcDelta(currentPos_w1,lastPos_w1,0)
        delta2=calcDelta(currentPos_w2,lastPos_w2,1)
        delta3=calcDelta(currentPos_w3,lastPos_w3,0)
        delta4=calcDelta(currentPos_w4,lastPos_w4,1)


	delta1 *= delta_factor
	delta2 *= delta_factor
	delta3 *= delta_factor
	delta4 *= delta_factor



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

	currentTime = rospy.get_time()
	timeDelta = currentTime-lastTime
	lastTime = currentTime


	v_x = (vx / timeDelta)
	v_y = (vy / timeDelta)
	v_w = (vw / timeDelta)


        br.sendTransform((pos_x, pos_y, 0), tf.transformations.quaternion_from_euler(0, 0, pos_w),rospy.Time.now(),"base_footprint","odom")
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, pos_w)
    	odom = Odometry()
    	odom.header.stamp = rospy.Time.now()
    	odom.header.frame_id = "odom"
	odom.pose.pose = Pose(Point(pos_x, pos_y, 0.0), Quaternion(*odom_quat))
	odom.child_frame_id = "base_footprint"
    	odom.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, v_w))
	odom_pub.publish(odom)


def speed_callback(data):
	global rad_s_to_motor_factor
	global global_dir1
	global global_dir2
	global global_dir3
	global global_dir4

	global global_speed1
	global global_speed2
	global global_speed3
	global global_speed4

	global isNewSpeed

	w=data.angular.z
	vx=data.linear.x
	vy=data.linear.y

	if w> 0.7:
            w= 0.7
        elif w< -0.7:
            w= -0.7

        if vx> 0.4:
            vx= 0.4
        elif vx< -0.4:
            vx= -0.4

        if vy> 0.12:
            vy= 0.12
        elif vy< -0.12:
            vy= -0.12


        lFactor = 0.25
	lx = 0.1175
	ly = 0.16
	r = 0.05

	w1 = (1/r)*(vx-vy-(lFactor)*w)
	w2 = (1/r)*(vx+vy+(lFactor)*w)
	w3 = (1/r)*(vx+vy-(lFactor)*w)
	w4 = (1/r)*(vx-vy+(lFactor)*w)

	dir1 = 1 if w1 < 0 else 0
	dir2 = 0 if w2 < 0 else 1
	dir3 = 1 if w3 < 0 else 0
	dir4 = 0 if w4 < 0 else 1



        global_dir1=  dir1
        global_speed1 = int(abs(w1) * rad_s_to_motor_factor)
        #motor.setSpeed(2,dir1,int(abs(w1) * rad_s_to_motor_factor))
        global_dir2 = dir2
        global_speed2 = int(abs(w2) * rad_s_to_motor_factor)
        #motor.setSpeed(1,dir2,int(abs(w2) * rad_s_to_motor_factor))
        global_dir3 = dir3
        global_speed3 = int(abs(w3) * rad_s_to_motor_factor)
        #motor.setSpeed(4,dir3,int(abs(w3) * rad_s_to_motor_factor))
        global_dir4 = dir4
        global_speed4 = int(abs(w4) * rad_s_to_motor_factor)
        #motor.setSpeed(3,dir4,int(abs(w4) * rad_s_to_motor_factor))
        isNewSpeed = True



if __name__ == '__main__':

	rospy.init_node('mix', anonymous=True)
   	rospy.Subscriber('cmd_vel', Twist , speed_callback)
   	br = tf.TransformBroadcaster()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=500)

	lastTime = rospy.get_time()

	motor.ser = motor.openSerial_servo()
	motor.config()
	currentPos_w1 = lastPos_w1  = motor.getPos(2)
	currentPos_w2 = lastPos_w2  = motor.getPos(1)
	currentPos_w3 = lastPos_w3  = motor.getPos(4)
	currentPos_w4 = lastPos_w4  = motor.getPos(3)

	rad_s_to_motor_factor = ((60 / (2 * math.pi)) * (1023 / float(117)))


	#motor.test()


	print(b'main running')

	while not rospy.is_shutdown():
            updateRobotPos()
            if isNewSpeed==True:
                motor.setSpeed(2,global_dir1,global_speed1)
                motor.setSpeed(1,global_dir2,global_speed2)
                motor.setSpeed(4,global_dir3,global_speed3)
                motor.setSpeed(3,global_dir4,global_speed4)
                isNewSpeed = False

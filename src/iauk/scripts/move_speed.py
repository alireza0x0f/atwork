#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
from iauk.srv import *

pub = None
newDirPub = None
rad_s_to_motor_factor = 0


def speed_callback(data):
	global pub
	global newDirPub
	global rad_s_to_motor_factor

	w=data.angular.z
	vx=data.linear.x
	vy=data.linear.y
	
	if w> 0.3:
            w= 0.3
        elif w< -0.3:
            w= -0.3
            
        if vx> 0.08:
            vx= 0.08
        elif vx< -0.08:
            vx= -0.08

	lx = 0.1175
	ly = 0.16
	r = 0.05

	w1 = (1/r)*(vx-vy-(lx+ly)*w)
	w2 = (1/r)*(vx+vy+(lx+ly)*w)
	w3 = (1/r)*(vx+vy-(lx+ly)*w)
	w4 = (1/r)*(vx-vy+(lx+ly)*w)

	dir1 = 1 if w1 < 0 else 0
	dir2 = 0 if w2 < 0 else 1
	dir3 = 1 if w3 < 0 else 0
	dir4 = 0 if w4 < 0 else 1

	#pub.publish('wheel,2,'+str(dir1)+','+str(abs(w1) * rad_s_to_motor_factor))
	#pub.publish('wheel,1,'+str(dir2)+','+str(abs(w2) * rad_s_to_motor_factor))
	#pub.publish('wheel,4,'+str(dir3)+','+str(abs(w3) * rad_s_to_motor_factor))
	#pub.publish('wheel,3,'+str(dir4)+','+str(abs(w4) * rad_s_to_motor_factor))
	
        rospy.wait_for_service('motor_request_move_speed')
        motor_req = rospy.ServiceProxy('motor_request_move_speed' , MotorRequest)
        result = motor_req(2,str(dir1)+','+str(abs(w1) * rad_s_to_motor_factor))
        
        rospy.wait_for_service('motor_request_move_speed')
        motor_req = rospy.ServiceProxy('motor_request_move_speed' , MotorRequest)
        result = motor_req(1,str(dir2)+','+str(abs(w2) * rad_s_to_motor_factor))
        
        rospy.wait_for_service('motor_request_move_speed')
        motor_req = rospy.ServiceProxy('motor_request_move_speed' , MotorRequest)
        result = motor_req(4,str(dir3)+','+str(abs(w3) * rad_s_to_motor_factor))
        
        rospy.wait_for_service('motor_request_move_speed')
        motor_req = rospy.ServiceProxy('motor_request_move_speed' , MotorRequest)
        result = motor_req(3,str(dir4)+','+str(abs(w4) * rad_s_to_motor_factor))
        
        

	if vx==0.0 and vy==0.0 and w==0.0:
		newDirPub.publish('none')
	else :
		newDirPub.publish(str(dir1)+str(dir2)+str(dir3)+str(dir4))


if __name__ == '__main__':
	#global pub
	#global newDirPub
	#global rad_s_to_motor_factor

	rospy.init_node('move_speed', anonymous=True)
   	rospy.Subscriber('cmd_vel', Twist , speed_callback)
	#rospy.Subscriber('turtle1/cmd_vel', Twist , speed_callback)
	#rospy.Subscriber('cmd_vel_test', Twist , speed_callback)
	pub=rospy.Publisher('motor_command', String , queue_size=1000)
	newDirPub=rospy.Publisher('robot_new_dir', String , queue_size=1000)

	rad_s_to_motor_factor = ((60 / (2 * math.pi)) * (1023 / float(117)))
	#rad_s_to_motor_factor = 9768.9304
	#rad_s_to_motor_factor = rad_s_to_motor_factor / 117

	print(b'move_speed is running - new motor speed')
	print rad_s_to_motor_factor
	rospy.spin()

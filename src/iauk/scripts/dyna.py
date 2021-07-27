#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from iauk.srv import *
import motor


todo=[]
isBusFree = True
isNoRequest = True

isBusFree_2 = True


def motor_command_callback(data):
	global isBusFree_2
        while isBusFree_2==False:
            temp = "waiting"
        isBusFree_2 = False
        data = data.data.split(",")
        if(data[0] == "wheel"):
                motor.setSpeed(int(data[1]),int(data[2]),int(float(data[3])))
        elif(data[0] == "joint"):
                motor.setPos(int(data[1]),int(float(data[2])),int(float(data[3])))
        isBusFree_2=True
        


            
def motor_request_handler_odom(req):
        global isBusFree_2
        
        while isBusFree_2==False:
            temp = "waiting"
        
        isBusFree_2 = False
	if req.type == 'pos':
            result = motor.getPos(req.id)
	elif req.type == 'trq':
            result = motor.getTrq(req.id)
        isBusFree_2=True
        return result
    
    
def motor_request_handler_move_speed(req):
        global isBusFree_2
        
        while isBusFree_2==False:
            temp = "waiting"
        
        isBusFree_2 = False
        data = req.type.split(",")
        motor.setSpeed(req.id,int(data[0]),int(float(data[1])))
        isBusFree_2=True
        return 1


if __name__ == '__main__':
	rospy.init_node('dyna', anonymous=True)
   	rospy.Subscriber('motor_command', String , motor_command_callback)
	requersService_odom = rospy.Service('motor_request_odom', MotorRequest , motor_request_handler_odom)
	requersService_move_speed = rospy.Service('motor_request_move_speed', MotorRequest , motor_request_handler_move_speed)
	motor.ser = motor.openSerial_servo()
	motor.config()
	print(b'dyna is running')
	################################################  config or test motors
        #for x in range(1, 5):
        #    result = motor.check_id(x)
        #    print(result)
        #print(motor.writeParameter(7,[6,0,0,0,0]))
        #print(motor.readParameterOnce(2,3,1))
        #print(motor.readParameter(2,3,1))
	#################################################
        #motor.test()
	rospy.spin()

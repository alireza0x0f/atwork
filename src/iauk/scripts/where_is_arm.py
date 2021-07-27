#!/usr/bin/env python

import roslib
import rospy
import tf
import geometry_msgs.msg
import turtlesim.srv




if __name__ == '__main__':

    rospy.init_node('robot_tf_listener')
    listener = tf.TransformListener()
    
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #(trans,rot) = listener.lookupTransform('/base_link', '/gripper_pointer_link', rospy.Time(0))
            (trans,rot) = listener.lookupTransform('/panda_link0', '/panda_link8', rospy.Time(0))
            rot = tf.transformations.euler_from_quaternion(rot)

            print trans
            print rot
    
            #message = "echo  \\'%d\\' : [ %f , %f] >>  ~/points.txt" % (count, trans[0] , trans[1])
            #os.system("echo \'" + str(count) + "\' : [" +  str(trans[0]) + "," + str(trans[1]) +"] >> ~/points.txt" )
            
            break
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
    
    
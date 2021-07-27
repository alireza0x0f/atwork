#!/usr/bin/env python

import roslib
import rospy
import tf
import geometry_msgs.msg
import turtlesim.srv
import os




if __name__ == '__main__':

    rospy.init_node('robot_tf_listener')
    listener = tf.TransformListener()
    
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            count = rospy.get_param('/positionCount', 0)

            print count
            print trans
    
            message = "echo  \\'%d\\' : [ %f , %f] >>  ~/points.txt" % (count, trans[0] , trans[1])
            #os.system("echo \'" + str(count) + "\' : [" +  str(trans[0]) + "," + str(trans[1]) +"] >> ~/points.txt" )
            os.system(message)
            
            
            
            count+=1
            rospy.set_param('/positionCount', count)
            
            break
            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
    
    
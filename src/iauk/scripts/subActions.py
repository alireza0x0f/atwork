import rospy


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
                        
                #print angle
                if angle<-0.02:
                    vel_msg = Twist()
                    vel_msg.angular.z = 0.3
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(abs(angle)/0.3)
                    vel_msg.angular.z = 0
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(0.5)
                elif angle > 0.02 :
                    vel_msg = Twist()
                    vel_msg.angular.z = -0.3
                    velocity_publisher.publish(vel_msg)
                    rospy.sleep(abs(angle)/0.3)
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
  
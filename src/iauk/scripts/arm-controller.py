#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from iauk.srv import *
import math



robot = None
scene = None


joint_start_pose= [2.9496,0.001,-0.03,2.2438,2.9229]
joint_safe1_pose= [2.9496,0.001,-0.03,2.1,2.9229]
joint_move_pose=  [2.94963492882,0.00115868772332,-0.500011706088,2.12,2.92280803626]
joint_safe_pose=  [2.9491,0.001,-1.3228,3.57,2.9229]
# for pickup
joint_pick1_pose =  [2.94903888292 ,0.26006073828 ,-1.2600447198 ,3.28851240578 ,2.92283998936]
joint_pick2_pose =  [3.00000721792 ,2 ,-3.39994960791 ,0.500010558716, 2.922892548]
joint_pick3_pose =  [2.99993324887 ,1.99999155528 ,-4.29996362874, 0.500035235046 ,2.92292377784]

joint_pick3_0_pose =   [3.35 ,1.99999155528 ,-4.29996362874, 0.500035235046 ,2.92292377784]
joint_pick4_0_pose =   [3.35, 0.600038105193, -3.30003363932, 0.00106057444904, 2.94993399475]
joint_pick5_0_pose =   [3.35, 0.550079754086, -3.42, 0.150043790009 ,2.95007795185]

joint_pick3_1_pose =   [2.9491 ,1.99999155528 ,-4.29996362874, 0.500035235046 ,2.92292377784]
joint_pick4_1_pose =   [2.94897544649, 0.600038105193, -3.30003363932, 0.00106057444904, 2.94993399475]
joint_pick5_1_pose =   [2.94911510489, 0.550079754086, -3.42, 0.150043790009 ,2.95007795185]

joint_pick3_2_pose =   [2.5482 ,1.99999155528 ,-4.29996362874, 0.500035235046 ,2.92292377784]
joint_pick4_2_pose =   [2.5482, 0.600038105193, -3.30003363932, 0.00106057444904, 2.94993399475]
joint_pick5_2_pose =   [2.5482, 0.550079754086, -3.42, 0.150043790009 ,2.95007795185]

# for pickup heavy
joint_pickheavy1_pose =   [2.94910607319 ,1.56578960269 ,-1.50264526621 ,3.4698928975 ,2.92298871852]
joint_pickheavy2_pose =   [5.84994812742 ,1.24792638047 ,-1.4629175599 ,3.57582534747 ,2.92281580646]
joint_pickheavy3_pose =   [5.74983291068 ,1.47032676397 ,-1.10466927442 ,3.1099575771 ,2.9658582208]


joint_shelf1_pose = [2.9490958818 ,1.47142656326 ,-0.132773855543 ,1.29437108403 ,2.92296843937]
joint_shelf2_pose = [2.94917038313 ,2.53003786784 ,-1.18388987359 ,0.487015434533 ,2.9229787742]



def go_callback(data):
    global joint_start_pose
    global joint_move_pose
    global joint_scan_pose
    global joint_spike_pose

    data = data.data

    if data=="move_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_move_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="safe_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_safe_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="safe1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_safe1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick2_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick3_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick3_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick4_0_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick4_0_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick5_0_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_0_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick4_1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick4_1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick5_1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick4_2_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick4_2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick5_2_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick1_pose, wait=True)
        group.go(joint_pick2_pose, wait=True)
        group.go(joint_pick3_pose, wait=True)
        group.go(joint_pick4_pose, wait=True)
        group.go(joint_pick5_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="unpick_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_pose, wait=True)
        group.go(joint_pick4_pose, wait=True)
        group.go(joint_pick3_pose, wait=True)
        group.go(joint_pick2_pose, wait=True)
        group.go(joint_pick1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="totalunpick_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick3_pose, wait=True)
        group.go(joint_pick2_pose, wait=True)
        group.go(joint_pick1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="totalpick_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick1_pose, wait=True)
        group.go(joint_pick2_pose, wait=True)
        group.go(joint_pick3_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="stack0_from_pick3_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick3_0_pose, wait=True)
        group.go(joint_pick4_0_pose, wait=True)
        group.go(joint_pick5_0_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="stack1_from_pick3_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick3_1_pose, wait=True)
        group.go(joint_pick4_1_pose, wait=True)
        group.go(joint_pick5_1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="stack2_from_pick3_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick3_2_pose, wait=True)
        group.go(joint_pick4_2_pose, wait=True)
        group.go(joint_pick5_2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick3_from_stack0_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_0_pose, wait=True)
        group.go(joint_pick4_0_pose, wait=True)
        group.go(joint_pick3_0_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick3_from_stack1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_1_pose, wait=True)
        group.go(joint_pick4_1_pose, wait=True)
        group.go(joint_pick3_1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="pick3_from_stack2_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pick5_2_pose, wait=True)
        group.go(joint_pick4_2_pose, wait=True)
        group.go(joint_pick3_2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="heavy_pic_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pickheavy1_pose, wait=True)
        group.go(joint_pickheavy2_pose, wait=True)
        group.go(joint_pickheavy3_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="heavy_unpic_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pickheavy3_pose, wait=True)
        group.go(joint_pickheavy2_pose, wait=True)
        group.go(joint_pickheavy1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="heavy_pic1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pickheavy1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="heavy_pic2_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pickheavy2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="heavy_pic3_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_pickheavy3_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="shelf_put1_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_shelf1_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="shelf_put2_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_shelf2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="shelf_totalput_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_shelf1_pose, wait=True)
        group.go(joint_shelf2_pose, wait=True)
        rospy.set_param('/armBussy',False)
    elif data=="shelf_totalunput_pos":
        rospy.set_param('/armBussy',True)
        group.go(joint_shelf2_pose, wait=True)
        group.go(joint_shelf1_pose, wait=True)
        rospy.set_param('/armBussy',False)



    else:
        #try:
            data = data.split(",") #angle,x,y,,m3Angle,m4Angle
            angle = float(data[0])
            x = float(data[1])
            y = float(data[2])
            m3Angle = float(data[3])
            m4Angle = 5.8458 - float(data[4])
            rospy.set_param('/armBussy',True)
            l1 = 0.155
            l2 = 0.135

            while True:
                v = math.sqrt(x**2 + y**2)
                if (v > 0.28):
                    x -= 0.01
                else:
                    break


            teta2 = math.acos(((x**2)+(y**2)-(l1**2)-(l2**2))/float(2*l1*l2))
            teta1 = math.atan(((y*((l2*math.cos(teta2))+l1)) - (x*l2*math.sin(teta2)))/float((x*((l2*math.cos(teta2))+l1)) + (y*l2*math.sin(teta2))))

            if teta2 > 0:
                gamma = math.asin( (l2* math.sin(math.pi - teta2)) / float(v))
                teta1 = (2*gamma) + (teta1)
                teta2 *= -1

            m3Angle += teta1+ teta2
            # convert datas to kuka arm values
            #remove minus part
            teta1 += math.pi
            teta2 += math.pi
            # negative parts
            teta1 = (math.pi*2) - teta1
            teta2 = (math.pi*2) - teta2
            # shift offset
            joint_2 = teta1-0.44
            joint_3 = teta2-5.71
            #limitss
            if angle > 5.89 : angle = 5.89
            if angle < 0.001 : angle = 0.001

            if joint_2 > 2.7 : joint_2 = 2.7
            if joint_2 < 0.001 : joint_2 = 0.001

            if joint_3 < -5.18 : joint_3 = -5.18
            if joint_3 > -0.001 : joint_3 = -0.001

            if m3Angle > 3.57 : m3Angle= 3.57

            if m4Angle > 5.84 : m4Angle= 5.84

            print 'arm is going to go to -> ' , angle , ' ' , joint_2 , ' ' , joint_3 , ' ' , m3Angle , ' ' , m4Angle


            #move the arm


            group.go([angle,joint_2,joint_3,m3Angle,m4Angle], wait=True)
            group.stop()

            rospy.set_param('/armBussy',False)
        #except Exception, e:
            #print("arm exception "+str(e))



if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('armcontroller',anonymous=True)

    rospy.sleep(5)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    #group = moveit_commander.MoveGroupCommander("panda_arm")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    '''planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()'''


    #group.set_goal_joint_tolerance(0.01)
    #group.set_goal_orientation_tolerance(0.01)
    #group.set_goal_position_tolerance(0.01)
    #group.set_planning_time(8)



    group.go(joint_start_pose, wait=True)
    rospy.set_param('/armCanGo', True)

    rospy.Subscriber("/arm_go", String , go_callback)

    rospy.set_param('/armBussy',False)






















    ######################################### joint control
    '''joint_goal = group.get_current_joint_values()
    joint_goal[0] = 2.9496  # 0.001~5.8982
    joint_goal[1] = 0.1920  #0.001~2.7042
    joint_goal[2] = -0.9021 #-5.1826~0.001
    joint_goal[3] = 2.2356  #0~3.5769
    joint_goal[4] = 2.9229  #0~5.8458

    group.go(joint_goal, wait=True)
    group.stop()

    ########################################## position control
    current_pose = group.get_current_pose()
    print current_pose


    group.set_goal_joint_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_position_tolerance(0.01)

    print group.get_goal_joint_tolerance()
    print group.get_goal_orientation_tolerance()
    print group.get_goal_position_tolerance()
    print group.get_goal_tolerance()

    group.set_planning_time(8)
    '''

    '''pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.3 #0.4
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.3 #0.3
    ori = tf.transformations.quaternion_from_euler(3.14,0,-0.7)
    pose_goal.orientation.x = ori[0]
    pose_goal.orientation.y = ori[1]
    pose_goal.orientation.z = ori[2]
    pose_goal.orientation.w = ori[3]


    group.set_pose_target(pose_goal)
    #group.set_joint_value_target(pose_goal,True,True) #approximated
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()'''


    ##################################################### Cartesian Paths


    '''waypoints = []


    wpose = group.get_current_pose().pose
    wpose.position.x += 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += 0.2
    waypoints.append(copy.deepcopy(wpose))

    #wpose.position.y -= 0.1
    #waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(waypoints,0.03,0.0)


    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    group.execute(plan, wait=True)'''




    rospy.spin()

#!/usr/bin/env python  
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool
from multi_robot.msg import check_msg
count = 0
mode = 0

def ar_check(check_aruco):
    global  count, mode

    mode = check_aruco.data
    # print("sub",aruco_check)



def main():
    global mode, count
    rospy.init_node('aruco_move')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber('mode',Int32, ar_check)
        # print("fist",mode,count)
        
        if (mode == 2  or mode ==3) :
            print("second",mode,count)
            
            try:
                (trans,rot) = listener.lookupTransform('/map', 'tb3_0/arucopose', rospy.Time(0))
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            # print(roll, pitch, yaw)
            


            if mode == 2:
                ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw+np.pi/2)
                print("222")
                yawMatrix = np.array([
                [math.cos(yaw), -math.sin(yaw), trans[0]],
                [math.sin(yaw), math.cos(yaw), trans[1]],
                [0, 0, 1]
                ])
                wMatrix = np.array([
                    [math.cos(0), -math.sin(0), 0],
                    [math.sin(0), math.cos(0), -0.5],
                    [0, 0, 1]
                ])
            elif mode == 3:
                print("333")
                ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw-np.pi/2)
                yawMatrix = np.array([
                [math.cos(yaw), -math.sin(yaw), trans[0]],
                [math.sin(yaw), math.cos(yaw), trans[1]],
                [0, 0, 1]
                ])
                wMatrix = np.array([
                    [math.cos(0), -math.sin(0), 0],
                    [math.sin(0), math.cos(0), -0.4],
                    [0, 0, 1]
                ])
            m = np.dot(yawMatrix,wMatrix)
            x, y = m[:2, 2]
            cosine, sine = m[:2, 0]
            theta = np.arctan2(sine, cosine)
            
            nav_goal = PoseStamped()
            nav_goal.header.frame_id = 'map'
            nav_goal.pose.position.x = x
            nav_goal.pose.position.y = y
            nav_goal.pose.position.z = 0
            nav_goal.pose.orientation.x = 0
            nav_goal.pose.orientation.y = 0
            nav_goal.pose.orientation.z = oz
            nav_goal.pose.orientation.w = ow
            # print("seceond",mode)
            if mode  == 2 :
                if count == 0:
                    turtle_vel.publish(nav_goal)
                    print("fin2")
                    count += 1
            if mode == 3 :
                if count == 1 :
                    turtle_vel.publish(nav_goal)
                    print("fin3")
                    count += 1
        else:
            print("reset",mode,count)
            count = 0
        rate.sleep()

if __name__ == '__main__':
    main()
    
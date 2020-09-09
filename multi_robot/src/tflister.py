#!/usr/bin/env python  
import roslib
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from multi_robot.msg import check_msg
count = 0
aruco_check =False

def ar_check(check_aruco):
    global  count,aruco_check

    aruco_check = check_aruco.check
    # print("sub",aruco_check)


        



def main():
    global aruco_check
    rospy.init_node('turtle_tf_listener')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber('check_aruco',check_msg, ar_check)
        print("fist",aruco_check)
        if aruco_check is True:
            

            try:
                (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            print(roll, pitch, yaw)
            ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw+np.pi/2)

            yawMatrix = np.array([
                [math.cos(yaw), -math.sin(yaw), trans[0]],
                [math.sin(yaw), math.cos(yaw), trans[1]],
                [0, 0, 1]
            ])
            wMatrix = np.array([
                [math.cos(np.pi), -math.sin(np.pi), 0],
                [math.sin(np.pi), math.cos(np.pi), -0.20],
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
            print("seceond",aruco_check)
            if aruco_check is True :
                turtle_vel.publish(nav_goal)
            rate.sleep()

if __name__ == '__main__':
    main()
    
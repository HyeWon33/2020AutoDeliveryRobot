#!/usr/bin/env python  
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool
from multi_robot.msg import check_msg
count = 0
# aruco_check =False

# def ar_check(check_aruco):
#     global  count,aruco_check

#     # aruco_check = check_aruco.data
#     # print("sub",aruco_check)



def main():
    # global aruco_check
    rospy.init_node('send_mani')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher("/mani_pose", Pose, queue_size=1)
    fin_pub = rospy.Publisher("/aruco_move_fin", Bool, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        # rospy.Subscriber('/mode',Int32, ar_check)
        # print("fist",aruco_check)
       
            

        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/arucopose', rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))

        
        nav_goal = Pose()
        # nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = trans[0]
        nav_goal.pose.position.y = trans[1]
        nav_goal.pose.position.z = trans[2]
        nav_goal.pose.orientation.x = rot[0]
        nav_goal.pose.orientation.y = rot[1]
        nav_goal.pose.orientation.z = rot[2]
        nav_goal.pose.orientation.w = rot[3]
        # print("seceond",aruco_check)
        # if aruco_check  == 2 :
        turtle_vel.publish(nav_goal)
        fin_pub.publish(True)
    else:
        fin_pub.publish(False)
    rate.sleep()


if __name__ == '__main__':
    main()
    
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
    turtle_vel = rospy.Publisher("cur_mani_pose", Pose, queue_size=1)
    fin_pub = rospy.Publisher("aruco_move_fin", Bool, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        # rospy.Subscriber('/mode',Int32, ar_check)
        # print("fist",aruco_check)
       
            

        try:
            (trans,rot) = listener.lookupTransform('tb3_1/base_link', '/arucopose', rospy.Time(0))
            print("ddd")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("aaa")
            continue
    # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
        br = tf.TransformBroadcaster()
        br.sendTransform((trans[0],trans[1],trans[2]),
                        (rot[0], rot[1], rot[2], rot[3]),
                        rospy.Time.now(),
                        "b_a",
                        "tb3_1/base_link")      
        nav_goal = Pose()
        # nav_goal.header.frame_id = 'map'
        if trans[1] < 0 and trans[1] > -0.06:
            trans[1] = trans[1] - 0.03
        elif trans[1] > 0 and trans[1] < 0.015:
            trans[1] = trans[1] - 0.03
        elif trans[1] < -0.06 and trans[1] > -0.08:
            trans[1] = trans[1] + 0.02
        elif trans[1] < -0.08:
            trans[1] = trans[1]
        nav_goal.position.x = trans[0]+0.02
        nav_goal.position.y = trans[1]
        nav_goal.position.z = trans[2]+0.02

        print("x: {}  y:{}  z:{}".format(trans[0], trans[1], trans[2]))
        # if aruco_check  == 2 :
        turtle_vel.publish(nav_goal)
        fin_pub.publish(True)
    else:
        fin_pub.publish(False)
    rate.sleep()


if __name__ == '__main__':
    main()
    

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool
from mani_robot.msg import check_msg

count = 0
mode = 0

"""
mani 가 물체에 접근을 할 수 있게 mani에서의 물체 위치 전송
base_link에서 본 aruco의 위치를 얻어와서 mani를 움직이는 노드로 전송
control에는 mani에 좌표를 보냈다는 msg를 pub
"""


def start(check_aruco):
    global count, mode

    mode = check_aruco.data
    # print("sub",aruco_check)


def main():
    global mode
    rospy.init_node('send_mani')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher("cur_mani_pose", Pose, queue_size=1)
    send_mani_pub = rospy.Publisher('send_mani_pub', Bool, queue_size=1)
    # fin_pub = rospy.Publisher("fin_send_mani", Bool, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber('send_mani_pub', Bool, start, queue_size=1)
        # print("fist",aruco_check)

        if mode:
            try:
                (trans, rot) = listener.lookupTransform('tb3_1/base_link', 'tb3_1/arucopose', rospy.Time(0))
                print("ddd")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("aaa")
                continue
            # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
            nav_goal = Pose()
            # nav_goal.header.frame_id = 'map'
            # --------------
            # 메니의 미세조정
            if trans[1] < 0 and trans[1] > -0.06:
                trans[1] = trans[1] - 0.03
            elif trans[1] > 0 and trans[1] < 0.015:
                trans[1] = trans[1] - 0.03
            elif trans[1] < -0.06 and trans[1] > -0.08:
                trans[1] = trans[1] + 0.02
            elif trans[1] < -0.08:
                trans[1] = trans[1]
            nav_goal.position.x = trans[0] + 0.02
            nav_goal.position.y = trans[1]
            nav_goal.position.z = trans[2] + 0.01
            # ------------------

            print("x: {}  y:{}  z:{}".format(trans[0], trans[1], trans[2]))
            # if aruco_check  == 2 :
            turtle_vel.publish(nav_goal)
            send_mani_pub.publish(False)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

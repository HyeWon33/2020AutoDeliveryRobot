#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool, String
from lift_robot.msg import check_msg

count = 0
mode = 0

"""
처음으로 aruco 마커가 감지가 됬을 경우 map에서의 marker 좌표와
y축으로 50m 후진 transformation을 구하여 연산을 진행을하여
aruco 마커의 전방 50cm에 로봇이 위치하게 해주고
second 가 들다음으로 실행 될때는 aruco 노드의 
전방 45cm에 180도 회전하여 위치하게 해주는 노드
"""


def ar_check(check_aruco):
    global mode

    mode = check_aruco.data
    print("sub", mode)


def main():
    global mode, count
    rospy.init_node('aruco_move')
    listener = tf.TransformListener()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber('aruco_move_pub', String, ar_check)

        if mode == "first":
            try:
                (trans, rot) = listener.lookupTransform('/map', 'tb3_0/arucopose', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            # print(roll, pitch, yaw)
            ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw + np.pi / 2)
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
            send_pose(yawMatrix, wMatrix, oz, ow)

        elif mode == "second":
            try:
                (trans, rot) = listener.lookupTransform('/map', 'tb3_0/arucopose', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            # print(roll, pitch, yaw)
            print("333")
            ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw - np.pi / 2)
            yawMatrix = np.array([
                [math.cos(yaw), -math.sin(yaw), trans[0]],
                [math.sin(yaw), math.cos(yaw), trans[1]],
                [0, 0, 1]
            ])
            wMatrix = np.array([
                [math.cos(0), -math.sin(0), 0],
                [math.sin(0), math.cos(0), -0.45],
                [0, 0, 1]
            ])
            send_pose(yawMatrix, wMatrix, oz, ow)

        else:
            print("reset", mode, count)
            count = 0
        rate.sleep()


def send_pose(yawMatrix, wMatrix, oz, ow):
    turtle_vel = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    aruco_move_pub = rospy.Publisher('aruco_move_pub', String, queue_size=10)  # aruco_move.py

    m = np.dot(yawMatrix, wMatrix)
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

    turtle_vel.publish(nav_goal)
    aruco_move_pub.publish("not")
    print("fin3")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

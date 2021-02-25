#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool

count = 0
aruco_check = False


"""
aruco 마커가 감지가 됬을 경우 map에서의 marker 좌표와
180도 회전과 y축으로 45cm 후진 transformation을 구하여 연산을 진행을하여
aruco 마커의 전방 45cm에 로봇이 위치하게 해주는 노드
"""

def main():
    # aruco_check 는 aruco 유무 확인(bool)
    global aruco_check, count
    rospy.init_node('aruco_move')
    # transformlistener 불러오기
    listener = tf.TransformListener()
    # fin_pub = rospy.Publisher("aruco_move_fin", Bool, queue_size=1)
    # rate 1초
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # aruco를 감지 했는지 확인해줄 subscriber
        rospy.Subscriber('aruco_move_pub', Bool, start)
        print("fist", aruco_check)
        # 만약 aruco가 감지 되면
        if aruco_check:
            # tf에 map 과 tb3_1/arucopose 사이의 translation 과 rotation이 계산이 되는지 확인
            # error 가 생기면 반복문 다시 실행
            try:
                (trans, rot) = listener.lookupTransform('/map', 'tb3_1/arucopose', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # aruco pose 를 계산(map 과 tb3_1/arucopose 사이의 translation 과 rotation 이용)
            send_aruco_pose(trans, rot)

            rate.sleep()


# aruco subscriber callback 함수
def start(check_aruco):
    global count, aruco_check
    # 받아온 data global로 저장
    aruco_check = check_aruco.data
    # print("sub",aruco_check)


# aruco maker 위치 계산하여 pose 전달
def send_aruco_pose(trans, rot):
    # goal 좌표 전송 publisher
    turtle_vel = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    # aruco move 노드 실행 여부 전송 publisher
    aruco_move_pub = rospy.Publisher('aruco_move_pub', Bool, queue_size=1)

    ## (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
    # rotation을 오일러로 변환
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
    # print(roll, pitch, yaw)
    # yaw를 90도 회전 시켜서 쿼터니언으로 변환
    ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw + np.pi / 2)


    # yaw 메트릭스
    yaw_matrix = np.array([
        [math.cos(yaw), -math.sin(yaw), trans[0]],
        [math.sin(yaw), math.cos(yaw), trans[1]],
        [0, 0, 1]
    ])
    # 180도 회전과 y축으로 45cm
    w_matrix = np.array([
        [math.cos(np.pi), -math.sin(np.pi), 0],
        [math.sin(np.pi), math.cos(np.pi), -0.45],
        [0, 0, 1]
    ])
    m = np.dot(yaw_matrix, w_matrix)
    x, y = m[:2, 2]
    cosine, sine = m[:2, 0]
    theta = np.arctan2(sine, cosine)

    # map에서의 aruco 마터 위치 (2차원) 전송
    nav_goal = PoseStamped()
    nav_goal.header.frame_id = 'map'
    nav_goal.pose.position.x = x
    nav_goal.pose.position.y = y
    nav_goal.pose.position.z = 0
    nav_goal.pose.orientation.x = 0
    nav_goal.pose.orientation.y = 0
    nav_goal.pose.orientation.z = oz
    nav_goal.pose.orientation.w = ow
    # print("seceond", aruco_check)
    # if aruco_check  == 2 :
    turtle_vel.publish(nav_goal)
    aruco_move_pub.publish(False)  # count += 1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from mani_robot.msg import aruco_msgs
import tf
import numpy as np


"""
aruco 마커의 좌표를 받아와 rgb 좌표계에서 본 aruco 위치 tf를 구성
"""
def get_aruco(msg):
    # tf broadcaster 생성
    br = tf.TransformBroadcaster()

    # aruco 에서 받아온 rotation 값들로 quaternion 계산
    angle = np.sqrt(msg.r_x * msg.r_x + msg.r_y * msg.r_y + msg.r_z * msg.r_z)
    cosa = np.cos(angle * 0.5)
    sina = np.sin(angle * 0.5)
    qx = msg.r_x * sina / angle
    qy = msg.r_y * sina / angle
    qz = msg.r_z * sina / angle
    qw = cosa

    # tf 생성
    br.sendTransform((msg.t_x, msg.t_y, msg.t_z),
                     (qx, qy, qz, qw),
                     rospy.Time.now(),
                     "tb3_1/arucopose",
                     "tb3_1/rgb_test")


def main():
    rospy.init_node("draw_aruco_axis")
    rospy.Subscriber('aruco_msg', aruco_msgs, get_aruco)
    rospy.loginfo_once("ARUCO_OK")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

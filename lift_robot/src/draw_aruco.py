#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from lift_robot.msg import aruco_msgs
import tf
import numpy as np

"""
camara 에서 aruco 의 tf 생성
"""
def get_aruco(msg):
    br = tf.TransformBroadcaster()
    # print(msg.r_x)

    angle = np.sqrt(msg.r_x * msg.r_x + msg.r_y * msg.r_y + msg.r_z * msg.r_z)
    cosa = np.cos(angle * 0.5)
    sina = np.sin(angle * 0.5)
    qx = msg.r_x * sina / angle
    qy = msg.r_y * sina / angle
    qz = msg.r_z * sina / angle
    qw = cosa
    rospy.loginfo("dd")
    # tb3_0/camera_rgb_optical_frame는 waffle_pi에 생성 되있는 tf
    br.sendTransform((msg.t_x, msg.t_y + 0.03, msg.t_z),
                     (qx, qy, qz, qw),
                     rospy.Time.now(),
                     "tb3_0/arucopose",
                     "tb3_0/camera_rgb_optical_frame")


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

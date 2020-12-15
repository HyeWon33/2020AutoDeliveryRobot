#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from mani_robot.msg import aruco_msgs
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf
import numpy as np
import math

"""
카메라센서의 좌표에서 카메라 좌표계의 tf 생성

"""

def main():
    rospy.init_node('rgb_tset')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    # -90, 0, -90에 대한 quaternion 계산
    x, y, z, w = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)
    while not rospy.is_shutdown():
        rospy.loginfo_once("rgb_OK")
        # sendTransform( translation, quaternion, 현제시간, 생성할 tf 이름, 부모 tf 이름)
        br.sendTransform((0, 0, 0),
                         (x, y, z, w),
                         rospy.Time.now(),
                         "tb3_1/rgb_test",
                         "tb3_1/cam_test")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

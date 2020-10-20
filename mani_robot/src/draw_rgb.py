#!/usr/bin/env python
import rospy
from mani_robot.msg import aruco_msgs
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf
import numpy as np
import math



def main():
    rospy.init_node('rgb_tset')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    x,y,z,w = tf.transformations.quaternion_from_euler(-np.pi/2,0,-np.pi/2)
    while not rospy.is_shutdown():
        rospy.loginfo_once("rgb_OK")
        br.sendTransform((0,0,0),
                         (x,y,z,w),
                         rospy.Time.now(),
                         "tb3_1/rgb_test",
                         "tb3_1/cam_test")
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


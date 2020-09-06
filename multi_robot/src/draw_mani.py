#!/usr/bin/env python
import rospy
from multi_robot.msg import aruco_msgs
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf
import numpy as np

# import util.util_funcs as uf


def get_mani_pose( msg):

    # self.mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]])
    # self.mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                rospy.Time.now(),
                "manipose",
                "base_link")


def main():
    rospy.init_node("draw_aruco_axis")
    listener = tf.TransformListener()
    
    # while not rospy.is_shutdown():
    #     try:
    #       (trans,rot) = listener.lookupTransform('/map', '/link5', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    rospy.Subscriber('mani_pose', Pose, get_mani_pose)
        # br = tf.TransformBroadcaster()
        # br.sendTransform((trans[0]+0.5, trans[1], trans[2]+0.05),
        #             (rot[0], rot[1], rot[2], rot[3]),
        #             rospy.Time.now(),
        #             "mani_pose",
        #             "link5")
    rospy.spin()

if __name__ == '__main__':
    main()

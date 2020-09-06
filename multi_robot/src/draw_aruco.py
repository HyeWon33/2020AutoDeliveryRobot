#!/usr/bin/env python
import rospy
from multi_robot.msg import aruco_msgs
import tf
import numpy as np

# import util.util_funcs as uf


def get_aruco( msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.t_z, -msg.t_x, -msg.t_y),
                    tf.transformations.quaternion_from_euler(msg.r_y-np.pi, -msg.r_x, -msg.r_z-np.pi),
                    rospy.Time.now(),
                    "arucopose",
                    "link5")
def main():
    rospy.init_node("draw_aruco_axis")
    rospy.Subscriber('aruco_msg', aruco_msgs, get_aruco)

    rospy.spin()

if __name__ == '__main__':
    main()

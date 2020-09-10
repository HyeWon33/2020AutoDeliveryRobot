#!/usr/bin/env python
import rospy
from multi_robot.msg import aruco_msgs
import tf
import numpy as np

def get_aruco( msg):
    br = tf.TransformBroadcaster()
    print(msg.r_x)
    
    angle = np.sqrt(msg.r_x * msg.r_x + msg.r_y * msg.r_y + msg.r_z * msg.r_z)
    cosa = np.cos(angle * 0.5)
    sina = np.sin(angle * 0.5)
    qx = msg.r_x * sina / angle
    qy = msg.r_y * sina / angle
    qz = msg.r_z * sina / angle
    qw = cosa

    br.sendTransform((msg.t_x, msg.t_y, msg.t_z),
                    (qx, qy, qz, qw),
                    rospy.Time.now(),
                    "arucopose",
                    "rgb_test")
def main():
    rospy.init_node("draw_aruco_axis")
    rospy.Subscriber('aruco_msg', aruco_msgs, get_aruco)
    rospy.loginfo_once("ARUCO_OK")
    rospy.spin()

if __name__ == '__main__':
    main()

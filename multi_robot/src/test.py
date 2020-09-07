#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import math

x,y,z,w = tf.transformations.quaternion_from_euler(-np.pi/3,np.pi/4,-np.pi/6)
print(x,y,z,w)

x,y,z,w = tf.transformations.quaternion_from_euler(-np.pi/6,np.pi/4,-np.pi/3)
print(x,y,z,w)


angle = np.sqrt(-np.pi/3 * -np.pi/3 + np.pi/4 * np.pi/4 + -np.pi/6 * -np.pi/6)
cosa = np.cos(angle * 0.5)
sina = np.sin(angle * 0.5)
qx = -np.pi/3 * sina / angle
qy = np.pi/4 * sina / angle
qz = -np.pi/6 * sina / angle
qw = cosa

print(qx,qy,qz,qw)

def get_aruco( msg):
    br = tf.TransformBroadcaster()
    x,y,z,w = tf.transformations.quaternion_from_euler(msg.r_x,msg.r_y,msg.r_z)
    print("x,y,z qua",x,y,z,w)

    x,y,z,w = tf.transformations.quaternion_from_euler(msg.r_z,msg.r_y,msg.r_x)
    print("z,y,x qua",x,y,z,w)
    angle = np.sqrt(msg.r_x * msg.r_x + msg.r_y * msg.r_y + msg.r_z * msg.r_z)
    cosa = np.cos(angle * 0.5)
    sina = np.sin(angle * 0.5)
    qx = msg.r_x * sina / angle
    qy = msg.r_y * sina / angle
    qz = msg.r_z * sina / angle
    qw = cosa

    print("test:", qx, qy, qz, qw)
    br.sendTransform((msg.t_x, msg.t_y, msg.t_z),
                    (qx, qy, qz, qw),
                    rospy.Time.now(),
                    "arucopose",
                    "rgb_test")
def main():
    rospy.init_node("draw_aruco_axis")
    rospy.Subscriber('aruco_msg', aruco_msgs, get_aruco)

    rospy.spin()

if __name__ == '__main__':
    main()
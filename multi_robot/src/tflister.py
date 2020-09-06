#!/usr/bin/env python  
import roslib

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print("trans,rot",trans,rot)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        qu = tf.transformations.quaternion_from_euler(0, 0, yaw)
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = trans[0]-0.2
        nav_goal.pose.position.y = trans[1]
        nav_goal.pose.position.z = trans[2]
        nav_goal.pose.orientation.x = qu[0]
        nav_goal.pose.orientation.y = qu[1]
        nav_goal.pose.orientation.z = qu[2]
        nav_goal.pose.orientation.w = qu[3]

        # print("angular,linear",angular,linear)
        turtle_vel.publish(nav_goal)

        rate.sleep()
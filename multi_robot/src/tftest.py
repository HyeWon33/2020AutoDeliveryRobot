#!/usr/bin/env python  
import roslib

import rospy
from nav_msgs.msg import Odometry
import tf
import turtlesim.msg

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "testpose",
                     "link5")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('odom', Odometry, handle_turtle_pose)
    # rospy.Subscriber('/%s/pose' % turtlename,
    #                  turtlesim.msg.Pose,
    #                  handle_turtle_pose,
    #                  turtlename)
    rospy.spin()

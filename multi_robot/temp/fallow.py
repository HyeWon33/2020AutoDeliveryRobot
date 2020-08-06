#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import random
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

GOAL_POSE = [[2.0, 1.0, -1.0, 1.0],
             [2.0, -2.0, 1.0, 1.0],
             [0.0, -1.0, 0.0, 1.0],
             [-1.0, 0.0, 0.0, 1.0]]


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("/tb3_1/move_base_simple/goal", PoseStamped, queue_size=10)
        self.nav_goal = PoseStamped()
        self.start_sum = rospy.Subscriber("/start", Int32, self.callback)
        self.odom_sub = rospy.Subscriber('tb3_0/odom', Odometry, self.callback2)
        self.mode = 0

    def callback(self, msg):
        self.mode = msg.data

    def publish_middle_goal(self):

        self.nav_pub.publish(self.nav_goal)
        rospy.loginfo("Sdfsdf")

    def callback2(self, msg):
        # print(msg.pose.pose)
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = msg.pose.pose.position.x
        self.nav_goal.pose.position.y = msg.pose.pose.position.y
        self.nav_goal.pose.orientation.z = msg.pose.pose.orientation.z
        self.nav_goal.pose.orientation.w = msg.pose.pose.orientation.w
        rospy.loginfo("ddd")


def main():
    rospy.init_node("follower")
    rp = Random_Pose()
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        rp.publish_middle_goal()
        r.sleep()



if __name__ == '__main__':
    main()

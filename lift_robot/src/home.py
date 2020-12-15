#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32, Bool
import random

"""
적제지로 돌아가는 좌표 전송 노드
"""


# 상황에 맞게 변경 필요
GOAL_POSE = [0.7, 0.7, 1, 0]


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.nav_goal = PoseStamped()
        self.go_home_pub = rospy.Publisher('go_home', Bool, queue_size=1)

        self.start = 0

    def Start(self, msg):
        self.start = msg.data

    def publish_middle_goal(self):
        rospy.Subscriber('go_home', Bool, self.Start)

        if self.start:
            pose = GOAL_POSE
            # rospy.Subscriber('/mode',Int32,self.Start)
            self.nav_goal.header.frame_id = 'map'
            self.nav_goal.pose.position.x = pose[0]
            self.nav_goal.pose.position.y = pose[1]
            self.nav_goal.pose.orientation.z = pose[2]
            self.nav_goal.pose.orientation.w = pose[3]
            print("ddd", self.start)

            self.nav_pub.publish(self.nav_goal)
            self.go_home_pub.publish(False)


def main():
    rospy.init_node("home")
    rp = Random_Pose()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rp.publish_middle_goal()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

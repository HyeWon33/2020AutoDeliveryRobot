#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32, Bool
import random

"""
랜덤하게 좌표를 전송하여 로봇을 움직이는 노드
GOAL_POSE에 저장된 좌표(position(x,y), oriantation(z,w))
GOAL_POSE의 data를 랜덤하게 섞은 후 첫번째 data를 사용
map이 클수록 여러개의 data를 사용 가능

"""

class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.fin_arive = rospy.Publisher("random_fin", Bool, queue_size=10)
        self.nav_goal = PoseStamped()
        self.count = 0
        self.a = 0
        self.goal_status = 0
        self.start = 0

    def random_pose(self, rate):

        rospy.Subscriber('random_dst_pub', Bool, self.Start)

        if self.start:
            # 상황에 맞게 변경 필요
            GOAL_POSE = [[0.2, -0.87, 0.92, -0.08],
                         [0.28, 0.7, -0.7, 0.70]]

            random.shuffle(GOAL_POSE)
            # print(GOAL_POSE)
            pose = GOAL_POSE[0]
            # print(pose)
            # rospy.Subscriber('/mode',Int32,self.Start)
            self.nav_goal.header.frame_id = 'map'
            self.nav_goal.pose.position.x = pose[0]
            self.nav_goal.pose.position.y = pose[1]
            self.nav_goal.pose.orientation.z = pose[2]
            self.nav_goal.pose.orientation.w = pose[3]
            rate.sleep()
            self.nav_pub.publish(self.nav_goal)
            rate.sleep()
        elif not self.start:
            print("st !0")

            self.count = 0

    def Start(self, msg):
        self.start = msg.data


def main():
    rospy.init_node("ranbom_pose")
    rp = Random_Pose()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rp.random_pose(rate)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

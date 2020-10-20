#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32, Bool
import random


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.fin_arive = rospy.Publisher("random_fin", Bool, queue_size=10)
        self.nav_goal = PoseStamped()
        self.start = 0

    def publish_middle_goal(self, rate):

        rospy.Subscriber('randome_dst_pub', Bool, self.Start)

        if self.start:
            print("ddd")
            GOAL_POSE = [[0.0, 0.8, -1, 1],
                         [-0.03, -0.7, 0.9, 0.4],
                         ]

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

    def Start(self, msg):
        self.start = msg.data



def main():
    rospy.init_node("ranbom_pose")
    rp = Random_Pose()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rp.publish_middle_goal(rate)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

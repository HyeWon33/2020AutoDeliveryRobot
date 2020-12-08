#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import random

GOAL_POSE = [[ 2.0,  1.0, -1.0, 1.0],
			 [ 2.0, -2.0,  1.0, 1.0],
			 [ 0.0, -1.0,  0.0, 1.0],
             [-1.0,  0.0,  0.0, 1.0]]


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("tb3_0/move_base_simple/goal",PoseStamped,queue_size=10)
        self.nav_goal = PoseStamped()
        # self.nav_pub2 = rospy.Publisher("tb3_1/move_base_simple/goal", PoseStamped, queue_size=10)
        # self.nav_goal2 = PoseStamped()
        self.rvecs_sub = rospy.Subscriber('tb3_0/move_base/result', MoveBaseActionResult, self.callback)
        self.a = 0
    def publish_middle_goal(self):
        pose = random.choice(GOAL_POSE)
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = pose[0]
        self.nav_goal.pose.position.y = pose[1]
        self.nav_goal.pose.orientation.z = pose[2]
        self.nav_goal.pose.orientation.w = pose[3]


        self.nav_pub.publish(self.nav_goal)


    def callback(self, data):
        self.goal_status = data.status.status
        print(self.goal_status)
        if (self.goal_status == 3 or self.goal_status == 4):
            print("goal reached")
            self.a = 1
        else :
            self.a = 0


def main():
    rospy.init_node("send_random_pose")
    rp = Random_Pose()
    r = rospy.Rate(0.3)
    while not rospy.is_shutdown():
        rp.publish_middle_goal()
        r.sleep()






if __name__ == '__main__':
    main()

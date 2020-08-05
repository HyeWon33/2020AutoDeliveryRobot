#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32
import random

GOAL_POSE = [ 0,  0, -1.0, 1.0]


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("tb3_1/move_base_simple/goal",PoseStamped,queue_size=10)
        self.nav_goal = PoseStamped()
        self.ddd = rospy.Subscriber('/start',Int32,self.Start)
        self.a = 0
        self.start = 0

    def Start(self, msg):
        self.start = msg.data


    def publish_middle_goal(self):
        pose = GOAL_POSE
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = pose[0]
        self.nav_goal.pose.position.y = pose[1]
        self.nav_goal.pose.orientation.z = pose[2]
        self.nav_goal.pose.orientation.w = pose[3]

        if self.start ==1:
            self.nav_pub.publish(self.nav_goal)



def main():
    rospy.init_node("home")
    rp = Random_Pose()
    r = rospy.Rate(0.3)
    while not rospy.is_shutdown():
        rp.publish_middle_goal()
        r.sleep()






if __name__ == '__main__':
    main()

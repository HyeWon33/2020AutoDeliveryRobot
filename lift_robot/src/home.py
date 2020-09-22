#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32, Bool
import random

GOAL_POSE = [ 2,  -0.5, 0, 1.0]


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("/tb3_0/move_base_simple/goal",PoseStamped,queue_size=10)
        self.fin_arive = rospy.Publisher("/arrived_mani",Bool,queue_size=10)
        self.nav_goal = PoseStamped()
        
        self.a = 0
        self.start = 0

    def Start(self, msg):
        self.start = msg.data
        print("dddsd")
        rospy.loginfo(self.start)
        self.publish_middle_goal()


    def publish_middle_goal(self):
        pose = GOAL_POSE
        # rospy.Subscriber('/mode',Int32,self.Start)
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = pose[0]
        self.nav_goal.pose.position.y = pose[1]
        self.nav_goal.pose.orientation.z = pose[2]
        self.nav_goal.pose.orientation.w = pose[3]
        print("ddd",self.start)

        if self.start == 6:
            
            self.nav_pub.publish(self.nav_goal)
            self.fin_arive.publish(True)
        elif self.start != 6:
            self.fin_arive.publish(False)


def main():
    rospy.init_node("home")
    rp = Random_Pose()
    rospy.Subscriber('/mode', Int32, lambda x: rp.Start(x))
    rospy.spin()





if __name__ == '__main__':
    main()

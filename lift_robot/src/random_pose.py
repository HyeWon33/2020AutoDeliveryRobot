#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32, Bool
import random



class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
        self.fin_arive = rospy.Publisher("arrived_mani",Bool,queue_size=10)
        self.nav_goal = PoseStamped()
        self.count = 0
        self.a = 0
        self.goal_status =0
        self.start = 0

    def Start(self, msg):
        self.start = msg.data
        # print("dddsd")
        # rospy.loginfo(self.start)
        # self.publish_middle_goal()

    def GoalPoseCallback(self, data):
        # rospy.loginfo(self.start)
        self.goal_status = data.status.status
        # if (self.goal_status == 3 or self.goal_status == 4) and (self.mode == "marker_waiting_position"):
        #     # print("goal reached")
    def sub(self):
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
        rospy.Subscriber('mode', Int32, self.Start)

    def publish_middle_goal(self,rate):
        
        self.sub()
        if self.start == 0:
            GOAL_POSE = [[ 2.1,  -0.0, -0.5, 0.8],
            [ 0.5,  -0.9, 0, 1.0],
            [ 2.6,  -0.58, -0.48, 0.87],
            [ 0.62,  -0.352, 0.9, 0.4]]

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
            # print("ddd",self.start)
            if self.count == 0:
 
                self.goal_status = 3 
                self.count = 1
                rate.sleep()

            elif self.count != 0:
                print("c !0")

                if (self.goal_status == 3 or self.goal_status ==4):
                    print("g 3")
                    self.nav_pub.publish(self.nav_goal)
                    self.fin_arive.publish(True)
                    self.count = 1
                    self.goal_status = 0 
                    rate.sleep()

                
                

        elif self.start != 0:
            print("st !0")
            self.fin_arive.publish(False)
            self.count = 0


def main():
    rospy.init_node("ranbom_pose")
    rp = Random_Pose()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # rospy.loginfo_once("CONTROL TOWER")
        rp.publish_middle_goal(rate)
        rate.sleep()





if __name__ == '__main__':
    main()

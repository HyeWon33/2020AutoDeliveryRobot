#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from multi_robot.msg import check_msg

class Control:
    def __init__(self):
        rospy.init_node("control_tower")
        self.mode_pub = rospy.Publisher('mode', Int32, queue_size=10)
        self.drive_pub = rospy.Publisher('drive', Int32, queue_size=10)


    def sub_msgs(self):
        rospy.Subscriber("check_aruco", check_msg, self.check)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.move_finish)               # robot이 aruco marker앞에 이동 완료

    def check(self, check_aruco):
        self.aruco_check = check_aruco.check

    def move_finish(self, finish):
        self.move_check = 'sucess'
        
    
    def main(self):
        mode = 0
        check_count = 0
        while not rospy.is_shutdown():
            rospy.loginfo_once("CONTROL TOWER")
            self.sub_msgs()
            if mode == 0:  # self drive mode
                if self.aruco_check == True:
                    mode = 1
                elif self.aruco_check == False:
                    mode = 0

            if mode == 1:  # aruco check mode
                if self.aruco_check == True:
                    check_count += 1
                    if check_count > 20:
                        mode = 2
                else:
                    mode = 0
                    check_count = 0

            if mode == 2:  # aruco marker로 접근
                if self.move_check == 'sucess':
                    mode = 3
                
            if mode == 3:   # manipulator 움직이기
                pass
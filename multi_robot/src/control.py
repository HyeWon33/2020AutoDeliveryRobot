#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from multi_robot.msg import check_msg

class Control():
    def __init__(self):
        
        self.mode_pub = rospy.Publisher('/mode', Int32, queue_size=1) # mode 바꾸기
        # self.drive_pub = rospy.Publisher('/drive', Int32, queue_size=1) # 
        self.mode = Int32()
        self.mode.data = 0
        self.aruco_check = False
        # self.self_pub = rospy.Publisher('/self_drive_on_off',Int32,queue_size=1)
        # self.stop_move = Int32()
        # self.aruco_move = rospy.Publisher('start_move_close',Int32, queue_size=1)
        self.check_count = 0


    def sub_msgs(self):
        rospy.Subscriber('/tb3_1/check_aruco',check_msg, self.ar_check)
        rospy.Subscriber("/aruco_move_fin", Bool, self.ar_fin)               # robot이 aruco marker앞에 이동 완료

    def ar_check(self, check_aruco):
        self.aruco_check = check_aruco.check

    def ar_fin(self, finish):
        self.aruco_fin = finish.data
        
    
    def control(self):
        rospy.loginfo_once("CONTROL TOWER : OK")
        rospy.loginfo_throttle(30, self.mode.data)
        
        print(self.check_count)
        
        self.sub_msgs()
        if self.mode.data == 0:  # self drive mode
            if self.aruco_check == True:
                self.mode.data = 1
            

        if self.mode.data == 1:  # aruco check mode
            if self.aruco_check == True:
                self.check_count += 1
                
                if self.check_count > 20:
                    self.mode.data = 2
            else:
                self.mode.data = 0
                self.check_count = 0

        if self.mode.data == 2:  # aruco marker로 접근
            if self.aruco_fin == True:
                self.mode.data = 3
            
        if self.mode.data == 3:   # manipulator 움직이기
            pass


        # self.self_pub.publish(self.stop_move)
        self.mode_pub.publish(self.mode)
        # self.aruco_move.publish(self.move)


def main():
    rospy.init_node("contol_tower")
    con = Control()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo_once("CONTROL TOWER")
        con.control()
        rate.sleep()
        






if __name__ == "__main__":
    main()
    
    

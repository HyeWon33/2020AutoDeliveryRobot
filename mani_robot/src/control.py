#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from mani_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID


class Control():
    def __init__(self):
        self.goal_cancel = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)   #cancel goal
        self.random_dst_pub = rospy.Publisher('randome_dst_pub', Bool, queue_size=1)   #random_pose.py 
        self.aruco_move_pub = rospy.Publisher('aruco_move_pub', Bool, queue_size=1)    #aruco_move.py
        self.move_close_pub = rospy.Publisher('start_move_closed', String, queue_size=1) #move_close.py
        self.send_mani_pub = rospy.Publisher('send_mani_pub', Bool, queue_size=1)      #send_mani.py
        self.mani_pick_up_pub = rospy.Publisher('pick_up',Bool,queue_size=1)           #test_mani
        self.go_home_pub = rospy.Publisher('go_home',Bool,queue_size=1)                #home.py
        self.pick_down_pub = rospy.Publisher('arrive_home',Bool,queue_size=1)          #test_mani
        
        self.mode = "patrol"
        self.aruco_check = False
        self.goal_status = 0
        self.check_count_t = 0
        self.check_count_f = 0
        self.close_fin = False
        self.pick_up_fin = False
        self.mani_fin = False
        self.mani_error = False
        self.mani_error_fin=False

    def control(self):
        # rospy.loginfo_once("CONTROL TOWER : OK")

        # patrol 
        rate = rospy.Rate(10)
        rate.sleep()
        # rospy.loginfo("goal : %s",self.goal_status)
        if self.mode == "patrol":  # self drive mode
            rospy.loginfo("mode : %s",self.mode)
            self.mani_error = False

            self.random_dst_pub.publish(True)  # random_pose.py 동작
            rate.sleep()
            self.mode = "find_marker"
            # self.goal_status = 0

        # find_marker  마커 확인하기
        if self.mode == "find_marker":
            # rospy.loginfo("mode : %s",self.mode)
            self.random_dst_pub.publish(False)  # random_pose.py 멈춤
            rospy.Subscriber('check_aruco', check_msg, self.ar_check)  # marker 검출여부 받기    #5 aruco_detect.py
            rate.sleep()
            if self.aruco_check:
                self.goal_cancel.publish()
                self.mode = "wait_count"  # marker 검출이 한번이라도 되면
            elif not self.aruco_check:
                self.mode = "check_finish_goal"

        if self.mode == "check_finish_goal":
            rospy.loginfo("mode : %s",self.mode)
            rospy.loginfo("goal : %s",self.goal_status)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  #2 randome_pose.py goal
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "patrol"
            else:
                self.mode = "find_marker"
            rate.sleep()

        if self.mode == "wait_count":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber('check_aruco', check_msg, self.ar_check)  #5 aruco_detect.py
            if self.aruco_check:
                self.check_count_t += 1
                if self.check_count_t > 20:
                    self.mode = "aruco_dst"
                    self.check_count_t = 0

            elif not self.aruco_check:
                self.check_count_f += 1
                if self.check_count_f > 20:
                    self.mode = "patrol"
                    self.check_count_f = 0
            rate.sleep()

        if self.mode == "aruco_dst":
            rospy.loginfo("mode : %s",self.mode)
            # for n in range(5):
            self.aruco_move_pub.publish(True)  # aruco_move.py
            self.mode = "wait_arrive_aruco"
            rate.sleep()
            

        if self.mode == "wait_arrive_aruco":
            rospy.loginfo("mode : %s",self.mode)
            
            # self.aruco_move_pub.publish(False) 
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  #2 aruco_move,py goal
            # self.aruco_move_pub.publish(False)  # aruco_move.py
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "move_closed_aruco"
                self.goal_status = 0
            rate.sleep()

        if self.mode == "move_closed_aruco":
            rospy.loginfo("mode : %s",self.mode)
            self.aruco_move_pub.publish(False)  # aruco_move.py
            self.move_close_pub.publish("front")  # mode_close.py
            self.mode = "wait_fin_closed"
            rate.sleep()

        if self.mode == "wait_fin_closed":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)   #4 move_close.py
            # self.move_close_pub.publish("not")  # mode_close.py
            if self.close_fin:
                self.mode = "send_mani_dst"
                self.close_fin = False
                self.move_close_pub.publish("not")  # mode_close.py
            rate.sleep()


        if self.mode == "send_mani_dst":
            rospy.loginfo("mode : %s",self.mode)
            # self.move_close_pub.publish(0)# mode_close.py
            self.send_mani_pub.publish(True)
            self.mode = "wait_pick_up"
            rate.sleep()

        if self.mode == "wait_pick_up":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("fin_pick_up", Bool, self.fin_pick)  #3 test_mani
            rospy.Subscriber("mani_plan_status", Bool, self.mani_er)  #3 test_mani
            rospy.loginfo("mani_error : %s",self.mani_error)
            
            rate.sleep()

            if self.pick_up_fin == True:
                self.mode = "go_home"
                self.pick_up_fin= False
                self.send_mani_pub.publish(False)
                rate.sleep()

            elif self.mani_error == True:
                self.mode = "patrol"
                self.mani_error = False
                self.send_mani_pub.publish(False)
                rate.sleep()

        # if self.mode == "wait_mani_return":

        #     rospy.loginfo("mode : %s",self.mode)
        #     rospy.Subscriber("mani_error_plan", Bool, self.mani_er_fin)  #3 test_mani
        #     self.send_mani_pub.publish(False)
        #     if self.mani_error_fin == True:
        #         self.mode = "patrol"
        #         self.mani_error_fin = False
        #         rate.sleep()                


        if self.mode == "go_home":
            rospy.loginfo("mode : %s",self.mode)
            self.send_mani_pub.publish(False)
            # for n in range(5):
            self.go_home_pub.publish(True)
            self.mode = "wait_arrive_home"
            rate.sleep()

        if self.mode == "wait_arrive_home":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback) #2 home_goal
              # aruco_move.py
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "move_back"
                self.goal_status = 0
            rate.sleep()

        if self.mode == "move_back":
            rospy.loginfo("mode : %s",self.mode)
            
            self.move_close_pub.publish("back")  # mode_close.py
            self.mode = "move_back_fin"
            rate.sleep()

        if self.mode == "move_back_fin":
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)
            print(self.close_fin)
            if self.close_fin:
                self.mode = "pick_down_mani"
                self.close_fin = False
                self.move_close_pub.publish("not")# mode_close.py
                

        if self.mode =="pick_down_mani":
            rospy.loginfo("mode : %s",self.mode)
            self.go_home_pub.publish(False)
            self.pick_down_pub.publish(True)
            self.mode = "wait_pick_down"
            rate.sleep()

        if self.mode == "wait_pick_down":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("fin_mani", Bool, self.fin_mani)   #1 test_mani
            if self.mani_fin:
                self.mode = "move_front"
            rate.sleep()

        if self.mode == "move_front":
            rospy.loginfo("mode : %s",self.mode)
            
            self.move_close_pub.publish("front")  # mode_close.py
            self.mode = "move_front_fin"
            rate.sleep()


        if self.mode == "move_front_fin":
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)
            print(self.close_fin)
            if self.close_fin:
                self.mode = "patrol"
                self.close_fin = False
                self.move_close_pub.publish("not")# mode_close.py           

    def mani_er_fin(self, error_fin):
        self.mani_error_fin = error_fin.data


    def ar_check(self, check_aruco):  #5
        self.aruco_check = check_aruco.check

    def fin_move_close(self, fin):   #4
        self.close_fin = fin.data

    def fin_pick(self, data):     #3
        self.pick_up_fin = data.data

    def GoalPoseCallback(self, data):  #2
        print("goal:", data.status.status)
        self.goal_status = data.status.status       

    def fin_mani(self, msg):   #1
        self.mani_fin = msg.data

    def mani_er(self, msg):
        self.mani_error = msg.data


def main():
    rospy.init_node("contol_tower")
    con = Control()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        con.control()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool, UInt16, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from lift_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID


class Control():
    def __init__(self):
        self.goal_cancel = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)   #cancel goal
        self.random_dst_pub = rospy.Publisher('randome_dst_pub', Bool, queue_size=10)   #random_pose.py 
        self.aruco_move_pub = rospy.Publisher('aruco_move_pub', String, queue_size=10)    #aruco_move.py
        self.move_close_pub = rospy.Publisher('start_move_closed', String, queue_size=10) #move_close.py
        self.send_mani_pub = rospy.Publisher('send_mani_pub', Bool, queue_size=10)      #send_mani.py
        self.mani_pick_up_pub = rospy.Publisher('pick_up',Bool,queue_size=10)           #test_mani
        self.go_home_pub = rospy.Publisher('go_home',Bool,queue_size=10)                #home.py
        self.pick_down_pub = rospy.Publisher('arrive_home',Bool,queue_size=10)          #test_mani


        self.frame_pub = rospy.Publisher('/frame', UInt16, queue_size=10) # 지게 보내기 
        self.frame = UInt16()

        self.mode = "patrol"
        self.aruco_check = False
        self.goal_status = 0
        self.check_count_t = 0
        self.check_count_f = 0
        self.close_fin = False
        self.pick_up_fin = False
        self.lift_satting = "setting"








    def sub_msgs(self):
        rospy.Subscriber("fin_move_close", Bool, self.fin_move)   
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
        rospy.Subscriber("arrived_mani", Bool, self.arrive_home)



    
    def control(self,rate_2):
        #####지게 내리고 시작하기#######
        if self.lift_satting == "setting":
            rate_2.sleep()
            self.frame.data = 100
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            self.lift_satting = "fin"
        
        # patrol 
        rate = rospy.Rate(10)
        rate.sleep()
        rospy.loginfo("goal : %s",self.goal_status)
        if self.mode == "patrol":  # self drive mode
            rospy.loginfo("mode : %s",self.mode)

            self.random_dst_pub.publish(True)  # random_pose.py 동작
            rate.sleep()
            self.mode = "find_marker"
            self.goal_status = 0



        # find_marker  마커 확인하기
        if self.mode == "find_marker":
            rospy.loginfo("mode : %s",self.mode)
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
            self.aruco_move_pub.publish("first")  # aruco_move.py
            self.mode = "wait_arrive_aruco"
            rate.sleep()

        if self.mode == "wait_arrive_aruco":

            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  #2 aruco_move,py goal
            
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "secend_aruco_dst"
                self.goal_status = 0

        if self.mode == "secend_aruco_dst":
            rospy.loginfo("mode : %s",self.mode)
            # for n in range(5):
            self.aruco_move_pub.publish("secend")  # aruco_move.py
            self.mode = "secend_wait_arrive_aruco"
            rate.sleep()           

        if self.mode == "secend_wait_arrive_aruco":
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  #2 aruco_move,py goal
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "move_closed_aruco"
                self.goal_status = 0
            rate.sleep()

        if self.mode == "move_closed_aruco":
            rospy.loginfo("mode : %s",self.mode)

            self.aruco_move_pub.publish("not")  # aruco_move.py
            
            self.move_close_pub.publish("first")  # mode_close.py
            self.mode = "wait_fin_closed"
            rate.sleep()

        if self.mode == "wait_fin_closed":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)   #4 move_close.py
            print(self.close_fin)
            if self.close_fin:
                self.mode = "pick_up_lift"
                self.close_fin = False
                self.move_close_pub.publish("not")# mode_close.py
                
            rate.sleep()


        if self.mode == "pick_up_lift":   # 지게 올리기
            
            self.close_fin = False
            # self.mode_pub.publish(self.mode)
            rate_2.sleep()
            self.frame.data = 3500
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            self.mode = "go_home"
            rate_2.sleep()
            # self.mode_pub.publish(self.mode)

        if self.mode == "go_home":
            rospy.loginfo("mode : %s",self.mode)
            self.send_mani_pub.publish(False)
            # for n in range(5):
            self.go_home_pub.publish(True)
            self.mode = "wait_arrive_home"
            self.goal_status = 0
            rate.sleep()

        if self.mode == "wait_arrive_home":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback) #2 home_goal
              # aruco_move.py
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "pick_down_lift"
                self.goal_status = 0
                self.move_close_pub.publish("not")# mode_close.py
            rate.sleep()



        if self.mode =="pick_down_lift":
            ##########도착하고 나서 내리려면 if 문 안으로 옮기기######### 
            print("down")
            rate_2.sleep()
            self.frame.data = 100
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            #####################################################
            self.goal_status =0
            rate_2.sleep()
            self.mode= "move_"

        if self.mode =="move_":
            rospy.loginfo("mode : %s",self.mode)
            self.move_close_pub.publish("secend")  # mode_close.py
            self.mode = "wait_fin_go"
            rate.sleep()            
            
        if self.mode =="wait_fin_go":
            rospy.loginfo("mode : %s",self.mode)
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)   #4 move_close.py
            print(self.close_fin)
            if self.close_fin:
                self.mode = "patrol"
                self.close_fin = False
                self.move_close_pub.publish("not")# mode_close.py
                
            rate.sleep()            

    def GoalPoseCallback(self, data):
        self.goal_status = data.status.status

    def ar_check(self, check_aruco):
        self.aruco_check = check_aruco.check


    def fin_move_close(self, fin):
        self.close_fin = fin.data


def main():
    rospy.init_node("lift_control")
    con = Control()
    rate = rospy.Rate(10)
    rate2 = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        # rospy.loginfo_once("CONTROL TOWER")

        con.control(rate2)
        rate.sleep()
        






if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
    


#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool, UInt16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from multi_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult

class Control():
    def __init__(self):
        
        self.mode_pub = rospy.Publisher('mode', Int32, queue_size=10) # mode 바꾸기
        # self.arrive_mani = rospy.Publisher('/arrive_home', Bool, queue_size=10) # mode 바꾸기
        # self.drive_pub = rospy.Publisher('/drive', Int32, queue_size=1) # 
        self.mode = Int32()
        self.mode.data = 0
        self.aruco_check = False
        self.frame_pub = rospy.Publisher('/frame', UInt16, queue_size=10) # 지게 보내기 
        self.frame = UInt16()
        # self.self_pub = rospy.Publisher('/self_drive_on_off',Int32,queue_size=1)
        # self.stop_move = Int32()
        # self.aruco_move = rospy.Publisher('start_move_close',Int32, queue_size=1)
        self.check_count = 0
        self.goal_status = 0
        self.close_fin = False
        self.pick_up_fin = False
        self.count2 = 0


    def sub_msgs(self):
        rospy.Subscriber('check_aruco',check_msg, self.ar_check)
        rospy.Subscriber("fin_move_close", Bool, self.fin_move)   
        # rospy.Subscriber("/tb3_1/move_base/result", Int32, self.GoalPoseCallback2)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
        # rospy.Subscriber("/fin_send_mani", Bool, self.fin_send)
        # rospy.Subscriber("/tb3_0/fin_pick_up", Bool, self.fin_pick)
        rospy.Subscriber("arrived_mani", Bool, self.arrive_home)


    def arrive_home(self,data):
        self.arrive_ = data.data

    def fin_pick(self,data):
        self.pick_up_fin = data.data
        print(self.pick_up_fin)


    # def fin_send(self, data):
    #     self.mani_fin = data.data

    def GoalPoseCallback(self, data):
        self.goal_status = data.status.status
        # if (self.goal_status == 3 or self.goal_status == 4) and (self.mode == "marker_waiting_position"):
        #     # print("goal reached")
    
    # def GoalPoseCallback2(self, data):
    #     self.goal_status = data.data
    #     # if (self.goal_status == 3 or self.goal_status == 4) and (self.mode == "marker_waiting_position"):
    #     #     # print("goal reached")

    def ar_check(self, check_aruco):
        self.aruco_check = check_aruco.check


    def fin_move(self, fin):
        self.close_fin = fin.data
        print(fin.data)
            
    
    def control(self,rate_2):
        # rospy.loginfo_once("CONTROL TOWER : OK")
        rospy.loginfo(self.mode.data)
        
        # print(self.check_count)
        #####지게 내리고 시작하기#######
        if self.count2 == 0:
            rate_2.sleep()
            self.frame.data = 0
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            self.count2 = 1
        
        ##############################
        self.sub_msgs()
        if self.mode.data == 0:  # self drive mode
            self.goal_status =0
            self.mode_pub.publish(self.mode)
            if self.aruco_check == True:
                self.mode.data = 1
                # self.mode_pub.publish(self.mode)

        elif self.mode.data == 1:  # aruco check mode
            self.mode_pub.publish(self.mode)
            if self.aruco_check == True:
                self.check_count += 1
                
                if self.check_count > 20:
                    self.mode.data = 2
                    # self.mode_pub.publish(self.mode)
            elif self.aruco_check == False:
                self.mode.data = 0
                self.check_count = 0
                # self.mode_pub.publish(self.mode)

        elif self.mode.data == 2:  # aruco marker 발라보기
            self.check_count = 0
            self.mode_pub.publish(self.mode)
            # print(self.goal_status)
            if (self.goal_status == 3 or self.goal_status == 4):
                self.mode.data = 3
                self.goal_status =0
                

        elif self.mode.data == 3:  # aruco marker 발라보기
            self.mode_pub.publish(self.mode)
            print(self.goal_status)
            if (self.goal_status == 3 or self.goal_status == 4):
                self.mode.data = 4
                self.goal_status =0
                self.mode_pub.publish(self.mode)
            
        elif self.mode.data == 4:   # aruco marker로 접근\
            # self.mode_pub.publish(self.mode)
            self.goal_status = 0
            print(self.close_fin)
            if self.close_fin == True:
                print("move fin subscribed")
                self.mode.data = 5
                self.close_fin = False
                # self.mode_pub.publish(self.mode)

        elif self.mode.data == 5:   # 지게 올리기
            self.mode_pub.publish(self.mode)
            self.close_fin = False
            # self.mode_pub.publish(self.mode)
            rate_2.sleep()
            self.frame.data = 3500
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            self.mode.data = 6
            # self.mode_pub.publish(self.mode)

        # elif self.mode.data == 4:   # mani 집기
        #     if self.pick_up_fin == True:
        #         self.mode.data = 5
        #         self.mode_pub.publish(self.mode)

        elif self.mode.data == 6:   # 집가기
            self.mode_pub.publish(self.mode)
           #####################################
                
            self.mode.data = 7


        elif self.mode.data == 7:   # 집가기
            self.mode_pub.publish(self.mode)
            if (self.goal_status == 3 or self.goal_status == 4):
                ##########도착하고 나서 내리려면 if 문 안으로 옮기기######### 
                print("down")
                rate_2.sleep()
                self.frame.data = 100
                self.frame_pub.publish(self.frame)
                rate_2.sleep()
                #####################################################
                self.goal_status =0
                self.mode.data = 0
                # self.mode_pub.publish(self.mode)
        # elif self.mode.data == 6:
            
        #     # self.arrive_mani.publish(False)
        #     if self.arrive_  == True:
        #         self.mode.data = 7
                # self.mode_pub.publish(self.mode)


        # self.self_pub.publish(self.stop_move)
        # self.mode_pub.publish(self.mode)
        # self.aruco_move.publish(self.move)

def main():
    rospy.init_node("lift_control")
    con = Control()
    rate = rospy.Rate(10)
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        # rospy.loginfo_once("CONTROL TOWER")

        con.control(r)
        rate.sleep()
        






if __name__ == "__main__":
    main()
    
    


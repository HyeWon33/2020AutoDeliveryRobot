#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from mani_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID

class Control():
    def __init__(self):
        
        self.mode_pub = rospy.Publisher('mode', Int32, queue_size=10) # mode 바꾸기
        self.arrive_mani = rospy.Publisher('arrive_home', Bool, queue_size=10) # mode 바꾸기
        self.goal_cancel = rospy.Publisher('move_base/cancel',GoalID,queue_size=10)
        self.random_dst_pub = rospy.Publisher('randome_dst_pub',Bool,queue_size=10)
        # self.drive_pub = rospy.Publisher('/drive', Int32, queue_size=1) # 
        self.mode = Int32()
        self.mode.data = 0
        self.aruco_check = False
        # self.self_pub = rospy.Publisher('/self_drive_on_off',Int32,queue_size=1)
        # self.stop_move = Int32()
        # self.aruco_move = rospy.Publisher('start_move_close',Int32, queue_size=1)
        self.check_count = 0
        self.goal_status = 0
        self.close_fin = False
        self.pick_up_fin = False
        self.arrive_  = False
        self.mani_fin = False
        self.fin_ran = False

    # def sub_msgs(self):
        
    #     # rospy.Subscriber("aruco_move_fin", Bool, self.ar_fin)               # robot이 aruco marker앞에 이동 완료
          
    #     # rospy.Subscriber("/tb3_1/move_base/result", Int32, self.GoalPoseCallback2)
        
    #     # rospy.Subscriber("/random_fin", Bool, self.fin_ran)
        
        
        
        

    def fin_mani(self,msg):
        self.mani_fin = msg.data
    def arrive_home(self,data):
        self.arrive_ = data.data

    def fin_pick(self,data):
        self.pick_up_fin = data.data
        # print(self.pick_up_fin)


    def fin_ran(self, data):
        self.fin_ran = data.data

    def GoalPoseCallback(self, data):
        self.goal_status = data.status.status
        # if (self.goal_status == 3 or self.goal_status == 4) and (self.mode == "marker_waiting_position"):
        #     # print("goal reached")
    
    def GoalPoseCallback2(self, data):
        self.goal_status = data.data
        # if (self.goal_status == 3 or self.goal_status == 4) and (self.mode == "marker_waiting_position"):
        #     # print("goal reached")

    def ar_check(self, check_aruco):
        self.aruco_check = check_aruco.check

    # def ar_fin(self, finish):
    #     self.aruco_fin = finish.data

    def fin_move(self, fin):
        self.close_fin = fin.data
            
    
    def control(self):
        # rospy.loginfo_once("CONTROL TOWER : OK")
        
        
        # print(self.check_count)
        
        # self.sub_msgs()
        if self.mode == "patrol":  # self drive mode
            # random_dst_bool = Bool()
            # random_dst_bool.data = True
            self.random_dst_pub.publish(True)
            rospy.loginfo(self.mode.data)
            self.mode = "find marker"
            
        if self.mode == "find marker":
            rospy.Subscriber('check_aruco',check_msg, self.ar_check)
            if self.aruco_check == False:
                self.mode = "patrol"
            elif self.aruco_check == True:
                





                self.mode.data = 1
                print(self.mode.data)
                self.random_dst_pub.publish(False)
                

        elif self.mode.data == 1:  # aruco check mode
            rospy.Subscriber('check_aruco',check_msg, self.ar_check)
            self.mode_pub.publish(self.mode)
            self.goal_cancel.publish()
            rospy.loginfo(self.mode.data)
            if self.aruco_check == True:
                self.check_count += 1
                
                if self.check_count > 20:
                    self.mode.data = 2
                    print(self.mode.data)
                    self.mode_pub.publish(self.mode)
            elif self.aruco_check == False:
                self.mode.data = 0
                self.check_count = 0
                print(self.mode.data)
                self.mode_pub.publish(self.mode)

        elif self.mode.data == 2:  # aruco marker 바라보기
            self.check_count = 0
            rospy.loginfo(self.mode.data)
            #self.mode_pub.publish(self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
            print(self.goal_status)
            if (self.goal_status == 3 or self.goal_status == 4):
                self.mode.data = 3
                self.goal_status =0
                print(self.mode.data)
                self.mode_pub.publish(self.mode)
                
            
        elif self.mode.data == 3:   # aruco marker로 접근\
            rospy.Subscriber("fin_move_close", Bool, self.fin_move) 
            self.goal_status = 0
            rospy.loginfo(self.mode.data)
            print(self.close_fin)
            if self.close_fin == True:
                print("move fin subscribed")
                print(self.mode.data)
                self.mode.data = 4
                self.mode_pub.publish(self.mode)
                self.close_fin = False


        elif self.mode.data == 4:   # mani 집기
            rospy.Subscriber("fin_pick_up", Bool, self.fin_pick)
            self.mode_pub.publish(self.mode)
            rospy.loginfo(self.mode.data)
            self.close_fin = False

            if self.pick_up_fin == True:
                self.mode.data = 5
                self.mode_pub.publish(self.mode)
                self.pick_up_fin = False

        elif self.mode.data == 5:   # 집가기
            rospy.Subscriber("arrived_mani", Bool, self.arrive_home)
            self.mode_pub.publish(self.mode)
            self.pick_up_fin = False
            rospy.loginfo(self.mode.data)
            if self.arrive_ == True:
                self.mode.data = 6
                self.arrive_ = False
                
                
                
        elif self.mode.data == 6:
            
            self.mode_pub.publish(self.mode)
            self.arrive_ = False
            rospy.loginfo(self.mode.data)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
            self.arrive_mani.publish(False)
            if (self.goal_status == 3 or self.goal_status == 4):
                self.mode.data = 7
                self.goal_status =0
                


        elif self.mode.data == 7:
            rospy.Subscriber("fin_mani", Bool, self.fin_mani)
            self.mode_pub.publish(self.mode)
            self.arrive_mani.publish(True)
            rospy.loginfo(self.mode.data)
            if self.mani_fin ==True:
                self.mode.data = 0
                self.mode_pub.publish(self.mode)




        # self.self_pub.publish(self.stop_move)
        # self.mode_pub.publish(self.mode)
        # self.aruco_move.publish(self.move)

def main():
    rospy.init_node("contol_tower")
    con = Control()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # rospy.loginfo_once("CONTROL TOWER")
        con.control()
        rate.sleep()
        






if __name__ == "__main__":
    main()
    
    


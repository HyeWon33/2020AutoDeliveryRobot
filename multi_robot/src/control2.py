#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from multi_robot.msg import Float32Multi
import numpy as np
from rospy.numpy_msg import numpy_msg


class Control:
    def __init__(self):
        rospy.init_node("control_tower")
        self.servo_pub = rospy.Publisher('/servo', Int32, queue_size=10)
        self.mode_pub=rospy.Publisher('/mode_msg',Int32,queue_size=10) 
        self.mani_robot_stop_pub=rospy.Publisher('/mani_robot_stop',Int32,queue_size=10)#self_drive.py
        self.call_frame_pub=rospy.Publisher('/call_frame',Int32,queue_size=10) #call_robot2.py
        self.mani_move_pub =rospy.Publisher('/mani_move_msg',Int32,queue_size=10)
        self.return_frame_pub=rospy.Publisher('/return_frame_msg',Int32,queue_size=10)#home.py
        
        self.mani_stop = Int32()
        self.serv = Int32()
        self.mani_state = Int32()
        
        self.call = -1
        self.home = -1
        self.frame_mode = "Ready to Raise up" #before raise up
        self.mani_mode = "Ready to Pick up box" 
        self.mode = 0
        
        #acuco
        self.aurco_move_pub =rospy.Publisher('/start_arco',Int32,queue_size=10)
        self.aruco_mode = -1
    def main(self):
          #모든 상태가 0일 때 mani_move동작
        while not rospy.is_shutdown():
            self.starting_trigger()
            ####메니 동작
            if self.mode == 0:               #mani만 움직이는 동안
                rospy.loginfo(self.mode)
                if self.aruco_mode == 1:  #aruco 발견시
                    self.mode = 1
                else:
                    self.mani_robot_stop_pub.publish(1) #mani robot 움직임

            
            elif self.mode == 1:             #아르코마커 발견시
                rospy.loginfo(self.mode)
                if self.aruco_move_mode == 0: #아르코(물체)에 접근 했을 때
                    self.mode = 2
                else:
                    self.mani_robot_stop_pub.publish(0) #mani robot 움직임 정지
                    self.aurco_move_pub.publish(1) #아르코(물체)에 접근 이동노드 on


            elif self.mode == 2: #물체에 접군 후
                rospy.loginfo(self.mode)
                self.aurco_move_pub.publish(0)#아르코(물체)에 접근 이동노드off
                if self.aruco_id == 1: #작은물체 인식 하면
                    self.mani_move_pub.publish(1) #mani 작은 동작 시작
                    self.mode = 3
                elif self.aruco_id == 2: #큰물체 인식 하면
                    self.mani_move_pub.publish(2) #mani 큰 동작 시작
                    self.mode = 4


            elif self.mode == 3: #작은 물제 집는동안
                rospy.loginfo(self.mode)
                if self.mani_mode == "Release small box":
                    self.mani_mode = "Ready to Pick up box"
                    self.mode = 0
            
            elif self.mode == 4: #큰 물체 잡는 동안
                rospy.loginfo(self.mode)
                if self.mani_mode == "Pick up large box": 
                    self.mani_mode = "Ready to Release large box"
                    self.call_frame_pub.publish(1)
                elif self.mani_mode == "Release large box":
                    self.mani_mode = "Ready to Pick up box"
                    self.return_frame_pub.publish(1)
                    self.mode = 0

            ###메니 동작 완료


            # elif self.frame_mode == "Raise up": #When recieve the control msg: 1 by sender.py 
		    #     #Raise up the frame
            #     self.PickUp()
            #     print(self.frame_mode)
            #     self.servo_pub.publish(self.serv)
            #     self.frame_mode = "Ready to Put down"
            
            # elif self.frame_mode == "Put down": #When recieve the control msg: 2 by sender.py 
            #     #Put down the frame
            #     self.PickDown()
            #     print(self.frame_mode)
            #     self.servo_pub.publish(self.serv)
            #     self.frame_mode = "Ready to Raise up"

            # else: #모든 상태가 0일 때 mani_move동작:
            #     self.mani_robot_stop_pub(1)

    def starting_trigger(self):
        rospy.Subscriber("/control_frame", Int32, self.StartingCallback)
        rospy.Subscriber("/Mani_state", Int32, self.ManiCallback)
        rospy.Subscriber("/call_fin", Int32, self.CallCallback)        
        rospy.Subscriber("/home_fin", Int32, self.HomeCallback)
        rospy.Subscriber('/aruco_msg',Int32, self.aruco)
        rospy.Subscriber('/id_msg',Int32, self.aruco_id)
        rospy.Subscriber("/aruco_move", Int32, self.aruco_move) #aruco_move.py
        
    def aruco(self,msg):
        self.aruco_mode = msg.data
    def aruco_id(self,msg):
        self.aruco_id =msg.data
    def aruco_move(self,msg):
        self.aruco_move_mode = msg.data

    def StartingCallback(self, data):
        if self.mani_mode == "Pick up large box" and data.data == 1:
            self.frame_mode = "Raise up"
            print("setting mode Raise up")
        elif self.mani_mode == "Release large box" and data.data == 2 and self.home == 1:
            self.frame_mode = "Put down"
            self.home = -1
            print("setting mode Put down")

            
    def ManiCallback(self, data):
        if self.mani_mode == "Ready to Pick up box"  and data.data == 1:
           self.mani_mode = "Release small box"
           print("Setting mode Release small box")
           
        elif self.mani_mode == "Ready to Pick up box"  and data.data == 2:
            self.mani_mode = "Pick up large box" 
            print("Setting Pick up large box")
            
        elif self.mani_mode == "Ready to Release large box" and data.data == 2 and self.call == 1:
            self.mani_mode = "Release large box"
            self.call = -1
            print("Setting mode Release large box")

    def CallCallback(self, data):
        self.call = data.data  
        
    def HomeCallback(self, data):
        self.home = data.data 
        
    def Delay(self, data):
        self.rate = rospy.Rate(data)
        self.rate.sleep()

    def PickUp(self):
        self.Delay(3)
        self.serv.data = 2000
        self.Delay(3)

    def PickDown(self):
        self.Delay(3)
        self.serv.data = 0
        self.Delay(3)
        
def main():
   
    Turtle = Control()
    Turtle.main()

if __name__ == "__main__":
    main()


#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from mani_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID

"""
mani control 노드 모든 노드들의 연결과 on/off 를 담당
"""
class Control():
    def __init__(self):
        # navigation 취소
        self.goal_cancel = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)  # cancel goal
        # 각 노드들 on/off
        self.random_dst_pub = rospy.Publisher('randome_dst_pub', Bool, queue_size=1)  # random_pose.py
        self.aruco_move_pub = rospy.Publisher('aruco_move_pub', Bool, queue_size=1)  # aruco_move.py
        self.move_close_pub = rospy.Publisher('start_move_closed', String, queue_size=1)  # move_close.py
        self.send_mani_pub = rospy.Publisher('send_mani_pub', Bool, queue_size=1)  # send_mani.py
        self.mani_pick_up_pub = rospy.Publisher('pick_up', Bool, queue_size=1)  # test_mani
        self.go_home_pub = rospy.Publisher('go_home', Bool, queue_size=1)  # home.py
        self.pick_down_pub = rospy.Publisher('arrive_home', Bool, queue_size=1)  # test_mani
        self.mode = "patrol"
        self.aruco_check = False
        self.goal_status = 0
        self.check_count_t = 0
        self.check_count_f = 0
        self.close_fin = False
        self.pick_up_fin = False
        self.mani_fin = False
        self.mani_error = False
        self.mani_error_fin = False

    # main_control
    def control(self):
        # rospy.loginfo_once("CONTROL TOWER : OK")

        rate = rospy.Rate(10)
        rate.sleep()
        ## rospy.loginfo("goal : %s",self.goal_status)

        # 로봇이 랜덤하게 navigation 하도록 목표지점 전송. random_dst_pub 노드(random_pose.py)실행
        if self.mode == "patrol":  # self drive mode
            rospy.loginfo("mode : %s", self.mode)
            self.mani_error = False # mani error 초기화

            self.random_dst_pub.publish(True)  # random_dst_pub 노드(random_pose.py)실행
            rate.sleep()
            self.mode = "find_marker"  # find_marker 모드로 변환
            # self.goal_status = 0

        # marker 마커 검출 여부 확인하기
        if self.mode == "find_marker":
            # rospy.loginfo("mode : %s",self.mode)
            self.random_dst_pub.publish(False)  # random_pose.py 멈춤
            rospy.Subscriber('check_aruco', check_msg, self.aruco_check_callback)  # marker 검출여부 받기    #5 aruco_detect.py
            rate.sleep()
            # aruco 검출 여부확인
            if self.aruco_check:  # 검출이 한번 되면 navigation 취소 후 대기 모드로 전환
                self.goal_cancel.publish()  # navigation 취소
                self.mode = "wait_count"  # wait_count 모드로 설정
            elif not self.aruco_check: # 검출이 안되면
                self.mode = "check_finish_goal" # check_finish_goal 모드로 변경

        # 목표지점에 도착 했는지 여부 확인
        if self.mode == "check_finish_goal":
            rospy.loginfo("mode : %s", self.mode)
            rospy.loginfo("goal : %s", self.goal_status)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  # 2 randome_pose.py goal
            # 도착시 다시 patrol 모드로 진입
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "patrol"
            # 진행중일때 find_marker 모드로 진입
            else:
                self.mode = "find_marker"
            rate.sleep()
        # aruco 검출 한번되면 기다리면서 확실히 검출
        if self.mode == "wait_count":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber('check_aruco', check_msg, self.aruco_check_callback)  # 5 aruco_detect.py
            # 계속 검출되고 있으면 카운트 증가
            if self.aruco_check:
                self.check_count_t += 1
                # 커운트가 20 이상 되면 aruco 접근 모드로 전환
                if self.check_count_t > 20:
                    self.mode = "aruco_dst"
                    self.check_count_t = 0
            # 검출이 안되는게 지속되면 patrol 로 변환
            elif not self.aruco_check:
                self.check_count_f += 1
                if self.check_count_f > 20:
                    self.mode = "patrol"
                    self.check_count_f = 0
            rate.sleep()

        # 검출이 확실히 되면 aruco 전방 45cm 접근 노드 실행
        if self.mode == "aruco_dst":
            rospy.loginfo("mode : %s", self.mode)
            # for n in range(5):
            self.aruco_move_pub.publish(True)  # aruco_move.py
            self.mode = "wait_arrive_aruco"
            rate.sleep()

        # 목표지점에 도착할때까지 대기
        if self.mode == "wait_arrive_aruco":
            rospy.loginfo("mode : %s", self.mode)

            # self.aruco_move_pub.publish(False) 
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  # 2 aruco_move,py goal
            # self.aruco_move_pub.publish(False)  # aruco_move.py

            # 목표지점에 도착 확인
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "move_closed_aruco"
                self.goal_status = 0
            rate.sleep()

        # 도착하면 물체 방향으로 일정거리 전진
        if self.mode == "move_closed_aruco":
            rospy.loginfo("mode : %s", self.mode)
            self.aruco_move_pub.publish(False)  # aruco_move.py
            self.move_close_pub.publish("front")  # mode_close.py("front"-전진)
            self.mode = "wait_fin_closed"
            rate.sleep()

        # 물체에 접근 대기
        if self.mode == "wait_fin_closed":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("fin_move_close", Bool, self.move_close_finish_callback)  # 4 move_close.py
            # self.move_close_pub.publish("not")  # mode_close.py
            # 접근이 완료되면
            if self.close_fin:
                self.mode = "send_mani_dst"
                self.close_fin = False
                self.move_close_pub.publish("not")  # mode_close.py("not" 노드 정지)
            rate.sleep()


        # mani에서 본 물체에 위치 계산 및 전송 노드 실행
        if self.mode == "send_mani_dst":
            rospy.loginfo("mode : %s", self.mode)
            # self.move_close_pub.publish(0)# mode_close.py
            self.send_mani_pub.publish(True)
            self.mode = "wait_pick_up"
            rate.sleep()

        # mani 가 동작 완료 할때까지 대기
        if self.mode == "wait_pick_up":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("fin_pick_up", Bool, self.pick_up_finish_callback)  # 3 test_mani(동작완료)
            rospy.Subscriber("mani_plan_status", Bool, self.mani_error_callback)  # 3 test_mani(도중에 에러러
            # rospy.loginfo("mani_eror : %s", self.mani_error)

            rate.sleep()
            #mani가 집는걸 완료했을때
            if self.pick_up_fin:
                self.mode = "go_home"
                self.pick_up_fin = False
                self.send_mani_pub.publish(False)
                rate.sleep()
            #mani가 집지 못하고 error를 보냈을때
            elif self.mani_error:
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

        # 집은 후 적재지로 이동
        if self.mode == "go_home":
            rospy.loginfo("mode : %s", self.mode)
            self.send_mani_pub.publish(False)
            # for n in range(5):
            self.go_home_pub.publish(True)
            self.mode = "wait_arrive_home"
            rate.sleep()

        # 적재지에 도착할때까지 대기
        if self.mode == "wait_arrive_home":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  # 2 home_goal
            # aruco_move.py
            if self.goal_status == 3 or self.goal_status == 4: # 도착했을때
                self.mode = "move_back"
                self.goal_status = 0
            rate.sleep()

        # 일정거리 후진
        if self.mode == "move_back":
            rospy.loginfo("mode : %s", self.mode)

            self.move_close_pub.publish("back")  # mode_close.py
            self.mode = "move_back_fin"
            rate.sleep()

        # 후진이 환료 될때 까지 대기
        if self.mode == "move_back_fin":
            rospy.Subscriber("fin_move_close", Bool, self.move_close_finish_callback)
            print(self.close_fin)
            if self.close_fin:
                self.mode = "pick_down_mani"
                self.close_fin = False
                self.move_close_pub.publish("not")  # mode_close.py

        # 물체 내려 놓기
        if self.mode == "pick_down_mani":
            rospy.loginfo("mode : %s", self.mode)
            self.go_home_pub.publish(False)
            self.pick_down_pub.publish(True)
            self.mode = "wait_pick_down"
            rate.sleep()

        # 물체 내려 놓을 때까지 대기
        if self.mode == "wait_pick_down":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("fin_mani", Bool, self.put_down_finish)  # 1 test_mani
            if self.mani_fin:
                self.mode = "move_front"
            rate.sleep()

        # 일정 거리 전진
        if self.mode == "move_front":
            rospy.loginfo("mode : %s", self.mode)

            self.move_close_pub.publish("front")  # mode_close.py
            self.mode = "move_front_fin"
            rate.sleep()


        # 전진 이 완료 될때까지 대기
        if self.mode == "move_front_fin":
            rospy.Subscriber("fin_move_close", Bool, self.move_close_finish_callback)
            print(self.close_fin)
            if self.close_fin: # 전진이 완료 되면면
                self.mode = "patro"
                self.close_fin = False
                self.move_close_pub.publish("not")  # mode_close.py

    # def mani_er_fin(self, error_fin):
    #     self.mani_error_fin = error_fin.data

    # aruco callback 함수
    def aruco_check_callback(self, check_aruco):  # 5
        self.aruco_check = check_aruco.check

    # 일정 거리접근 노드(move_close) 완료 callback 함수
    def move_close_finish_callback(self, fin):  # 4
        self.close_fin = fin.data

    # 메니가 물체를 집는거 완료 callback 함수
    def pick_up_finish_callback(self, data):  # 3
        self.pick_up_fin = data.data

    # navigation goalpose callback 함수
    def GoalPoseCallback(self, data):  # 2
        print("goal:", data.status.status)
        self.goal_status = data.status.status

    # 메니가 물체를 놓는거 완료 callback 함수
    def put_down_finish(self, msg):  # 1
        self.mani_fin = msg.data

    # 메니가 에러 났을때 callback 함수
    def mani_error_callback(self, msg):
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

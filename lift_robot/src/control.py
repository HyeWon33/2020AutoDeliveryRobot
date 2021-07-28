#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Int32, Bool, UInt16, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from lift_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID

"""
지게 control 노드 모든 노드들의 연결과 on/off 를 담당

1. 로봇이 랜덤하게 navigation 하도록 목표지점 전송.
   - random 좌표는 2~4개가 적당하고, 다른 로봇과 곂치지 않게 설정
2. 랜덤 좌표로 이동하면서 marker 마커 검출 여부 확인하기
   - 검출이 한번이라도 되면 잠시 대기 하면서 여러번 검출되는지 확인(현재는 20번 count 기준)
   - 대기도중 마커가 안보이거나 랜덤 좌표로 도착시 다시 1. 2. 반복
3. 검출이 될시 검출이 확실히 되면 aruco 전방 접근 노드 실행
   - navigation을 사용하여 마커가 로봇 전방 50cm에 오게 이동(frist)
   - 이후 navigation이 끝날때까지 대기
4. 도착을 하면 다시  aruco 전방 접근 노드 실행
   - navigation을 사용하여 마커가 로봇 후방 45cm에 오게 이동(second)
   - 이후 navigation이 끝날때까지 대기
   - 바로 후방으로 보게 하였을 때보다 접근 노드 두번 실행 시 오차 감소
5. 도착하면 물체 방향으로 일정거리 후진
   - 후진이 완료 될때까지 대기
6. 지게올리기
   - 3500으로 설정 되있지만 물체 변경시 조정 필요
7. 집은 후 적재지로 이동
   - navigation을 사용하여 미리 설정해둔 적재지로 이동(이때는 적제지 주변에 물체(벽)이 없으므로 바로 이동)
   - 물체를 내려놓은후 일정거리 전진 필요
8. 위 과정 반복
"""
class Control():
    def __init__(self):
        # navigation 취소
        self.goal_cancel = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)  # cancel goal
        # 각 노드들 on/off
        self.random_dst_pub = rospy.Publisher('randome_dst_pub', Bool, queue_size=10)  # random_pose.py 
        self.aruco_move_pub = rospy.Publisher('aruco_move_pub', String, queue_size=10)  # aruco_move.py
        self.move_close_pub = rospy.Publisher('start_move_closed', String, queue_size=10)  # move_close.py
        self.send_mani_pub = rospy.Publisher('send_mani_pub', Bool, queue_size=10)  # send_mani.py
        self.mani_pick_up_pub = rospy.Publisher('pick_up', Bool, queue_size=10)  # test_mani
        self.go_home_pub = rospy.Publisher('go_home', Bool, queue_size=10)  # home.py
        self.pick_down_pub = rospy.Publisher('arrive_home', Bool, queue_size=10)  # test_mani
        
        self.frame_pub = rospy.Publisher('/frame', UInt16, queue_size=10)  # 지게 보내기 
        self.frame = UInt16()

        self.mode = "patrol"
        self.aruco_check = False
        self.goal_status = 0
        self.check_count_t = 0
        self.check_count_f = 0
        self.close_fin = False
        self.pick_up_fin = False
        self.lift_satting = "setting"

    # def sub_msgs(self):
    #     rospy.Subscriber("fin_move_close", Bool, self.fin_move)
    #     rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
    #     rospy.Subscriber("arrived_mani", Bool, self.arrive_home)

    def control(self, rate_2):
        #####지게 내리고 시작하기#######
        if self.lift_satting == "setting":
            rate_2.sleep()
            self.frame.data = 100
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            self.lift_satting = "fin"


        rate = rospy.Rate(10)
        rate.sleep()
        rospy.loginfo("goal : %s", self.goal_status)
        # 로봇이 랜덤하게 navigation 하도록 목표지점 전송. random_dst_pub 노드(random_pose.py)실행
        if self.mode == "patrol":  # self drive mode
            rospy.loginfo("mode : %s", self.mode)

            self.random_dst_pub.publish(True)  # random_pose.py 동작
            rate.sleep()
            self.mode = "find_marker"
            self.goal_status = 0

        # marker 마커 검출 여부 확인하기
        if self.mode == "find_marker":
            rospy.loginfo("mode : %s", self.mode)
            self.random_dst_pub.publish(False)  # random_pose.py 멈춤
            rospy.Subscriber('check_aruco', check_msg, self.ar_check)  # marker 검출여부 받기    #5 aruco_detect.py
            rate.sleep()
            if self.aruco_check:  # 검출이 한번 되면 navigation 취소 후 대기 모드로 전환
                self.goal_cancel.publish()  # navigation 취소
                self.mode = "wait_count"  # marker 검출이 한번이라도 되면
            elif not self.aruco_check:  # 검출이 안되면
                self.mode = "check_finish_goal" # check_finish_goal 모드로 변경

        # 목표지점에 도착 했는지 여부 확인
        if self.mode == "check_finish_goal":
            rospy.loginfo("mode : %s", self.mode)
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
            rospy.Subscriber('check_aruco', check_msg, self.ar_check)  # 5 aruco_detect.py
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
            self.aruco_move_pub.publish("first")  # aruco_move.py
            self.mode = "wait_arrive_aruco"
            rate.sleep()
            
        # 목표지점에 도착할때까지 대기
        if self.mode == "wait_arrive_aruco":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  # 2 aruco_move,py goal
            # 목표지점에 도착 확인
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "secend_aruco_dst"
                self.goal_status = 0
                
        # 한번더 aruco 마커 전방 40cm 에 180 회전
        if self.mode == "secend_aruco_dst":
            rospy.loginfo("mode : %s", self.mode)
            # for n in range(5):
            self.aruco_move_pub.publish("second")  # aruco_move.py
            self.mode = "secend_wait_arrive_aruco"
            rate.sleep()
            
        # 완료 될때까지 대기
        if self.mode == "secend_wait_arrive_aruco":
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  # 2 aruco_move,py goal
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "move_closed_aruco"
                self.goal_status = 0
            rate.sleep()

        # 도착하면 물체 방향으로 일정거리 전진
        if self.mode == "move_closed_aruco":
            rospy.loginfo("mode : %s", self.mode)
            self.aruco_move_pub.publish("not")  # aruco_move.py
            self.move_close_pub.publish("first")  # mode_close.py
            self.mode = "wait_fin_closed"
            rate.sleep()
            
        # 물체에 접근 대기
        if self.mode == "wait_fin_closed":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)  # 4 move_close.py
            print(self.close_fin)
            if self.close_fin:
                self.mode = "pick_up_lift"
                self.close_fin = False
                self.move_close_pub.publish("not")  # mode_close.py

            rate.sleep()
            
        # 지게올리기
        if self.mode == "pick_up_lift":
            self.close_fin = False
            # self.mode_pub.publish(self.mode)
            rate_2.sleep()
            self.frame.data = 3500
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            self.mode = "go_home"
            rate_2.sleep()
            # self.mode_pub.publish(self.mode)

        # 적재지로 이동
        if self.mode == "go_home":
            rospy.loginfo("mode : %s", self.mode)
            self.send_mani_pub.publish(False)
            # for n in range(5):
            self.go_home_pub.publish(True)
            self.mode = "wait_arrive_home"
            self.goal_status = 0
            rate.sleep()

        # 도착할때까지 대기
        if self.mode == "wait_arrive_home":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("move_base/result", MoveBaseActionResult, self.GoalPoseCallback)  # 2 home_goal
            # aruco_move.py
            if self.goal_status == 3 or self.goal_status == 4:
                self.mode = "pick_down_lift"
                self.goal_status = 0
                self.move_close_pub.publish("not")  # mode_close.py
            rate.sleep()

        # 재게 내리기
        if self.mode == "pick_down_lift":
            ##########도착하고 나서 내리려면 if 문 안으로 옮기기######### 
            print("down")
            rate_2.sleep()
            self.frame.data = 100
            self.frame_pub.publish(self.frame)
            rate_2.sleep()
            #####################################################
            self.goal_status = 0
            rate_2.sleep()
            self.mode = "move_front"

        # 앞으로 전진
        if self.mode == "move_front":
            rospy.loginfo("mode : %s", self.mode)
            self.move_close_pub.publish("secend")  # mode_close.py
            self.mode = "wait_fin_go"
            rate.sleep()
        
        # 전진 완료할때까지 대기 후 patrol 모드로 전환
        if self.mode == "wait_fin_go":
            rospy.loginfo("mode : %s", self.mode)
            rospy.Subscriber("fin_move_close", Bool, self.fin_move_close)  # 4 move_close.py
            print(self.close_fin)
            if self.close_fin:
                self.mode = "patrol"
                self.close_fin = False
                self.move_close_pub.publish("not")  # mode_close.py

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

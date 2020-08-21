#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import random
import cv2


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.nav_goal = PoseStamped()
        
        
        self.ddd = rospy.Subscriber('call_frame', Int32, self.Start)

        self.a = 0
        self.start = 0
        self.quaternion = None
        self.orientation_q = None
        self.rvecs = None
        self.tvecs = None
        self.sub = rospy.Subscriber('odom', Odometry, self.get_rotation)
        self.aruco_sub = rospy.Subscriber('rvecs_msg', aruco_msgs, self.callback1)
    def Start(self, msg):
        self.start = msg.data

    def callback1(self, aruco_data):

        self.rvecs = None
        self.tvecs = None
        self.rvecs = np.array([[aruco_data.r_x], [aruco_data.r_y], [aruco_data.r_z]])
        self.tvecs = np.array([[aruco_data.t_x], [aruco_data.t_y], [aruco_data.t_z]])
        print("sdfd")
    def get_rotation(self, msg):
        self.pose_p = msg.pose.pose.position
        self.orientation_q = msg.pose.pose.orientation
        orientation_q = self.orientation_q
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        self.Cul(self.pose_p.x,self.pose_p.y, yaw)

    def Cul(self, x, y, theta):

        self.main_matrix = self.make_matrix(x, y, theta)
        # base_matrix = self.make_matrix(self.tvecs[2], -self.tvecs[0], self.rvecs[1])

        # move_tp = self.cul_matrix(self.main_matrix,base_matrix)
        # rospy.loginfo("move_tp")
        # rospy.loginfo(move_tp)
        # self.transe_quaternion(move_tp)
        ar_pose = np.array([[self.tvecs[2], -self.tvecs[0],1]])
        fin_pose = np.dot(self.main_matrix,ar_pose.T)
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = fin_pose[0]  #self.quaternion[0]
        self.nav_goal.pose.position.y =  fin_pose[1]   #self.quaternion[1]
        print("ddd")
        self.nav_goal.pose.orientation.z = self.orientation_q.z
        self.nav_goal.pose.orientation.w = self.orientation_q.w
        
        if self.start ==1:
            self.nav_pub.publish(self.nav_goal)
            self.main_matrix=None
            base_matrix=None
            


    def make_matrix(self, x, y, theta):
        matrix = np.array([[np.cos(theta), -np.sin(theta), x],
                           [np.sin(theta), np.cos(theta), y],
                           [0, 0, 1]])
        return matrix

    def cul_matrix(self, ma_1=None, ma_2=None):
        zero_matrix = self.make_matrix(0, 0, 0)
        if ma_2 is None:

            result_matrix = np.dot(zero_matrix, ma_1)
        else:
            result_matrix = np.dot(ma_1, ma_2)
        return result_matrix

    def transe_quaternion(self, matrix):
        goal_x, goal_y, goal_rad = self.get_params(matrix)
        goal_deg = (goal_rad * (180 / np.pi))
        ox, oy, ow, oz = tf.transformations.quaternion_from_euler(goal_x, goal_y, goal_rad)
        self.quaternion = [ goal_x, goal_y, ow, oz]

    def get_params(self, matrix):
        x, y = matrix[:2, 2]
        theta = np.arccos(matrix[0, 0])
        # cosine, sine = matrix[:2, 0]
        # theta = np.arctan2(sine, cosine)
        return x, y, theta


def main():
    rospy.init_node("aruco_move")
    rp = Random_Pose()
    rospy.spin()


if __name__ == '__main__':
    main()

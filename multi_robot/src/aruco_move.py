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
        self.sub = rospy.Subscriber('odom', Odometry, self.get_rotation)
        self.aruco_sub = rospy.Subscriber('rvecs_msg', aruco_msgs, self.callback1)
        self.ddd = rospy.Subscriber('call_frame', Int32, self.Start)

        self.a = 0
        self.start = 0
        self.quaternion = None
        self.orientation_q = None

    def Start(self, msg):
        self.start = msg.data
        rospy.loginfo(self.start)

    def callback1(self, aruco_data):
	self.r_x = aruco_data.r_x
	self.r_y = aruco_data.r_y
	self.r_z = aruco_data.r_z
	self.t_x = aruco_data.t_x
	self.t_y = aruco_data.t_y
	self.t_z = aruco_data.t_z
        self.rvecs = np.array([[aruco_data.r_x], [aruco_data.r_y], [aruco_data.r_z]])
        self.tvecs = np.array([[aruco_data.t_x], [aruco_data.t_y], [aruco_data.t_z]])
        rotation_matrix = np.zeros((3, 3))
        cv2.Rodrigues(src=self.rvecs,dst=rotation_matrix)
	original_matrix = np.array([[1],[0],[0]])
        camera_matrix = np.dot(rotation_matrix.T, original_matrix)
        self.camera_matrix = np.squeeze(camera_matrix)
        rospy.loginfo(self.camera_matrix)


    def get_rotation(self, msg):
        pose_p = msg.pose.pose.position

        self.orientation_q = msg.pose.pose.orientation
        orientation_q = self.orientation_q
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.Cul(pose_p.x, pose_p.y, yaw)

    def Cul(self, x, y, theta):
        rospy.loginfo("x      : %f , y      : %f , theta      : %f " % (x, y, theta * (180 / np.pi)))
        self.main_matrix = self.make_matrix(0, 0, theta)
        base_matrix = self.make_matrix(self.t_z, -self.t_x, self.r_y)


        zero_tp = self.cul_matrix(base_matrix)
        move_tp = self.cul_matrix(self.main_matrix, base_matrix)
        zero_move_tp = self.cul_matrix(zero_tp, base_matrix)
        self.transe_quaternion(zero_tp)
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = self.quaternion[0]
        self.nav_goal.pose.position.y = self.quaternion[1]
        self.nav_goal.pose.orientation.z = self.quaternion[2]
        self.nav_goal.pose.orientation.w = self.quaternion[3]

        if self.start == 1:
            self.nav_pub.publish(self.nav_goal)

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
        rospy.loginfo("goal_x : %f , goal_y : %f , goal_theta : %f" % (goal_x, goal_y, goal_deg))
        ox, oy, ow, oz = tf.transformations.quaternion_from_euler(goal_x, goal_y, goal_rad)
        self.quaternion = [goal_x, goal_y, ow, oz]

    def get_params(self, matrix):
        x, y = matrix[:2, 2]
        theta = np.arccos(matrix[0, 0])
        return x, y, theta


def main():
    rospy.init_node("aruco_move")
    rp = Random_Pose()
    rospy.spin()


if __name__ == '__main__':
    main()

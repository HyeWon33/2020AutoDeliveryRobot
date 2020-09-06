#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import math


class Mani_Calcul():
    def __init__(self):
        self.robot_pub = rospy.Publisher("/tb3_0/move_base_simple/goal", PoseStamped, queue_size=10)
        self.aruco_pub = rospy.Publisher("/tb3_1/move_base_simple/goal", PoseStamped, queue_size=10)
        self.main_pub = rospy.Publisher("/tb3_2/move_base_simple/goal", PoseStamped, queue_size=10)
        self.robot_mani_pub = rospy.Publisher("/tb3_3/move_base_simple/goal", PoseStamped, queue_size=10)
        self.mani_aruco_pub = rospy.Publisher("/tb3_4/move_base_simple/goal", PoseStamped, queue_size=10)
        self.robot_aruco_pub = rospy.Publisher("/tb3_5/move_base_simple/goal", PoseStamped, queue_size=10)
        self.r_m_a_pub = rospy.Publisher("/tb3_6/move_base_simple/goal", PoseStamped, queue_size=10)
        self.r_a_m_pub = rospy.Publisher("/tb3_7/move_base_simple/goal", PoseStamped, queue_size=10)
        self.nav_goal = PoseStamped()

        self.aruco_rv = (1, 1, 1)
        self.aruco_tv = [1, 1, 1]
        self.robot_orientation = []
        self.mani_orientation = []
        self.robot_pose = np.zeros((1, 3))
        self.mani_pose = np.zeros((1, 3))
        self.aruco_tv = np.zeros((1, 3))
        rospy.Subscriber('odom', Odometry, self.Robot_pose)
        self.mani = rospy.Subscriber('mani_pose', Pose, self.Mani_pose)

        self.aruco = rospy.Subscriber('rvecs_msg', aruco_msgs, self.Aruco)

    def main(self):
        # robot
        robot_3d_matrix = self.Make_3d_matrix(self.robot_pose, self.robot_orientation)
        # mani
        mani_3d_matrix = self.Make_3d_matrix(self.mani_pose, self.mani_orientation)
        # aruco
        aruco_rotation = self.Create_Rotation_matrix_T(self.aruco_rv)
        aruco_3d_matrix = self.Marge_rota_trace(aruco_rotation, self.aruco_tv.T)
        # robot_mani
        r_m_matirx = np.dot(robot_3d_matrix, mani_3d_matrix)
        # mani_aruco
        m_a_matrix = np.dot(mani_3d_matrix, aruco_3d_matrix)
        r_a_matrix = np.dot(robot_3d_matrix, aruco_3d_matrix)
        r_m_a_matrix = np.dot(robot_3d_matrix, mani_3d_matrix, aruco_3d_matrix)
        r_a_m_matrix = np.dot(robot_3d_matrix, aruco_3d_matrix, mani_3d_matrix)

        #########################################

        robot_goal = self.sss(robot_3d_matrix)
        print(robot_goal)
        self.robot_pub.publish(robot_goal)

        mani_goal = self.sss(mani_3d_matrix)
        self.main_pub.publish(mani_goal)

        aruco_goal = self.sss(aruco_3d_matrix)
        self.aruco_pub.publish(aruco_goal)

        r_m_goal = self.sss(r_m_matirx)
        self.robot_mani_pub.publish(r_m_goal)
        m_a_goal = self.sss(m_a_matrix)
        self.mani_aruco_pub.publish(m_a_goal)
        r_a_goal = self.sss(r_a_matrix)
        self.robot_aruco_pub.publish(r_a_goal)
        r_m_a_goal = self.sss(r_m_a_matrix)
        self.r_m_a_pub.publish(r_m_a_goal)
        r_a_m_goal = self.sss(r_a_m_matrix)
        self.r_a_m_pub.publish(r_a_m_goal)

    def sss(self, matrix):
        # matrix = np.dot(robot_3d_matrix, aruco_3d_matrix)
        t, r, p, y = self.Get_RPY_to_rotation_vector(matrix)
        quaternion = self.Quaternion_from_Euler(0, 0, y)
        goal = self.send_pose(t, quaternion=quaternion)
        return goal
        # print(t,r,p,y)

    def Make_3d_matrix(self, pose, orientation):
        euler = self.Euler_from_Quaternion(orientation)
        rotaion = self.Create_Rotation_matrix(euler)
        matrix_3d = self.Marge_rota_trace(rotaion, pose.T)
        return matrix_3d

    def Robot_pose(self, msg):
        # print(msg.position)
        self.robot_pose = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])
        self.robot_orientation = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                                  msg.pose.pose.orientation.w]
        # self.robot_orientation = [0, 0, msg.pose.pose.position.z,
        #                           msg.pose.pose.orientation.w]

    def Mani_pose(self, msg):
        # print(msg.position)
        self.mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]])
        self.mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def Aruco(self, msg):
        self.aruco_tv = np.array([[msg.t_x, msg.t_y, msg.t_z]])
        self.aruco_rv = [msg.r_x, msg.r_y, msg.r_z]

    def Euler_from_Quaternion(self, orientation_q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
        euler = [roll, pitch, yaw]
        return euler

    def Quaternion_from_Euler(self, roll, pich, yaw):
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pich, yaw)
        quaternion = [ox, oy, oz, ow]
        return quaternion

    def Create_Rotation_matrix(self, euler):
        (roll, pitch, yaw) = euler

        yawMatrix = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        pitchMatrix = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        rotation_matrix_A = np.dot(rollMatrix, pitchMatrix)
        rotation_matrix = np.dot(yawMatrix, rotation_matrix_A)
        # print("rotaion_matrix")
        # print(rotation_matrix, type(rotation_matrix))
        return rotation_matrix

    def Create_Rotation_matrix_T(self, euler):
        (roll, pitch, yaw) = euler

        yawMatrix = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        pitchMatrix = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        # print(yawMatrix.T, rollMatrix.T, pitchMatrix.T)

        rotation_matrix_A = np.dot(rollMatrix.T, pitchMatrix.T)
        rotation_matrix =np.dot(yawMatrix.T,rotation_matrix_A)

        print("rotaion_matrix")
        # print(rotation_matrix, type(rotation_matrix))
        return rotation_matrix

    def Marge_rota_trace(self, r_matrix, t_matrix):  # r_matrix(3,3,np.array), t_matrix(3,1,np.array)
        matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
        zero_one = np.array([[0., 0., 0., 1.]])
        matrix_4x4 = np.concatenate((matrix_3x4, zero_one), axis=0)
        # print(matrix_4x4)
        return matrix_4x4

    def Cul_matrix(self, ma_1=None, ma_2=None):
        result_matrix = np.dot(ma_1, ma_2)
        return result_matrix

    def Get_RPY_to_rotation_vector(self, matrix):
        # print(matrix.shape)
        r_11 = matrix[0, 0]
        r_21 = matrix[1, 0]
        r_31 = matrix[2, 0]
        r_32 = matrix[2, 1]
        r_33 = matrix[2, 2]
        t_ = list(matrix[:3, 3])
        roll = np.arctan(r_21 / r_11)
        pitch = np.arctan(-r_31 / np.sqrt((np.square(r_32)) + np.square(r_33)))
        yaw = np.arctan(r_32 / r_33)
        quaternion = self.Quaternion_from_Euler(roll, pitch, yaw)
        return t_, roll, pitch, yaw

    def send_pose(self, t_, quaternion):
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = t_[0]
        nav_goal.pose.position.y = t_[1]
        nav_goal.pose.position.z = t_[2]
        nav_goal.pose.orientation.x = quaternion[0]
        nav_goal.pose.orientation.y = quaternion[1]
        nav_goal.pose.orientation.z = quaternion[2]
        nav_goal.pose.orientation.w = quaternion[3]
        return nav_goal
        # print(nav_goal)
        # self.nav_pub.publish(nav_goal)


def main():
    rospy.init_node("calcul")
    cul = Mani_Calcul()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        cul.main()
        rate.sleep()


if __name__ == '__main__':
    main()

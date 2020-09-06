#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import math
# import util.util_funcs as uf


class Calcul_3D():
    def __init__(self):
        self.r_m_a_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self.mani_pose_pub = rospy.Publisher("/mani_pose/move_base_simple/goal", PoseStamped, queue_size=1)
        self.r_m_pose_pub = rospy.Publisher("/r_m_pose/move_base_simple/goal", PoseStamped, queue_size=1)
        self.nav_goal = PoseStamped()

        self.aruco_rv = [0, 0, 0]
        self.aruco_tv = np.zeros((1, 3))
        self.robot_orientation = []
        self.mani_orientation = []
        self.robot_pose = np.zeros((1, 3))
        self.mani_pose = np.zeros((1, 3))
        rospy.Subscriber('odom', Odometry, self.Robot_pose)
        # rospy.Subscriber(name='odom', data_class=Odometry, callback=self.Robot_pose)

        rospy.Subscriber('mani_pose', Pose, self.Mani_pose)

        rospy.Subscriber('aruco_msg', aruco_msgs, self.Aruco)

    def main(self):
        # robot
        robot_3d_matrix = self.Make_3d_matrix(self.robot_pose, self.robot_orientation)
        #mani
        mani_3d_matrix = self.Make_3d_matrix(self.mani_pose, self.mani_orientation)
        # aruco
        
        aruco_rotation = self.Create_Rotation_matrix(self.aruco_rv)
        aruco_3d_matrix = self.Marge_rota_trace(aruco_rotation, self.aruco_tv.T)
        mani_goal =self.sss(mani_3d_matrix)
        self.mani_pose_pub.publish(mani_goal)
        # print(self.aruco_rv)
        r_m_matrix = np.dot(robot_3d_matrix,mani_3d_matrix)
        r_m_goal =self.sss(r_m_matrix)
        self.r_m_pose_pub.publish(r_m_goal)

        m_a_matrix = np.dot(mani_3d_matrix,aruco_3d_matrix)

        r_m_a_matrix = np.dot(robot_3d_matrix,m_a_matrix)
        print("send goal published")
        r_a_goal =self.sss(r_m_a_matrix)
        # self.r_m_a_pub.publish(r_a_goal)

    def sss(self, matrix):
        t, quaternion = self.Get_RPY_to_rotation_vector(matrix)
        goal = self.send_pose(t, quaternion=quaternion)
        return goal
        # print(t,r,p,y)

    def Make_3d_matrix(self, pose, orientation):
        euler = self.Euler_from_Quaternion(orientation)
        rotaion = self.Create_Rotation_matrix(euler)
        matrix_3d = self.Marge_rota_trace(rotaion, pose.T)
        return matrix_3d

    def Robot_pose(self, msg):
        # tf.transforamtions.getOrigin(msg.pose.pose.position)
        self.robot_pose = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])
        self.robot_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w]
        msg.pose.pose.position.x = 0

        # print("get robot")
    def Mani_pose(self, msg):

        self.mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]])
        self.mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def Aruco(self, msg):
        self.aruco_tv = np.array([[msg.t_z-0.25, msg.t_x, msg.t_y]])
        self.aruco_rv = [msg.r_x, 0, msg.r_y]

        br = tf.TransformBroadcaster()
        br.sendTransform((msg.t_z, -msg.t_x, -msg.t_y),
                     tf.transformations.quaternion_from_euler(msg.r_z,-msg.r_x, msg.r_y),
                     rospy.Time.now(),
                     "arucopose",
                     "link5")


        # yawMatrix = np.array([
        #     [math.cos(msg.r_x), -math.sin(msg.r_x), 0],
        #     [math.sin(msg.r_x), math.cos(msg.r_x), 0],
        #     [0, 0, 1]
        # ])

        # rollMatrix = np.array([
        #     [1, 0, 0],
        #     [0, math.cos(msg.r_y), -math.sin(msg.r_y)],
        #     [0, math.sin(msg.r_y), math.cos(msg.r_y)]
        # ])

        # self.rot = np.dot(yawMatrix,rollMatrix)
        # print(self.rot)

    def Euler_from_Quaternion(self, orientation_q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
        

        euler = [yaw, pitch, roll]
        euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]

        return euler

    def Quaternion_from_Euler(self, yaw, pitch, roll):
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(yaw,pitch, yaw)
        quaternion = [ox, oy, oz, ow]
        return quaternion

    def Create_Rotation_matrix(self, euler):
        (yaw, pitch, roll) = euler

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

        rotation_matrix_A = np.dot(pitchMatrix, rollMatrix)
        rotation_matrix = np.dot(yawMatrix, rotation_matrix_A)

        return rotation_matrix

    def Create_Rotation_matrix_T(self, euler):
        (yaw, pitch, roll) = euler

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

        rotation_matrix_A = np.dot(pitchMatrix.T, rollMatrix.T)
        rotation_matrix = np.dot(yawMatrix.T, rotation_matrix_A)

        # print("rotaion_matrix")
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
        r_11 = matrix[0, 0]  ## cos(yaw)cos(pitch)
        r_21 = matrix[1, 0]  ## sin(yaw)cos(pitch)
        r_31 = matrix[2, 0]  ## -sin(pitch)
        r_32 = matrix[2, 1]  ## cos(pitch)sin(roll)
        r_33 = matrix[2, 2]  ## cos(pitch)cos(roll)
        t_ = list(matrix[:3, 3])
        yaw = np.arctan2(r_21, r_11)
        pitch = np.arctan2(-r_31, np.sqrt((np.square(r_32)) + np.square(r_33)))
        roll = np.arctan2(r_32, r_33)
        euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]
        # print("euler2", yaw, pitch, roll)
        quaternion = self.Quaternion_from_Euler(yaw, pitch, roll)
        # print("qu2", quaternion)
        return t_, quaternion

    def send_pose(self, t_, quaternion):
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = t_[0]
        nav_goal.pose.position.y = t_[1]
        nav_goal.pose.position.z = 0
        nav_goal.pose.orientation.x = quaternion[0]
        nav_goal.pose.orientation.y = quaternion[1]
        nav_goal.pose.orientation.z = quaternion[2]
        nav_goal.pose.orientation.w = quaternion[3]
        return nav_goal
        # print(nav_goal)
        # self.nav_pub.publish(nav_goal)


def main():
    rospy.init_node("calcul")
    count = 0
    cul = Calcul_3D()
    rate = rospy.Rate(50)
    prepare_publish_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        prepare_publish_rate.sleep()
        cul.main()
        count += 1
        rate.sleep()
        # if count == 1:
        #     print("published! breaking while function")
            # break

if __name__ == '__main__':
    main()
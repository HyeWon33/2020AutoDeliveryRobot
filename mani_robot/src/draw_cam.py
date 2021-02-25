#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from mani_robot.msg import aruco_msgs
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf
import numpy as np
import math

"""
카메라센서의 tf를 생성
메니의 좌표를 받아오고 메니의 좌표로부터 카메라까지의 떨어진 거리를 입력하여 생성 
(메니의 좌표는 집게 중앙을 기준으로 함)
"""
def main():
    rospy.init_node("draw_cam")
    rospy.Subscriber('mani_pose', Pose, get_mani_pose) # mani 노드에서부터 로봇(base_link)에서 본 mani 좌표를 sub하게됨
    rospy.loginfo_once("CAM_OK")
    rospy.spin()


def get_mani_pose(msg):
    mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]]) # mani의 위치정보
    mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] # mani의 회전정보(quaternion)
    r, p, y = tf.transformations.euler_from_quaternion(mani_orientation) # 회전정보를 euler로 변환
    mani_3d_matrix = make_3d_matrix(mani_pose, mani_orientation)    # mani의 transformation matrix 생성
    cam_rgb_pose = np.array([[-0.06, 0, 0.04]])                     # mani의 위치(gripper의 중심)에서의 카메라 위치
    cam_rgb_rpy = [0, 0, 0]                                         # 카메라의 위전
    cam_rotation = create_rotation_matrix(cam_rgb_rpy)              # 카메라의 rotation matrix 생성
    cam_3d_matrix = make_transformation_matrix(cam_rotation, cam_rgb_pose.T) # 카메라의 transformation matrix 생성
    m_c_matrix = np.dot(mani_3d_matrix, cam_3d_matrix)              # T(mani) * T(camera) = 로봇에서(base_link)에서 본 카메라 좌표
    t, q = get_rpt_to_rotation_vector(m_c_matrix)                   # 로봇에서(base_link)에서 본 카메라 좌표에서 transration, rotation을 추출
    br = tf.TransformBroadcaster()                                  # tf broadcaster 생성
    br.sendTransform(t, q, rospy.Time.now(), "tb3_1/cam_test", "tb3_1/base_link") # base_link에서 카메라의 tf 생성
    # base_link에서 mani의 tf 생성
    br.sendTransform((msg.position.x, msg.position.y, msg.position.z),      
                     (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                     rospy.Time.now(),
                     "tb3_1/mani_pose",
                     "tb3_1/base_link")


# rotation matrix 생성 및 3d transformation matrix 생성
def make_3d_matrix(pose, orientation):
    euler = euler_from_quaternion(orientation)
    rotation = create_rotation_matrix(euler)
    matrix_3d = make_transformation_matrix(rotation, pose.T)
    return matrix_3d


# 쿼터니언을 오일러로 변환
def euler_from_quaternion(orientation_q):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
    euler = [yaw, pitch, roll]
    euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]
    return euler


# 오일러를 쿼터니언으로 변환
def quaternion_from_euler(yaw, pitch, roll):
    ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    quaternion = [ox, oy, oz, ow]
    return quaternion


# rotation matrix 생성
def create_rotation_matrix(euler):
    (yaw, pitch, roll) = euler

    yaw_matrix = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_matrix = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    roll_matrix = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    rotation_matrix_a = np.dot(pitch_matrix, roll_matrix)
    rotation_matrix = np.dot(yaw_matrix, rotation_matrix_a)

    return rotation_matrix


# transformation matrix 생성
def make_transformation_matrix(r_matrix, t_matrix):
    matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
    zero_one = np.array([[0., 0., 0., 1.]])
    matrix_4x4 = np.concatenate((matrix_3x4, zero_one), axis=0)
    return matrix_4x4


# transformation matrix 에서 quaternion 과 translation 추출
def get_rpt_to_rotation_vector(matrix):
    r_11 = matrix[0, 0]  # cos(yaw)cos(pitch)
    r_21 = matrix[1, 0]  # sin(yaw)cos(pitch)
    r_31 = matrix[2, 0]  # -sin(pitch)
    r_32 = matrix[2, 1]  # cos(pitch)sin(roll)
    r_33 = matrix[2, 2]  # cos(pitch)cos(roll)
    translation = list(matrix[:3, 3])
    yaw = np.arctan2(r_21, r_11)
    pitch = np.arctan2(-r_31, np.sqrt((np.square(r_32)) + np.square(r_33)))
    roll = np.arctan2(r_32, r_33)
    euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]
    quaternion = quaternion_from_euler(yaw, pitch, roll)
    return translation, quaternion


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

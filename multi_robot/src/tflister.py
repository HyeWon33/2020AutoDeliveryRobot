#!/usr/bin/env python  
import roslib
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

def main():
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print("trans,rot",trans,rot)
        # trans = np.array([trans])
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw+np.pi/2)

        yawMatrix = np.array([
            [math.cos(yaw), -math.sin(yaw), trans[0]],
            [math.sin(yaw), math.cos(yaw), trans[1]],
            [0, 0, 1]
        ])
        wMatrix = np.array([
            [math.cos(np.pi), -math.sin(np.pi), 0],
            [math.sin(np.pi), math.cos(np.pi), -0.25],
            [0, 0, 1]
        ])
        m = np.dot(yawMatrix,wMatrix)
        x, y = m[:2, 2]
        cosine, sine = m[:2, 0]
        theta = np.arctan2(sine, cosine)
        


        # goal = send_pose(trans,rot)
        
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = x
        nav_goal.pose.position.y = y
        nav_goal.pose.position.z = 0
        nav_goal.pose.orientation.x = 0
        nav_goal.pose.orientation.y = 0
        nav_goal.pose.orientation.z = oz
        nav_goal.pose.orientation.w = ow
        turtle_vel.publish(nav_goal)

        rate.sleep()

def sss( matrix):
    t, quaternion = Get_RPY_to_rotation_vector(matrix)
    goal = send_pose(t, quaternion=quaternion)
    return goal

def send_pose( t_, quaternion):
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
    # print(t,r,p,y)

def Make_3d_matrix(pose, orientation):
    euler = Euler_from_Quaternion(orientation)
    # print(euler)
    rotaion = Create_Rotation_matrix(euler)
    # print(rotaion)
    matrix_3d = Marge_rota_trace(rotaion, pose.T)
    return matrix_3d
def Euler_from_Quaternion(orientation_q):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
    

    euler = [yaw, pitch, roll]
    euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]

    return euler

def Quaternion_from_Euler(yaw, pitch, roll):
    ox, oy, oz, ow = tf.transformations.quaternion_from_euler(yaw,pitch, yaw)
    quaternion = [ox, oy, oz, ow]
    return quaternion

def Create_Rotation_matrix( euler):
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
        [0, math.cos(roll), -math.sin(roll)]])

    rotation_matrix_A = np.dot(pitchMatrix, rollMatrix)
    rotation_matrix = np.dot(yawMatrix, rotation_matrix_A)

    return rotation_matrix

def Marge_rota_trace(r_matrix, t_matrix):  # r_matrix(3,3,np.array), t_matrix(3,1,np.array)
    print(r_matrix,t_matrix)
    matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
    zero_one = np.array([[0., 0., 0., 1.]])
    matrix_4x4 = np.concatenate((matrix_3x4, zero_one), axis=0)
    # print(matrix_4x4)
    return matrix_4x4

def Cul_matrix( ma_1=None, ma_2=None):
    result_matrix = np.dot(ma_1, ma_2)
    return result_matrix

def Get_RPY_to_rotation_vector( matrix):
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
    quaternion = Quaternion_from_Euler(yaw, pitch, roll)
    # print("qu2", quaternion)
    return t_, quaternion


if __name__ == '__main__':
    main()
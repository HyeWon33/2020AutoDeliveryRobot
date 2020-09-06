#!/usr/bin/env python
import rospy
from multi_robot.msg import aruco_msgs
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf
import numpy as np
import math



def get_mani_pose( msg):

    mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]])
    mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    r, p, y = tf.transformations.euler_from_quaternion(mani_orientation)
    print(np.around([r,p,y],3))
    mani_3d_matrix = Make_3d_matrix(mani_pose, mani_orientation)
    cam_rgb_pose = np.array([[-0.08,0,0.04]])
    cam_rgb_rpy = [-np.pi/2,0,-np.pi/2]
    cam_rotation = Create_Rotation_matrix(cam_rgb_rpy)
    cam_3d_matrix = Marge_rota_trace(cam_rotation,cam_rgb_pose.T)
    m_c_matrix = np.dot(mani_3d_matrix,cam_3d_matrix)
    t,q = Get_RPY_to_rotation_vector(m_c_matrix)
    # x,y,z,w = tf.transformations.quaternion_from_euler(r-np.pi/2,p,y-np.pi/2)
    # print(np.around([x,y,z,w],3))
    br = tf.TransformBroadcaster()
    br.sendTransform((t),
                (q),
                rospy.Time.now(),
                "cam_rgb_test",
                "base_link")


def main():
    rospy.init_node("draw_rgb")
    listener = tf.TransformListener()
    
    # while not rospy.is_shutdown():
    #     try:
    #       (trans,rot) = listener.lookupTransform('/map', '/link5', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    rospy.Subscriber('mani_pose', Pose, get_mani_pose)
        # br = tf.TransformBroadcaster()
        # br.sendTransform((trans[0]+0.5, trans[1], trans[2]+0.05),
        #             (rot[0], rot[1], rot[2], rot[3]),
        #             rospy.Time.now(),
        #             "mani_pose",
        #             "link5")
    rospy.spin()

def Make_3d_matrix(pose, orientation):
    euler = Euler_from_Quaternion(orientation)
    rotaion = Create_Rotation_matrix(euler)
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
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    rotation_matrix_A = np.dot(pitchMatrix, rollMatrix)
    rotation_matrix = np.dot(yawMatrix, rotation_matrix_A)

    return rotation_matrix

def Marge_rota_trace(r_matrix, t_matrix):  # r_matrix(3,3,np.array), t_matrix(3,1,np.array)
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


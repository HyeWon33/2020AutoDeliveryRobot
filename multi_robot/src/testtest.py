#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import math
# import util.util_funcs as uf


class Mani_Calcul():
    def __init__(self):
        self.r_m_a_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
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

        # print(self.aruco_rv)

        m_a_matrix = np.dot(mani_3d_matrix,aruco_3d_matrix)
        print(self.robot_pose, self.robot_orientation)
        print(self.aruco_tv, self.aruco_rv)
        r_m_a_matrix = np.dot(robot_3d_matrix,m_a_matrix)
        print("send goal published")
        r_a_goal =self.sss(r_m_a_matrix)
        self.r_m_a_pub.publish(r_a_goal)

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
        self.robot_pose = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])
        self.robot_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w]
        msg.pose.pose.position.x = 0

        # print("get robot")
    def Mani_pose(self, msg):

        self.mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]])
        self.mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def Aruco(self, msg):
        self.aruco_tv = np.array([[msg.t_z-0.25, msg.t_y, 0]])
        self.aruco_rv = [msg.r_z, -msg.r_y, 0]



    def Euler_from_Quaternion(self, orientation_q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
        euler = [yaw, pitch, roll]
        euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]

        return euler

    def Quaternion_from_Euler(self, yaw, pitch, roll):
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(0,0, yaw)
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
    cul = Mani_Calcul()
    rate = rospy.Rate(50)
    prepare_publish_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        prepare_publish_rate.sleep()
        cul.main()
        count += 1
        rate.sleep()
        if count == 1:
            print("published! breaking while function")
            break

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from nav_msgs.msg import Odometry
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import math
import cv2
# import util.util_funcs as uf


class Mani_Calcul():
    def __init__(self):
        # rospy.Subscriber(name='odom', data_class=Odometry, callback=self.Robot_pose)



        self.aruco_rv = None
        # self.aruco_rv2 = [0, 0, 0]
        self.aruco_tv = None
        # self.rm = np.zeros((3,3))
        self.robot_pose = np.zeros((1, 3))
        self.robot_orientation =[]
        self.mani_pose = np.zeros((1, 3))
        self.mani_orientation = []
        # self.aruco_rv_list = []
        # self.rv_aver = np.zeros((3,))
        # self.count = 0
        rospy.Subscriber('odom', Odometry, self.Robot_pose)

        rospy.Subscriber('mani_pose', Pose, self.Mani_pose)

        rospy.Subscriber('aruco_msg', aruco_msgs, self.get_aruco_msg)


    def get_sub_param(self):
        return self.aruco_rv, self.aruco_tv, self.robot_pose, self.robot_orientation, \
               self.mani_pose, self.mani_orientation

    # def calculate_goal_pose(self):
    #     # robot
    #     print("subscribed")
    #     robot_3d_matrix = self.Make_3d_matrix(self.robot_pose, self.robot_orientation)
    #     #mani
    #     mani_3d_matrix = self.Make_3d_matrix(self.mani_pose, self.mani_orientation)
    #     # aruco
    #     print(len(self.aruco_rv_list))
    #     aruco_rotation = self.Create_Rotation_matrix_T(self.rv_aver)
    #     aruco_3d_matrix = self.Marge_rota_trace(aruco_rotation, self.aruco_tv.T)
    #     print(aruco_3d_matrix)
    #
    #     # print(self.aruco_rv)
    #
    #     m_a_matrix = np.dot(mani_3d_matrix,aruco_3d_matrix)
    #     # print(self.robot_pose, self.robot_orientation)
    #     # print(self.aruco_tv, self.aruco_rv)
    #
    #     r_m_a_matrix = np.dot(robot_3d_matrix,m_a_matrix)
    #
    #
    #
    #     print("send goal published")
    #     r_a_goal =self.sss(r_m_a_matrix)
    #     # self.r_m_a_pub.publish(r_a_goal)
    #
    #
    #
    # def sss(self, matrix):
    #     t, quaternion = self.Get_RPY_to_rotation_vector(matrix)
    #     goal = self.send_pose(t, quaternion=quaternion)
    #     return goal
    #     # print(t,r,p,y)
    #
    # def Make_3d_matrix(self, pose, orientation):
    #     euler = self.Euler_from_Quaternion(orientation)
    #     rotaion = self.Create_Rotation_matrix(euler)
    #     matrix_3d = self.Marge_rota_trace(rotaion, pose.T)
    #     return matrix_3d
    #
    def Robot_pose(self, msg):
        self.robot_pose = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])
        self.robot_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w]


        # print("get robot")
    def Mani_pose(self, msg):

        self.mani_pose = np.array([[msg.position.x, msg.position.y, msg.position.z]])
        self.mani_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def get_aruco_msg(self, msg):
        self.aruco_tv = np.array([[msg.t_z-0.25, -msg.t_y, msg.t_x]])
        self.aruco_rv = [msg.r_z, -msg.r_y, msg.r_x]
        # self.aruco_rv2 = [msg.r_z, msg.r_y, msg.r_x]
        # self.aruco_rv_list = []
        # print(self.aruco_rv)
    #
    #
    #     # print("rv average in Aruco : ", self.rv_aver)
    #     # print("rv",self.aruco_rv)
    #     # print(len(self.aruco_rv_list))
    #     # if len(self.aruco_rv_list) > 30:
    #     #     self.aruco_rv_list.pop(0)
    #     #     self.aruco_rv_list.sort()
    #     #     self.aruco_rv_list2 = self.aruco_rv_list[5:25]
    #     #     self.rv_aver = self.average(self.aruco_rv_list2)
    #
    #     # r = np.array([self.aruco_rv])
    #     # # print(r.T.shape)
    #     # self.rm, ja = cv2.Rodrigues(r.T)
    #     # print(self.rm)
    #
    # def make_rv_list(self, rvs):
    #     final_rvs = list()
    #     for single_rv in rvs:
    #         final_rvs.append(single_rv)
    #
    # def average(self,lists):
    #     list_sum = np.array([0,0,0])
    #     list_ar = np.array(lists)
    #     # print(list_ar.shape)
    #     average = np.mean(list_ar,axis=0)
    #     # for l in list_ar:
    #     #     list_sum = list_sum + l
    #     # print(list_sum)
    #     # list_sum = list_sum/20
    #     # print(average.shape)
    #     return average



class CalcPose():
    def __init__(self, rv_mean, aruco_tv, robot_pose, robot_orientation, mani_pose, mani_orientation):
        self.rv_mean = rv_mean
        self.aruco_tv = aruco_tv
        self.robot_pose = robot_pose
        self.robot_orientation = robot_orientation
        self.mani_pose = mani_pose
        self.mani_orientation = mani_orientation
        self.r_m_a_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        # self.nav_goal = PoseStamped()

    def calculate_goal_pose(self):
        # robot
        print("subscribed")
        print("ro_p, robot_o : ",self.robot_pose, self.robot_orientation)
        robot_3d_matrix = self.Make_3d_matrix(self.robot_pose, self.robot_orientation)
        # mani
        print("mani_or, mani_pose : ", self.mani_orientation, self.mani_pose)
        mani_3d_matrix = self.Make_3d_matrix(self.mani_pose, self.mani_orientation)
        # aruco
        aruco_rotation = self.Create_Rotation_matrix_T(self.rv_mean)
        aruco_3d_matrix = self.Marge_rota_trace(aruco_rotation, self.aruco_tv.T)
        print(aruco_3d_matrix)

        # print(self.aruco_rv)

        m_a_matrix = np.dot(mani_3d_matrix, aruco_3d_matrix)
        # print(self.robot_pose, self.robot_orientation)
        # print(self.aruco_tv, self.aruco_rv)

        r_m_a_matrix = np.dot(robot_3d_matrix, m_a_matrix)

        print("send goal published")
        r_a_goal = self.sss(r_m_a_matrix)
        print(r_a_goal)
        rate = rospy.Rate(1)
        rate.sleep()
        self.r_m_a_pub.publish(r_a_goal)

    def Euler_from_Quaternion(self, orientation_q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_q)
        euler = [yaw, pitch, roll]
        # euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]
        return euler

    def Quaternion_from_Euler(self, yaw, pitch, roll):
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll,pitch, yaw)
        quaternion = [0, 0, oz, ow]
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


def main():
    rospy.init_node("calcul")
    # count = 0
    cul = Mani_Calcul()
    rate = rospy.Rate(100)
    prepare_publish_rate = rospy.Rate(0.2)
    rv_total = list()
    while not rospy.is_shutdown():
        # prepare_publish_rate.sleep()
        aruco_rv, aruco_tv, robot_pose, robot_orientation, mani_pose, mani_orientation = cul.get_sub_param()
        if aruco_rv is not None:
            rv_total.append(aruco_rv)
        # count += 1
        rate.sleep()
        print(rv_total)
        # print("processing step : ", count)
        if len(rv_total) == 30:
            rv_to = np.array(rv_total)
            print(rv_to.shape)
            rv_tot = np.sort(rv_to)
            print(rv_tot.shape)
            rv_cc = rv_tot[10:20]
            rv_mean = np.mean(np.array(rv_cc), axis=0)
            # print(rv_mean)
            calpose = CalcPose(aruco_rv, aruco_tv, robot_pose, robot_orientation, mani_pose, mani_orientation)
            calpose.calculate_goal_pose()
            prepare_publish_rate.sleep()
            print("pub finished")
            rv_total = []
            break

        # if count == 1:
        #     print("published! breaking while function")
        #     break

if __name__ == '__main__':
    main()

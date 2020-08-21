#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from multi_robot.msg import aruco_msgs
import numpy as np
import tf
import math

class Mani_Calcul():
    def __init__(self):
        self.cul_pub = rospy.Publisher("cul",Pose,queue_size=10)
        self.pub_pose = Pose()
        self.mani = rospy.Subscriber('mani_pose',Pose,self.Manipose)
        self.aruco = rospy.Subscriber('rvecs_msg',aruco_msgs,self.Aruco)    


    def Manipose(self, msg):
        print(msg.position)
        self.pose= [msg.position.x,msg.position.y,msg.position.z]
        self.orientation = msg.orientation
        orientation_q = self.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.make_matrix(roll,pitch,yaw)
        
        
    def Aruco(self,msg):
        self.aruco = np.array([[msg.t_z,-msg.t_x,msg.t_y,1]])


    # def transe_quaternion(self, matrix):
    #     goal_x, goal_y, goal_rad = self.get_params(matrix)
    #     goal_deg = (goal_rad * (180 / np.pi))
    #     # rospy.loginfo("goal_x : %f , goal_y : %f , goal_theta : %f" % (goal_x, goal_y, goal_deg))
    #     ox, oy, ow, oz = tf.transformations.quaternion_from_euler(goal_x, goal_y, goal_rad)
    #     self.quaternion = [goal_x, goal_y, ow, oz]

    def make_matrix(self,roll,pitch,yaw):

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

        self.rotation_matrix = np.dot(yawMatrix,pitchMatrix,rollMatrix)
        #print(self.rotation_matrix)
        self.make_4()

    def make_4(self):
        # self.rotation_matrix = np.array(self.rotation_matrix)

        rotation_matrix=self.rotation_matrix
        print(np.array(self.pose))
        t_matrix = np.array([self.pose]).T

        ds = np.concatenate((rotation_matrix,t_matrix),axis=1)

        zero_one = np.array([[0.,0.,0.,1.]])
 
        sss = np.concatenate((ds,zero_one),axis=0)
        print(np.around(sss,4))
        # print(self.rotation_matrix)
        c=(self.cul_matrix(sss,self.aruco.T)).T
        print(type(c),c.shape)
        self.pub_pose.position.x = c[0,0]
        self.pub_pose.position.y = c[0,1]
        self.pub_pose.position.z = c[0,2]
        
        self.cul_pub.publish(self.pub_pose)

    def cul_matrix(self, ma_1=None, ma_2=None):
        print("1",ma_1)
        print("\n2",ma_2)
        result_matrix = np.dot(ma_1, ma_2)
        return result_matrix    



def main():
    rospy.init_node("mani_calcul")
    cul = Mani_Calcul()
    rospy.spin()

if __name__ == '__main__':
    main()
    
    



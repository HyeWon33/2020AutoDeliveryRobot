#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import numpy as np
import tf
import random


class Random_Pose():
    def __init__(self):
        self.nav_pub = rospy.Publisher("tb3_1/move_base_simple/goal",PoseStamped,queue_size=10)
        self.nav_goal = PoseStamped()
        self.sub = rospy.Subscriber('tb3_0/odom', Odometry, self.get_rotation)
        self.ddd = rospy.Subscriber('/call_frame',Int32,self.Start)
        self.a = 0
        self.start = 0
        self.quaternion = None

    def Start(self, msg):
        #print(msg.data)
        self.start = msg.data
        rospy.loginfo(self.start)

    def get_rotation(self, msg):
        pose_p = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.Cul(pose_p.x,pose_p.y,yaw)
        
        
    def Cul(self,x,y,theta):
        
        #print(x,y,theta* (180 / np.pi))      
        self.main_matrix = self.make_matrix(x,y,theta)
        base_matrix = self.make_matrix(0.4,0,0)

        zero_tp = self.cul_matrix(self.main_matrix)
        move_tp = self.cul_matrix(self.main_matrix,base_matrix)
        zero_move_tp =self.cul_matrix(zero_tp,base_matrix)
        self.transe_quaternion(move_tp)
        #print(goal_x,goal_y,goal_deg)
        #print(self.quaternion)
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = self.quaternion[0]
        self.nav_goal.pose.position.y = self.quaternion[1]
        self.nav_goal.pose.orientation.z = self.quaternion[2]
        self.nav_goal.pose.orientation.w = self.quaternion[3]

        if self.start ==1:
            self.nav_pub.publish(self.nav_goal)

    def make_matrix(self,x,y,theta):
        print("x:{}\ny:{}\ntheta:{}".format(x,y,theta* (180 / np.pi)))
        matrix = np.array([[np.cos(theta), -np.sin(theta), x],
                           [np.sin(theta), np.cos(theta), y],
                           [0, 0, 1]])
        return matrix

    def cul_matrix(self,ma_1=None,ma_2=None):
        zero_matrix = self.make_matrix(0,0,0)
        if ma_2 is None:
            
            result_matrix=np.dot(zero_matrix,ma_1)
        else :
            result_matrix=np.dot(ma_1,ma_2)
        return result_matrix
    def transe_quaternion(self,matrix):
        goal_x ,goal_y,goal_rad = self.get_params(matrix)
        goal_deg = (goal_rad * (180 / np.pi))
        print("goal_x:{}\ngoal_y:{}\ngoal_theta:{}".format(goal_x,goal_y,goal_deg))
        ox,oy,ow,oz = tf.transformations.quaternion_from_euler(goal_x, goal_y, goal_rad)
        self.quaternion = [goal_x,goal_y,ow,oz]

    def get_params(self,matrix):
        x, y = matrix[:2, 2]
        theta = np.arccos(matrix[0,0])
        #cosine, sine = matrix[:2, 0]
        #theta = np.arctan2(sine, cosine)
        return x, y, theta

def main():
    rospy.init_node("call_robot2")
    rp = Random_Pose()
    rospy.spin()


if __name__ == '__main__':
    main()

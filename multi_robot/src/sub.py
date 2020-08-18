#!/usr/bin/env python
import rospy
from multi_robot.msg import aruco_msgs
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2

class test_sub():
	def __init__(self):

		self.aruco_sub = rospy.Subscriber('rvecs_msg', aruco_msgs, self.callback1)

	def callback1(self,aruco_data):
		
		rospy.loginfo("\n id:%d\n r_x : %.3f\n r_y : %.3f\n r_z : %.3f\n t_x : %.3f\n t_y : %.3f\n t_z : %.3f\n x : %.3f\n y : %.3f \n z : %.3f" % (aruco_data.id,aruco_data.r_x, aruco_data.r_y, aruco_data.r_z,
		aruco_data.t_x, aruco_data.t_y, aruco_data.t_z,aruco_data.x,aruco_data.y,aruco_data.z))
		#rospy.loginfo("t_x:%f, t_y:%f, t_z:%f" % (aruco_data.t_x, aruco_data.t_y, aruco_data.t_z))
		# rvecs_list = [aruco_data.r_x, aruco_data.r_y, aruco_data.r_z]
		# tvecs_list = [aruco_data.t_x, aruco_data.t_y, aruco_data.t_z]
		rvecs = np.array([aruco_data.r_x,aruco_data.r_y,aruco_data.r_z])
		tvecs = np.array([aruco_data.t_x,aruco_data.t_y,aruco_data.t_z])
		#rospy.loginfo(rvecs)
		
		rotation_matrix = np.zeros((3,3))
		cv2.Rodrigues(rvecs, rotation_matrix)
		camera_matrix = np.dot(rotation_matrix.T , (tvecs * -1))
		camera_matrix = np.squeeze(camera_matrix)
		rospy.loginfo(camera_matrix)
		


                # rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                # rvecs_temp = np.squeeze(rvecs)
                # rotation_matrix = np.zeros((3,3))
                # cv2.Rodrigues(rvecs, rotation_matrix)
                # camera_matrix = rotation_matrix.T * tvecs * -1
                # camera_matrix = np.squeeze(camera_matrix)
		
		
def main():
	rospy.init_node('test_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

#!/usr/bin/env python
import rospy
from multi_robot.msg import Float32Multi
from multi_robot.msg import aruco_msgs
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

class test_sub():
	def __init__(self):

		# self.rvecs_sub = rospy.Subscriber('/rvecs_msg', numpy_msg(Float32Multi), self.callback1)
		self.rvecs_sub = rospy.Subscriber('rvecs_msg', aruco_msgs, self.callback1)
		# self.tvecs_sub = rospy.Subscriber('/tvecs_msg', numpy_msg(Float32Multi), self.callback2)
		# self.id_sub = rospy.Subscriber('/id_msg', numpy_msg(Float32Multi), self.callback3)

	def callback1(self,rvecs):
		
		rospy.loginfo("r_x:%f, r_y:%f, r_z:%f" % (rvecs.r_x, rvecs.r_y, rvecs.r_z))
		rospy.loginfo("t_x:%f, t_y:%f, t_z:%f" % (rvecs.t_x, rvecs.t_y, rvecs.t_z))
		rospy.loginfo("id:%d" % (rvecs.id))
		
		
		# print("R{}\n".format(revcs.y))
		# print("R{}\n".format(dd.z))

	def callback2(self, dd):
		print("D{}\n".format(dd.x))

	def callback3(self, dd):
		print("I{}\n".format(dd.x))
		


		
def main():
	rospy.init_node('test_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

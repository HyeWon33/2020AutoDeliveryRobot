#!/usr/bin/env python
import rospy
from multi_robot.msg import Float32Multi
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

class test_sub():
	def __init__(self):

		self.rvecs_sub = rospy.Subscriber('/rvecs_msg', numpy_msg(Float32Multi), self.callback1)
		self.tvecs_sub = rospy.Subscriber('/tvecs_msg', numpy_msg(Float32Multi), self.callback2)
		self.id_sub = rospy.Subscriber('/id_msg', numpy_msg(Float32Multi), self.callback3)

	def callback1(self,dd):
		
		print("R{}\n".format(dd.d))

	def callback2(self, dd):
		print("D{}\n".format(dd.d))

	def callback3(self, dd):
		print("I{}\n".format(dd.d))
		


		
def main():
	rospy.init_node('test_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

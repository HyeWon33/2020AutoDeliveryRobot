#!/usr/bin/env python
import rospy
from multi_robot.msg import aruco_msgs
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

class test_sub():
	def __init__(self):

		self.aruco_sub = rospy.Subscriber('aruco_msg', aruco_msgs, self.callback1)

	def callback1(self,aruco_data):
		
		rospy.loginfo("r_x:%f, r_y:%f, r_z:%f" % (aruco_data.r_x, aruco_data.r_y, aruco_data.r_z))
		rospy.loginfo("t_x:%f, t_y:%f, t_z:%f" % (aruco_data.t_x, aruco_data.t_y, aruco_data.t_z))
		rospy.loginfo("id:%d" % (aruco_data.id))
		
		
def main():
	rospy.init_node('test_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool, String
import numpy as np
import time


class SelfDrive:
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.fin_pub = rospy.Publisher("fin_move_close", Bool, queue_size=1)
        self.turtle_vel = Twist()
        self.bool = Bool()
        self.mode = "not"
        self.real_mode = 0

    def callback(self, msg):

        self.mode = msg.data

        print(self.mode)

    def act(self):
        rospy.Subscriber('start_move_closed', String, self.callback)
        rate = rospy.Rate(1)
        rospy.loginfo(self.mode)
        if self.mode =="back":
            for n in range(5):
                self.goturn(-0.05, 0)
                rate.sleep()
            self.goturn(0, 0)
            rate.sleep()
            self.bool.data = True
            self.fin_pub.publish(self.bool)
            rate.sleep()
            rospy.loginfo("move_fin")

        if self.mode =="front":
            for n in range(4):
                self.goturn(0.05, 0)
                rate.sleep()
            self.goturn(0, 0)
            rate.sleep()
            self.bool.data = True
            self.fin_pub.publish(self.bool)
            rate.sleep()
            rospy.loginfo("move_fin")
        elif self.mode =="not":
            self.bool.data = False
            self.fin_pub.publish(self.bool)

    def goturn(self, x, z):

        self.turtle_vel.linear.x = x
        self.turtle_vel.angular.z = z
        self.publisher.publish(self.turtle_vel)


def main():
    rospy.init_node('move_close')

    driver = SelfDrive()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        driver.act()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
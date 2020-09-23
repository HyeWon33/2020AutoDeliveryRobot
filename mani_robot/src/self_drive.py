#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import numpy as np


class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.turtle_vel = Twist()
        self.mode = 0

    def Stop(self, msg):
        self.mode = msg.data


    def lds_callback(self, scan):
        if self.mode == 0:
            avg = list()
            for n in range(35):
                # print(n)
                a = self.average(scan.ranges[10*n:10*(n+1)])
                avg.append(a)

            if avg[0] < 0.70:
                self.goturn(0, -1)
            elif avg[1] < 0.70:
                self.goturn(0, -1)
            elif avg[2] < 0.70:
                self.goturn(0, -1)
            elif avg[3] < 0.70:
                self.goturn(0, -1)
            elif avg[4] < 0.70:
                self.goturn(0, -1)
            elif avg[5] < 0.70:
                self.goturn(0, -1)
            elif avg[6] < 0.70:
                self.goturn(0, -1)
            elif avg[7] < 0.70:
                self.goturn(0, -1)
            elif avg[8] < 0.70:
                self.goturn(0, -1)
    

            elif avg[-1] < 0.70:
                self.goturn(0, 1)
            elif avg[-2] < 0.70:
                self.goturn(0, 1)
            elif avg[-3] < 0.70:
                self.goturn(0, 1)
            elif avg[-4] < 0.70:
                self.goturn(0, 1)
            elif avg[-5] < 0.70:
                self.goturn(0, 1)
            elif avg[-6] < 0.70:
                self.goturn(0, 1)
            elif avg[-7] < 0.70:
                self.goturn(0, 1)
            elif avg[-8] < 0.70:
                self.goturn(0, 1)
            else:
                self.goturn(0.24, 0)
        elif self.mode == 1:
            self.goturn(0, 0)
            rate = rospy.Rate(0.5)
            rate.sleep()



    def goturn(self, x, z):
        self.turtle_vel.linear.x = x
        self.turtle_vel.angular.z = z
        print(x,z)
        self.publisher.publish(self.turtle_vel)

    def average(self, a):
        avg = 0
        sum_ = 0
        count_ = 0
        for num in a:
            if num != np.inf:
                sum_ = sum_ + num
                count_ = count_ + 1
        # print(count_)
        try:
            avg = sum_ / count_
        except ZeroDivisionError:
            avg = 10

        return avg


def main():
    rospy.init_node('self_drive')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('mode', Int32,
                                  lambda scan: driver.Stop(scan))
    

    subscriber = rospy.Subscriber('scan', LaserScan,
                                    lambda scan: driver.lds_callback(scan))

    rospy.spin()


if __name__ == "__main__":
    main()

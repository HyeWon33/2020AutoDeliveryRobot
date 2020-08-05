#! /home/j/.pyenv/versions/ros_py36/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import numpy as np


class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.stop_sub = rospy.Subscriber('/stop',Int32,self.Stop)
        self.turtle_vel = Twist()
        self.stop = 0
    def Stop(self,msg):
        self.stop = msg.data
    def lds_callback(self, scan):

        avg_right = self.average(scan.ranges[300:360])
        avg_left = self.average(scan.ranges[1:60])
        center = scan.ranges[335:360] + scan.ranges[0:15]
        avg_center = self.average(center)
        print(avg_right,avg_left)

        if self.stop == 1:
            self.goturn(0,0)
        else :
            if avg_right <1:
                self.goturn(0,1.2)
            elif  avg_left < 1:
                self.goturn(0, -1.2)
            else:
                self.goturn(0.2,0)       



    def goturn(self,x,z):
    
        self.turtle_vel.linear.x = x
        self.turtle_vel.angular.z = z
        #print(a*4)
        self.publisher.publish(self.turtle_vel)

    def average(self,a):
        avg = 0
        sum_ = 0
        count_ = 0
        for num in a:
            if num != np.inf:
                sum_ = sum_ + num
                count_ = count_ + 1
           # print(count_)
        try :
            avg = sum_ / count_
        except ZeroDivisionError:
            pass
            
        return avg
        
        
def main():
    rospy.init_node('self_drive')
    publisher = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    driver.rate = rospy.Rate(3.8)
    subscriber = rospy.Subscriber('/tb3_0/scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()

if __name__ == "__main__":
    main()
    
    

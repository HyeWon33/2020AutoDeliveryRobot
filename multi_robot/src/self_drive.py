#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import numpy as np


class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.stop_sub = rospy.Subscriber('/mode',Int32,self.Stop)
        self.turtle_vel = Twist()
        self.stop = 0
    def Stop(self,msg):
        self.stop = msg.data
        rospy.loginfo(self.stop)
    def lds_callback(self, scan):

        # if self.stop == 0:

        avg_right = self.average(scan.ranges[300:350])
        avg_left = self.average(scan.ranges[10:60])
        center = scan.ranges[350:360] + scan.ranges[0:10]
        # print(len(center))
        avg_center = self.average(center)
        # print(avg_left,avg_center,avg_right)

        if self.stop == 1:
            self.goturn(0,0)
        elif self.stop == 0  :
            if avg_right <0.6 and avg_left < 0.6 and avg_center <0.75:
                self.goturn(-0.1,1)
            elif avg_right <0.9 :
                self.goturn(0,1.2)
            elif  avg_left < 0.9:
                self.goturn(0, -1.2)
            elif avg_center <0.75:
                self.goturn(-0.1,1.2)
            else:
                self.goturn(0.2,0)   

        else:
            pass



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
    publisher = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    driver.rate = rospy.Rate(3.8)
    subscriber = rospy.Subscriber('/tb3_1/scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()

if __name__ == "__main__":
    main()
    
    

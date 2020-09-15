#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool
import numpy as np
import time


class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.fin_pub = rospy.Publisher("/fin_move_close",Bool,queue_size=1)
        self.turtle_vel = Twist()
        self.stop = 0



    def callback(self, msg):

        mode = msg.data
        
        
        if mode == 3:
            start_time = time.time()
            while time.time() <=start_time+0.4:

                self.goturn(0.1,0)
            self.goturn(0,0)
            self.fin_pub.publish(True)
        else:
            self.fin_pub.publish(False)

        



    def goturn(self,x,z):
    
        self.turtle_vel.linear.x = x
        self.turtle_vel.angular.z = z
        #print(a*4)
        self.publisher.publish(self.turtle_vel)


        
        
def main():
    rospy.init_node('move_close')
    publisher = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    driver.rate = rospy.Rate(3.8)
    subscriber = rospy.Subscriber('/mode',Int32,
                                  lambda msg: driver.callback(msg))
    rospy.spin()

if __name__ == "__main__":
    main()
    
    

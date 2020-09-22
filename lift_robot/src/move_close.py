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
        self.subscriber = rospy.Subscriber('/mode',Int32,
                                  lambda msg: self.callback(msg))
        self.bool = Bool()
        self.mode = 0
        self.real_mode = 0
        self.count = 0



    def callback(self, msg):

        self.mode = msg.data

        print(self.mode)


    def act(self):
        print("1mode: ", self.mode)
        if self.mode == 4:
            rate = rospy.Rate(1)
            d_rate = rospy.Rate(0.5)
            # start_time = time.time()

            # while time.time() <=start_time+0.4:
            #     print(time.time(),start_time+0.4)
            #     self.goturn(0.1,0)
            # self.goturn(0,0)
            # self.fin_pub.publish(True)
            # rate = rospy.Rate(0.5)
            
            rate.sleep()
            self.goturn(-0.1,0)
            self.goturn(-0.1,0)
            self.goturn(-0.1,0)
            d_rate.sleep()
            self.goturn(0,0)
            rate.sleep()
            self.bool.data = True
            self.fin_pub.publish(self.bool)
            rate.sleep()
            rospy.loginfo("move_fin")
            # self.count += 1
            # rate.sleep()
                

    def deeely(self):
        print("bool:", self.bool)
        print("count:", self.count)
        if self.count != 0:
            
            self.bool.data = True
            
            self.fin_pub.publish(self.bool)
            
            # rospy.loginfo("move_fin")
            
        elif self.count == 0:
            self.bool.data = False
            # self.count =0

            self.fin_pub.publish(self.bool)

            


        



    def goturn(self,x,z):
        rate = rospy.Rate(1)
    
        self.turtle_vel.linear.x = x
        self.turtle_vel.angular.z = z
        #print(a*4)
        self.publisher.publish(self.turtle_vel)
        rate.sleep()


        
        
def main():
    rospy.init_node('move_close')
    publisher = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    
    while not rospy.is_shutdown():
        # rate = rospy.Rate(1)    
        
        # rate.sleep()
        driver.act()
        # rate.sleep()
        # driver.deeely()



if __name__ == "__main__":
    main()
    
    

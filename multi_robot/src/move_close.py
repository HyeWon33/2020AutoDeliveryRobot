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
        self.fin_pub = rospy.Publisher("/fin_move_close",Bool,queue_size=10)
        self.turtle_vel = Twist()
        self.subscriber = rospy.Subscriber('/mode',Int32,
                                  lambda msg: self.callback(msg))
        self.mode = 0
        self.real_mode = 0



    def callback(self, msg):

        self.mode = msg.data

        print(self.mode)


    def act(self, sleep_rate,q_sleep_rate):
        if self.mode == 3:
            # start_time = time.time()

            # while time.time() <=start_time+0.4:
            #     print(time.time(),start_time+0.4)
            #     self.goturn(0.1,0)
            # self.goturn(0,0)
            # self.fin_pub.publish(True)
            # rate = rospy.Rate(0.5)
            
            q_sleep_rate.sleep()
            self.goturn(0.1,0)
            sleep_rate.sleep()
            self.goturn(0,0)
            q_sleep_rate.sleep()
            self.fin_pub.publish(True)
            q_sleep_rate.sleep()
            rospy.loginfo("move_fin")
            self.mode = 4

        elif self.mode != 3:
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
    while not rospy.is_shutdown():
        rate = rospy.Rate(0.5)
        q_rate = rospy.Rate(10)

        
        driver.act(rate,q_rate)



if __name__ == "__main__":
    main()
    
    

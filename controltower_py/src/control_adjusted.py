#!/usr/bin/env python
import rospy
from aruco_pkg.msg import ArucoTriggerMsg
from aruco_pkg.msg import ArucoMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult


class Control:
    def __init__(self):
        rospy.init_node("control", anonymous=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.trigger_aruco = rospy.Publisher('arucoTrigger_msg', ArucoTriggerMsg, queue_size=1)
        self.servo_pub = rospy.Publisher('/servo', UInt16, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.aruco_sub = None
        self.marker_id = 0
        self.goal_status = 0
        self.vel = PoseStamped()
        self.ArucoTrigger = ArucoTriggerMsg()
        self.serv = UInt16()
        self.cmd = Twist()
        self.mode = "waiting_position"
        self.check = 0
        self.count = 0

    def main(self):
        while True:
            # 1. Program starting trigger subscribe
            self.starting_trigger()
            rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.GoalPoseCallback)
            # 2. if self.mode is "return_to_base", publish : "home" 
            if self.mode == "return_to_base":
                print(self.mode)
                self.ArucoTrigger.trigger = 0
                self.trigger_aruco.publish(self.ArucoTrigger)
                self.return_to_base()
                self.Delay(3)
                self.serv.data = 495
                self.servo_pub.publish(self.serv)
                self.Delay(5)
                self.mode = "marker_waiting_position"
                
            # 3. if goal reached, aruco trigger activate as 1
            elif self.mode == "marker_waiting_position":
                #self.mode = "start_detect_marker"
                self.check_reached()
                if self.check == 1:
                    self.mode = "start_detect_marker"
                    print(self.mode)
                    self.trigger_detect_marker()
                    self.mode = "detect_marker"
                    
            # 4. subscibe marker id 
            elif self.mode == "detect_marker":
                rospy.Subscriber('aruco_msg', ArucoMsg, self.MarkerIdCallback)
                self.Delay(3)
                print("marker id = ", self.marker_id)
                if self.marker_id == 1:
                    self.turn_right()
                    self.go_straight()
                    self.trigger_detect_marker()
                    self.marker_id = 200
                    print("command complete")
                    
                elif (self.marker_id >= 2 and self.marker_id <=4):
                    self.mode = "pickup_box"
                    
                elif self.marker_id == 5:
                    self.ArucoTrigger.trigger = 0
                    self.trigger_aruco.publish(self.ArucoTrigger)
                    self.mode = "return_to_base"
                    self.marker_id = 300
                    
                
                elif self.marker_id == 200:
                    #pass
                    self.go_straight()
                    self.trigger_detect_marker()
                    
                else:
                    pass
                    
            # 5. picking up box
            elif self.mode == "pickup_box":
                self.PickUp()
                self.goal_status = 0
                self.mode = "navigation"
            # 6. navigation start , publish : "dest1"
            elif self.mode == "navigation":
                self.ArucoTrigger.trigger = 0
                self.trigger_aruco.publish(self.ArucoTrigger)
                self.Navigation()
                self.mode = "pickdown_box"
            # 7. if goal reached, pickdown
            elif (self.goal_status == 3 or self.goal_status == 4) and self.mode == "pickdown_box":
                self.PickDown()
                self.goal_status = 0
                self.mode = "return_to_base"
            # 8. return to # 2.


    
    def Delay(self, data):
        self.rate = rospy.Rate(data)
        self.rate.sleep()
        
    def starting_trigger(self):
        if self.mode == "waiting_position" or self.mode == "detect_marker":
            rospy.Subscriber("/starting_trigger", ArucoTriggerMsg, self.StartingCallback)
        else:
            pass

    def GoalPoseCallback(self, data):
        self.goal_status = data.status.status
        if (self.goal_status == 3 or self.goal_status == 4) and (self.mode == "marker_waiting_position"):
            print("goal reached")

    def StartingCallback(self, data):
        if data.trigger == 1:
            self.mode = "return_to_base"
        elif data.trigger == -1:
            self.mode = "exit_program"
        else:
            pass

    def MarkerIdCallback(self, data):
        self.marker_id = data.marker_id
        if self.marker_id == 5:
            print("5555555")
        

    def check_reached(self):
        rospy.Subscriber('/move_base_msgs/result', MoveBaseActionResult, self.GoalPoseCallback)
        if self.goal_status == 3 or self.goal_status == 4:
            self.check = 1
            
        else:
            self.check = 0

    def return_to_base(self):
        self.vel.pose.position.x = -0.150932595
        self.vel.pose.position.y = 0.374625533
        self.vel.pose.position.z = 0
        self.vel.pose.orientation.z = -0.713975596
        self.vel.pose.orientation.w = 0.70017058
        self.vel.header.frame_id = "map"
        self.goal_pub.publish(self.vel)


    def trigger_detect_marker(self):
        if self.mode == "start_detect_marker":
            print(self.mode)
            self.Delay(3)
            self.ArucoTrigger.trigger = 1
            self.trigger_aruco.publish(self.ArucoTrigger)
            
        else:
            pass

        
    def turn_right(self):
        self.Delay(3)
        print("turnturn")
        self.cmd.angular.z = -0.480
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.3)
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)
        self.Delay(3)
        
    def go_straight(self):
        self.Delay(3)
        self.cmd.linear.x = 0.25
        self.cmd.angular.z = -0.02
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.55)
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.5)

    def Delay(self, data):
        self.rate = rospy.Rate(data)
        self.rate.sleep()

    def PickUp(self):
        self.Delay(3)
        self.cmd.angular.z = -0.480
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.3)
        self.cmd.angular.z = 0.0
        self.cmd.linear.x = -0.135
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.6)
        self.cmd.linear.x = 0.0
        self.serv.data = 835
        self.servo_pub.publish(self.serv)
        self.cmd_pub.publish(self.cmd)
        self.Delay(3)
        self.cmd.linear.x = 0.135
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.6)
        self.cmd.linear.x = 0.0
        self.cmd_pub.publish(self.cmd)
        self.Delay(3)

    def PickDown(self):
        self.Delay(3)
        self.cmd.linear.x = -0.10
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.6)
        self.cmd.linear.x = 0.0
        self.servo_pub.publish(self.serv)
        self.Delay(0.8)
        self.serv.data = 495
        self.servo_pub.publish(self.serv)
        self.cmd.linear.x = 0.13
        self.cmd_pub.publish(self.cmd)
        self.Delay(0.6)
        self.cmd.linear.x = 0.0
        self.cmd_pub.publish(self.cmd)
        self.Delay(5)

    def Navigation(self):
        #if self.marker_id == 2:
            #self.vel.pose.position.x = -0.404999643564
            #self.vel.pose.position.y = 0.290000021458
            #self.vel.pose.position.z = 0
            #self.vel.pose.orientation.z = 0.544158668119
            #self.vel.pose.orientation.w = 0.83898232634
            #self.vel.header.frame_id = "map"
        if self.marker_id == 3:
            self.vel.pose.position.x = -1.12754
            self.vel.pose.position.y = 1.75136
            self.vel.pose.position.z = 0
            self.vel.pose.orientation.z = -0.38301
            self.vel.pose.orientation.w = 0.92374
            self.vel.header.frame_id = "map"
        
        elif self.marker_id == 4:
            self.vel.pose.position.x = -0.34915
            self.vel.pose.position.y = 1.72543
            self.vel.pose.position.z = 0
            self.vel.pose.orientation.z = 0.87702
            self.vel.pose.orientation.w = -0.48044
            self.vel.header.frame_id = "map"
        else:
            pass
        self.goal_pub.publish(self.vel)
        self.marker_id = 0


Turtle = Control()

if __name__ == "__main__":
    Turtle.main()

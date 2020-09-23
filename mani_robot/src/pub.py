#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from mani_robot.msg import check_msg
from move_base_msgs.msg import MoveBaseActionResult
import numpy as np
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('test_pub')
    mode_pub = rospy.Publisher('mode', Int32, queue_size=10)
    check_pub=rospy.Publisher('check_aruco',check_msg,  queue_size=10)
    move_fin_pub=rospy.Publisher("aruco_move_fin", Bool,  queue_size=10)           
    close_pub=rospy.Publisher("fin_move_close", Bool,  queue_size=10)   
    # result_pub=rospy.Publisher("/tb3_1/move_base/result", Int32,  queue_size=10)
    # rospy.Subscriber("/fin_send_mani", Bool, self.fin_send)
    pick_up_pub=rospy.Publisher("fin_pick_up", Bool,  queue_size=10)
    arrived_pub=rospy.Publisher("arrived_mani", Bool,  queue_size=10)


    try:
        while (1):
            key = getKey()

            if key == '1':
                mode_pub.publish(1)
                print('mode_pub 1')
            if key == '2':
                mode_pub.publish(2)
                print('mode_pub 2')
            if key == '3':
                mode_pub.publish(3)
                print('mode_pub 3')
            if key == '4':
                mode_pub.publish(4)
                print('mode_pub 4')
            if key == '5':
                mode_pub.publish(5)
                print('mode_pub 5')
            if key == '6':
                mode_pub.publish(6)
                print('mode_pub 6')

            if key == 'q':
                check_pub.publish(True)
                print('mode_pub 1')
            if key == 'w':
                move_fin_pub.publish(True)
                print('mode_pub 2')
            if key == 'e':
                close_pub.publish(True)
                print('mode_pub 3')
            # if key == 'r':
            #     result_pub.publish(3)
            #     print('mode_pub 1')
            if key == 't':
                pick_up_pub.publish(True)
                print('mode_pub 4')
            if key == 'y':
                arrived_pub.publish(True)
                print('mode_pub 5')
                


            else:
                if (key == '\x03'):
                    #pub_start.publish(0)
                    break



    except rospy.ROSInterruptException:
        pass

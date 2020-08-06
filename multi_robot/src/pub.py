#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from multi_robot.msg import Float32Multi

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
    pub_aruco_msg = rospy.Publisher('/aruco_msg', Int32, queue_size=10)
    pub_id_msg = rospy.Publisher('/id_msg', Int32, queue_size=10)
    pub_aruco_move = rospy.Publisher('/aruco_move', Int32, queue_size=10)
    pub_home_fin = rospy.Publisher('/home_fin',Int32,queue_size=10)
    pub_call_fin = rospy.Publisher('/call_fin', Int32, queue_size=10)
    pub_Mani_state = rospy.Publisher('/Mani_state', Int32, queue_size=10)
    pub_control_frame = rospy.Publisher('/control_frame',Int32,queue_size=10)

    try:
        while (1):
            key = getKey()

            if key == 'q':
                pub_aruco_msg.publish(0)
                print('id_msg 1')
            if key == 'w':
                pub_aruco_msg.publish(1)
                print('id_msg 2')


            if key == 'a':
                pub_aruco_move.publish(1)
                print('aruco_move 1')
            if key == 's':
                pub_aruco_move.publish(0)
                print('aruco_move 0')


            if key == 'z':
                pub_home_fin.publish(1)
                print('home_fin 1')
            if key == 'x':
                pub_home_fin.publish(0)
                print('home_fin 0')



            if key == '1':
                pub_call_fin.publish(1)
                print('call_fin 1')
            if key == '2':
                pub_call_fin.publish(0)
                print('call_fin 0')


            if key == '3':
                pub_Mani_state.publish(1)
                print('Mani_state 1')
            if key == '4':
                pub_Mani_state.publish(0)
                print('Mani_state 0')


            if key == '5':
                pub_control_frame.publish(1)
                print('control_frame 1')
            if key == '6':
                pub_control_frame.publish(0)
                print('control_frame 0')


            
            if key == 'b':
                pub_id_msg.publish(1)
                print('control_frame 1')
            if key == 'n':
                pub_id_msg.publish(2)
                print('control_frame 0')
            if key == 'm':
                pub_id_msg.publish(3)
                print('control_frame 0')
            else:
                if (key == '\x03'):
                    #pub_start.publish(0)
                    break



    except rospy.ROSInterruptException:
        pass

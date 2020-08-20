#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose
from multi_robot.msg import aruco_msgs
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
    pub_aruco_msg = rospy.Publisher('aruco_msg', Int32, queue_size=10)
    pub_id_msg = rospy.Publisher('id_msg', Int32, queue_size=10)
    pub_aruco_move = rospy.Publisher('aruco_move', Int32, queue_size=10)
    pub_home_fin = rospy.Publisher('return_frame_msg',Int32,queue_size=10)
    pub_call_fin = rospy.Publisher('call_frame', Int32, queue_size=10)
    pub_Mani_state = rospy.Publisher('Mani_state', Int32, queue_size=10)
    pub_control_frame = rospy.Publisher('control_frame',Int32,queue_size=10)
    pub_control_frame = rospy.Publisher('control_frame',Int32,queue_size=10)
    pub_Mani_pose=rospy.Publisher('mani_pose',Pose,queue_size=10)
    pub_aruco_pose=rospy.Publisher('aruco',aruco_msgs,queue_size=10)
    poses = Pose()
    aruco = aruco_msgs()
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

            if key == '7':
                poses.position.x = 0.2
                poses.position.y = 0.1
                poses.position.z = 0.31
                poses.orientation.x = 1
                poses.orientation.y = 1
                poses.orientation.z = 1
                poses.orientation.w = 1
                pub_Mani_pose.publish(poses)
               
                aruco.t_x = 0.2
                aruco.t_y = 0.3
                aruco.t_z = 0.1
                aruco.id = 1
                aruco.r_x=0.3
                aruco.r_y=0.2
                aruco.r_z=0.1

                
                pub_aruco_pose.publish(aruco)
                print("sss")
            else:
                if (key == '\x03'):
                    #pub_start.publish(0)
                    break



    except rospy.ROSInterruptException:
        pass

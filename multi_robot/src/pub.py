#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


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
    pub_start = rospy.Publisher('/start', Int32, queue_size=10)
    pub_starting = rospy.Publisher('/starting', Int32, queue_size=10)
    pub_stop = rospy.Publisher('/stop',Int32,queue_size=10)

    try:
        while (1):
            key = getKey()

            if key == '1':
                pub_start.publish(1)
                print('sss1')
            if key == '2':
                pub_start.publish(0)
                print('sss0')
            if key == '3':
                pub_starting.publish(1)
                print('ssds1')
            if key == '4':
                pub_starting.publish(0)
                print('ssds0')

            if key == '5':
                pub_stop.publish(1)
                print('ssds1')
            if key == '6':
                pub_stop.publish(0)
                print('ssds0')

            else:
                if (key == '\x03'):
                    pub_start.publish(0)
                    break



    except rospy.ROSInterruptException:
        pass

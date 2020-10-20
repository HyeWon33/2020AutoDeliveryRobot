#!/usr/bin/env python  
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool
from mani_robot.msg import check_msg

count = 0
aruco_check = False


def main():
    global aruco_check, count
    rospy.init_node('aruco_move')
    listener = tf.TransformListener()
    fin_pub = rospy.Publisher("aruco_move_fin", Bool, queue_size=1)
    

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        rospy.Subscriber('aruco_move_pub', Bool, start)
        print("fist", aruco_check)
        if aruco_check:

            try:
                (trans, rot) = listener.lookupTransform('/map', 'tb3_1/arucopose', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            send_aruco_pose(trans, rot)

            rate.sleep()



def start(check_aruco):
    global count, aruco_check

    aruco_check = check_aruco.data
    # print("sub",aruco_check)


def send_aruco_pose(trans, rot):
    turtle_vel = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    aruco_move_pub = rospy.Publisher('aruco_move_pub', Bool, queue_size=1)  

    # (trans,rot) = listener.lookupTransform('/map', '/arucopose', rospy.Time(0))
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
    # print(roll, pitch, yaw)
    ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw + np.pi / 2)

    yawMatrix = np.array([
        [math.cos(yaw), -math.sin(yaw), trans[0]],
        [math.sin(yaw), math.cos(yaw), trans[1]],
        [0, 0, 1]
    ])
    wMatrix = np.array([
        [math.cos(np.pi), -math.sin(np.pi), 0],
        [math.sin(np.pi), math.cos(np.pi), -0.45],
        [0, 0, 1]
    ])
    m = np.dot(yawMatrix, wMatrix)
    x, y = m[:2, 2]
    cosine, sine = m[:2, 0]
    theta = np.arctan2(sine, cosine)

    nav_goal = PoseStamped()
    nav_goal.header.frame_id = 'map'
    nav_goal.pose.position.x = x
    nav_goal.pose.position.y = y
    nav_goal.pose.position.z = 0
    nav_goal.pose.orientation.x = 0
    nav_goal.pose.orientation.y = 0
    nav_goal.pose.orientation.z = oz
    nav_goal.pose.orientation.w = ow
    #print("seceond", aruco_check)
    # if aruco_check  == 2 :
    turtle_vel.publish(nav_goal)
    aruco_move_pub.publish(False)    # count += 1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

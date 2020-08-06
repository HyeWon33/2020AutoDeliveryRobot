#!/usr/bin/env python
import numpy as np
import cv2
import glob
import math
import os
import rospy
from multi_robot.msg import aruco_msgs
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

def cal():
    rospy.loginfo("===================================START CALIBRATION===================================")
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/home/mun/catkin_ws/src/multi_robot/src/camera_cal_img/*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9, 6), corners, ret)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    rospy.loginfo("===================================END CALIBRATION===================================")
    return mtx, dist


def arucomark(mtx, dist):
    rospy.loginfo("===================================START DETECT===================================")
    aruco_pub = rospy.Publisher('aruco_msg', aruco_msgs, queue_size=10)
    aruco = aruco_msgs()

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    param = cv2.aruco.DetectorParameters_create()
    param.adaptiveThreshConstant = 10
    os.system('sudo modprobe bcm2835-v4l2')
    cam = cv2.VideoCapture(0)

    if cam.isOpened():
        while True:
            _, frame = cam.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=param)

            if np.all(ids != None):
                # rvecs, tvecs, objpoint = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                rvecs_msg = rvecs.tolist()
                tvecs_msg = tvecs.tolist()
                

                for i in range(0, ids.size):
                    rvecs_msg_x = rvecs_msg[i][0][0]
                    rvecs_msg_y = rvecs_msg[i][0][0]
                    rvecs_msg_z = rvecs_msg[i][0][0]
                    tvecs_msg_x = tvecs_msg[i][0][0]
                    tvecs_msg_y = tvecs_msg[i][0][0]
                    tvecs_msg_z = tvecs_msg[i][0][0]
    
                    aruco.r_x = rvecs_msg_x
                    aruco.r_y = rvecs_msg_y
                    aruco.r_z = rvecs_msg_z
                    aruco.t_x = tvecs_msg_x
                    aruco.t_y = tvecs_msg_y
                    aruco.t_z = tvecs_msg_z
                    aruco.id = int(ids[i])
                    rospy.loginfo(aruco)
                    aruco_pub.publish(aruco)
                    
                    frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)

                frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
                frame = cv2.aruco.drawDetectedMarkers(frame, point, borderColor=(0, 255, 0))

            cv2.imshow("result", frame)
            k = cv2.waitKey(100)
            if k == ord('q'):
                break
    cam.release()
    cv2.destroyAllWindows()


def main():
    rospy.init_node("aruco_detect")
    mtx, dist = cal()
    arucomark(mtx, dist)


if __name__ == "__main__":
    main()

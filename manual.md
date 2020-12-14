# 공통

1. remote pc 와 각 로봇에 clone 하기

2. lift 로봇은 tb3_0, mani 로봇은 tb3_1로 설정한다.

3. 로봇 하나를 사용하여 map 파일 만들기(참조 : https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node)
   \$ roslaunch turtlebot3_slam turtlebot3_slam.launch

   \$ rosrun map_server map_saver -f ~/map

4. \$ 는 터미널창에서 , -> 기호는 같은 터미널창에서

# 각각의 터틀봇(lift, mani)

1. 각각의 로봇에 이름에 맞는 aruco_into_robot 안에 있는 해당 패키지만 남기기
2. 각각의 로봇 ip 확인하기
3. bashrc에서 ROS_HOSTNAME는 로봇 ip, ROS_MASTER_URI를 remote ip로 변경하기
4.  export TURTLEBOT3_MODEL=waffle_pi 로 변경하기
5. export ROS_NAMESPACE =tb3_[숫자] 을 추가한다. -> 공통 2번 참고



# REMOTE PC

1. bashrc 에서 ROS_HOSTNAME, ROS_MASTER_URI를 remote ip로 변경하기

2.  export TURTLEBOT3_MODEL=waffle_pi 로 변경하기

3. \$ roscore

4. \$ ssh pi@{로봇 ip} ->

5. \$ roslanch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=tb3_r set_lidar_frame_id:=tb3_r/base_scan

   [INFO] [1597128027.053218]: Setup TF on Odometry [tb3\_[숫자]/odom]
   [INFO] [1597128027.069242]: Setup TF on IMU [tb3\_[숫자]/imu_link]
   [INFO] [1597128027.079824]: Setup TF on MagneticField [tb3\_[숫자]/mag_link]
   [INFO] [1597128027.103283]: Setup TF on JointState [tb3\_[숫자]/base_link]

   킬때 이렇게 뜨는지 확인

7. ssh pi@{로봇 ip} ->

8. lift로봇에서는 \$ rosrun lift_robot_aruco charuco_clibration.py , mani 로봇에서는 \$ rosrun mani_robot_aruco charuco_clibration.py

9. \$ ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch

10. \$ ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_manipulation_moveit_config move_group.launch

11. \$ ROS_NAMESPACE=tb3_1 rosrun test_turtle_mani test_turtle_mani

12. \$ roslaunch turtlebot3_nps move_base_two.launch

13. \$ roslaunch turtlebot3_nps navigation_two.launch map_file:={$map.yaml 디렉토리 경로}
    rviz가 켜지는 확인하기
    로봇들의 표시가 잘 나오는지 확인하기

14. \$ roslaunch mani_robot all.launch

15. \$ roslaunch lift_robot all.launch
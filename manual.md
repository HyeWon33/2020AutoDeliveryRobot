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

3. roscore 실행

   ```
   $ roscore
   ```

4. 각 로봇에 원격접속

   ```
   $ ssh pi@{로봇 ip}
   ```

   원격 접속한 터미널 창에서 실행

   ```
   pi@pi$ roslanch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=tb3_[숫자] set_lidar_frame_id:=tb3_[숫자]/base_scan
   
   [INFO] [1597128027.053218]: Setup TF on Odometry [tb3\_[숫자]/odom]
   [INFO] [1597128027.069242]: Setup TF on IMU [tb3\_[숫자]/imu_link]
   [INFO] [1597128027.079824]: Setup TF on MagneticField [tb3\_[숫자]/mag_link]
   [INFO] [1597128027.103283]: Setup TF on JointState [tb3\_[숫자]/base_link]
   ```

   실행 되는 중간에 위와 같이 나오는 지 확인 만약 안나오면 실행 취소후 재실행

5. 다른 터미널창에서 다시 원격 접속

   ```
   $ ssh pi@{로봇 ip}
   ```

   그 후 aruco marker 노드 실행
   지게(lift)로봇

   ```
   $ rosrun lift_robot_aruco charuco_clibration.py
   ```

   mani 로봇

   ```
   $ rosrun mani_robot_aruco charuco_clibration.py
   ```

6. 메니퓰레이터 bringup 실행

   ```
   $ ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
   ```

7. 메니퓰레이터 move_group 실행

   ```
   $ ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_manipulation_moveit_config move_group.launch
   ```

8. 메니 동작 노드 실행

   ```
   $ ROS_NAMESPACE=tb3_1 rosrun test_turtle_mani test_turtle_mani
   ```

9. 두대의 move_base 동시 실행 launch 실행

   ```
   $ roslaunch turtlebot3_nps move_base_two.launch
   ```

10. nvigation 두대 동시 실행

    ```
    $ roslaunch turtlebot3_nps navigation_two.launch map_file:={$map.yaml 디렉토리 경로}
    ```

    rviz가 켜지는 확인하기
    로봇들의 표시가 잘 나오는지 확인하기

11. 메니퓰레이터 와 지게 control launch 파일 실행

    ````
    $ roslaunch mani_robot all.launch
    $ roslaunch lift_robot all.launch
    ````

    
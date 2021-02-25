# 2020AutoDeliveryRobot

## 목표

첫 번째, 랜덤으로 특정한 위치로 이동하면 물체(벽) 또는 다른 로봇과의 충돌없이 스스로 이동해야 한다.

두 번째 ArUco Marker를 인식하여 물건의 위치까지 이동 후 물건을 집을 수 있어야 한다. 

세 번째, 분류하기 위하여 물건마다 지정된 위치에 각각 다른 물건을 내려놓을 수 있어야 한다. 

이 모든 기준은 사람의 조작 없이 로봇 스스로 작동하게 하는 것을 목표로 한다.



##  Manipulator **장착 로봇 작동 알고리즘**

   1. 지정된 좌표를 랜덤으로 로봇에게 보내어 로봇을 이동시킨다.

   2. 로봇이 해당 위치로 이동하는 중 Marker가 붙은 물체가 있는지 확인한다. Marker가 붙은 물체가 없다면 1번 동작을 반복한다

   3. 2번의 동작 중 Marker를 발견하면 Marker가 제대로 인식된 것인지 한 번 더 확인한다.

   4. marker에서 받은 정보로 물체의 위치를 추정하여 로봇을 물체 앞에    위치시킨다.

   5.  Marker에서 얻은 값을 계산하여 물체의 좌표를 구해 Manipulator를 이동시켜 물체를 집는다. 만약, Manipulator가 집을 수 없는 거리라면 로봇의 위치를 물체 앞으로 조정한다.

   6. 로봇을 목적지로 이동시켜 물체를 내려놓고 1의 단계로 돌아가 동작을 반복한다. 

       <img src=".\img\mani_algorithm" alt="mani_algorithm" style="zoom:70%;" />  

## 지게(Lift) 장착 로봇 작동 알고리즘

1. 지정된 좌표를 랜덤으로 로봇에게 보내어 로봇을 이동시킨다.

2. 로봇이 해당 위치로 이동하는 중 Marker가 붙은 물체가 있는지 확인한다. Marker가 붙은 물체가 없다면 1번 동작을 반복한다.

3. 2번의 동작 중 Marker를 발견하면 Marker가 제대로 인식된 것인지 한 번 더 확인한다.

4. Marker에서 받은 정보로 물체의 위치를 추정하여 로봇을 물체 앞에 위치시킨다.

5. 지게가 물체의 아래 공간에 들어가도록 로봇을 180도 회전시켜 로봇을 이동시킨 후 지게로 물건을 들어올린다. 

6. 로봇을 목적지로 이동시켜 물체를 내려놓고 1의 단계로 돌아가 동작을 반복한다.

  <img src=".\img\lift_algorithm" alt="lift_algorithm" style="zoom:70%;" />  

## 로봇의 형태

**manipultor**

<img src="/home/j/workspace/github/2020AutoDeliveryRobot/img/mani_img.png" alt="mani_img" style="zoom:50%;" />

**lift**

<img src="/home/j/workspace/github/2020AutoDeliveryRobot/img/lift_img.png" alt="lift_img" style="zoom:50%;" />



## 실행시 필요한 의존성 패키지 및 미리 build 되있어야 할 패키지



### 의존성 패키지

- turtlebot/manipulator을 동작시키기 위한 의존성 패키지

  ```
  sudo apt install ros-melodic-joy\
    ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard\
    ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino \
    ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs \ 
    ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf \
    ros-melodic-xacro ros-melodic-compressed-image-transport \
    ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers \
    ros-melodic-gazebo* ros-melodic-moveit* ros-melodic-industrial-core \
    ros-melodic-ros-control* ros-melodic-control*  ros-melodic-moveit*
  ```



### 사용 패키지

전체 패키지

- 드래그 해서 터미널창에 붙혀넣으면 전체 설치 가능(마지막에서 한번더 enter)

| git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git |
| ------------------------------------------------------------ |
| git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git |
| git clone https://github.com/ROBOTIS-GIT/turtlebot3.git      |
| git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git    |
| git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git |
| git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git |
| git clone https://github.com/ROBOTIS-GIT/open_manipulator.git |
| git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git |
| git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git |
| git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git |
| git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git |

// 실재로 사용 되는 패키지(추가 필요시 작성)

 ex) turtlebot3 -> turtlebot3_bringup(turtlebot3 core 및 lidar 키는 패키지 포함)


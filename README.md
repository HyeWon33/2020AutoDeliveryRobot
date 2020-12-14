# 2020AutoDeliveryRobot

사용 패키지

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

#  Manipulator **장착 로봇 작동 알고리즘**

   1. 지정된 좌표를 랜덤으로 로봇에게 보내어 로봇을 이동시킨다.

   2. 로봇이 해당 위치로 이동하는 중 Marker가 붙은 물체가 있는지 확인한다. Marker가 붙은 물체가 없다면 1번 동작을 반복한다

   3. 2번의 동작 중 Marker를 발견하면 Marker가 제대로 인식된 것인지 한 번 더 확인한다.

   4. marker에서 받은 정보로 물체의 위치를 추정하여 로봇을 물체 앞에    위치시킨다.

   5.  Marker에서 얻은 값을 계산하여 물체의 좌표를 구해 Manipulator를 이동시켜 물체를 집는다. 만약, Manipulator가 집을 수 없는 거리라면 로봇의 위치를 물체 앞으로 조정한다.

   6. 로봇을 목적지로 이동시켜 물체를 내려놓고 1의 단계로 돌아가 동작을 반복한다. 

       <img src=".\mani_algorithm" alt="mani_algorithm" style="zoom:70%;" />  

# 지게(Lift) 장착 로봇 작동 알고리즘

1. 지정된 좌표를 랜덤으로 로봇에게 보내어 로봇을 이동시킨다.

2. 로봇이 해당 위치로 이동하는 중 Marker가 붙은 물체가 있는지 확인한다. Marker가 붙은 물체가 없다면 1번 동작을 반복한다.

3. 2번의 동작 중 Marker를 발견하면 Marker가 제대로 인식된 것인지 한 번 더 확인한다.

4. Marker에서 받은 정보로 물체의 위치를 추정하여 로봇을 물체 앞에 위치시킨다.

5. 지게가 물체의 아래 공간에 들어가도록 로봇을 180도 회전시켜 로봇을 이동시킨 후 지게로 물건을 들어올린다. 

6. 로봇을 목적지로 이동시켜 물체를 내려놓고 1의 단계로 돌아가 동작을 반복한다.

  <img src=".\lift_algorithm" alt="lift_algorithm" style="zoom:70%;" />  
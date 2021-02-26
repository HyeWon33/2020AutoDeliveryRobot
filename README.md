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

#### **manipultor**

<img src="/home/j/workspace/github/2020AutoDeliveryRobot/img/mani_img.png" alt="mani_img" style="zoom:50%;" />

#### **lift**

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

### repositorie 내의 패키지 설명

#### aruco_into_robot

내부에 lift_robot_aruco,mani_robot_aruco 패키지가 있습니다. 각각 이름의 robot에 받아서 build합니다.

각 패키지는 aruco 마커를 확인하는 패키지입니다.

#### lift_robot

lift robot을 동작시키 위한 패키지입니다.

#### mani_robot

mani robot을 동작시키 위한 패키지입니다.

#### test_turtle_mani

manipulator를 동작시키 위한 패키지입니다.

#### turtlebot3_nps

두대의 로봇을 동시에 동작 시키기 위해 사용 되는 패키지입니다.

해당 패키지에서 move_base_two.launch와  navigation_two.launch를 실행 시킵니다.

navigation_two.launch을 실행 시킬때는 map_file을 지정 해줘야합니다.





## 실행방법

### 공통

1. remote pc 와 각 로봇에 clone 하기

2. lift 로봇은 tb3_0, mani 로봇은 tb3_1로 설정한다.

3. 로봇 하나를 사용하여 map 파일 만들기(참조 : https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node)
   \$ roslaunch turtlebot3_slam turtlebot3_slam.launch

   \$ rosrun map_server map_saver -f ~/map

4. \$ 는 터미널창에서 , -> 기호는 같은 터미널창에서

### 각각의 터틀봇(lift, mani)

1. 각각의 로봇에 이름에 맞는 aruco_into_robot 안에 있는 해당 패키지만 남기기
2. 각각의 로봇 ip 확인하기
3. bashrc에서 ROS_HOSTNAME는 로봇 ip, ROS_MASTER_URI를 remote ip로 변경하기
4. export TURTLEBOT3_MODEL=waffle_pi 로 변경하기
5. export ROS_NAMESPACE =tb3_[숫자] 을 추가한다. -> 공통 2번 참고



### REMOTE PC

1. bashrc 에서 ROS_HOSTNAME, ROS_MASTER_URI를 remote ip로 변경하기

2. export TURTLEBOT3_MODEL=waffle_pi 로 변경하기

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

    

# 사용 이론, 기법

## tf

tf는 프레임을 추적할 수 있게 해주는 ros 라이브러리 입니다. tf를 이용해서 두 프레임간의 변환 행렬을 얻거나 서로 다른 프레임의 정보를 Rviz에 보여주기 쉽게 하기도 합니다. 또한 시간이 지나도 과거의 프레임을 추적할 수 있게 해줍니다.

### tf.TransformBroadcaster

transration, rotation 정보들과 frame_id ,child_frame_id,stamp의 정보들을 이용하여 tf를 생성해준다. 

geometry_msgs/TransformStamped 을 이용하여 생성이 가능하다.

```python
br = tf.TransformBroadcaster() 
br.sendTransform((msg.position.x, msg.position.y, msg.position.z),      
                     (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                     rospy.Time.now(),
                     "tb3_1/mani_pose",
                     "tb3_1/base_link")
```



### tf.TransformListener

두 좌표계간의 transration, rotation 정보들을 받아오고자 할 때 사용됩니다.

```python
listener = tf.TransformListener()
	try:
    	(trans, rot) = listener.lookupTransform('tb3_1/base_link', 'tb3_1/arucopose', rospy.Time(0))

	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    	print("aaa")
```



### tf.transformations

quaternion과 euler를 서로변경할때 사용하게 됩니다.

tf.transformations.euler_from_quaternion 은 quaternion을 euler로

tf.transformations.quaternion_from_euler 은 euler을 quaternion으로 변경 되게 됩니다.

```python
roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation)
ox, oy, oz, ow = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
```



## 좌표계 변환

1. 두개의 좌표계 정보들 얻어옵니다.
2. tf.transformations.euler_from_quaternion으로 euler 값으로 변환 해줍니다. 
3. 그후 아래 식을 이용하여 rotation matrix를 만들어줍니다.

$$
yaw = R_z(\alpha) =\begin{pmatrix}cos(\alpha) & -sin(\alpha) & 0 \\ sin(\alpha) & cos(\alpha) & 0 \\ 0& 0&1 \end{pmatrix}\\
pitch = R_y(\beta) = \begin{pmatrix}cos(\beta) & 0 & sin(\beta) \\ 0 & 1 & 0 \\ -sin(\beta)& 0&cos(\beta) \end{pmatrix}\\
roll = R_x(\gamma) = \begin{pmatrix}1 & 0 & 0 \\ 0 & cos(\gamma) & -sin(\gamma) \\ 0& sin(\gamma) & cos(\gamma) \end{pmatrix} \\
$$

![rotation_matrix](/home/j/workspace/github/2020AutoDeliveryRobot/img/rotation_matrix.gif)

```python
# rotation matrix 생성
def create_rotation_matrix(euler):
    (yaw, pitch, roll) = euler

    yaw_matrix = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_matrix = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    roll_matrix = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    rotation_matrix_a = np.dot(pitch_matrix, roll_matrix)
    rotation_matrix = np.dot(yaw_matrix, rotation_matrix_a)

    return rotation_matrix
```

4. transformation 생성



![img843](/home/j/workspace/github/2020AutoDeliveryRobot/img/img843.gif)

```python
# transformation matrix 생성
def make_transformation_matrix(r_matrix, t_matrix):
    matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
    zero_one = np.array([[0., 0., 0., 1.]])
    matrix_4x4 = np.concatenate((matrix_3x4, zero_one), axis=0)
    return matrix_4x4
```

5. 두 transformation을 dot 연산 시행

$$
T = T_a * T_b
$$


```python
m_c_matrix = np.dot(mani_3d_matrix, cam_3d_matrix)
```



6. 계산된 transformation에서 tranlation, rotation 추출

![img843](/home/j/workspace/github/2020AutoDeliveryRobot/img/img843.gif)

햬당 matrix에서 T[:3,3]을 translation을 추출,

 r_11 = T[0,0], r_21=T[1,0] .... 으로 할때
$$
\alpha = tan^{-1}(r_{21}/r_{11})\\
\beta = tan^{-1}(-r_{31}/\sqrt{r^2_{32}+r^2_{33}})\\
\gamma = tan^{-1}(r_{32}/r_{33})\\
$$
그 후 quaternion으로 변환하여 사용 한다.

```python
# transformation matrix 에서 quaternion 과 translation 추출
def get_rpt_to_rotation_vector(matrix):
    r_11 = matrix[0, 0]  # cos(yaw)cos(pitch)
    r_21 = matrix[1, 0]  # sin(yaw)cos(pitch)
    r_31 = matrix[2, 0]  # -sin(pitch)
    r_32 = matrix[2, 1]  # cos(pitch)sin(roll)
    r_33 = matrix[2, 2]  # cos(pitch)cos(roll)
    translation = list(matrix[:3, 3])
    yaw = np.arctan2(r_21, r_11)
    pitch = np.arctan2(-r_31, np.sqrt((np.square(r_32)) + np.square(r_33)))
    roll = np.arctan2(r_32, r_33)
    euler_deg = [180 * (yaw / np.pi), 180 * (pitch / np.pi), 180 * (roll / np.pi)]
    quaternion = quaternion_from_euler(yaw, pitch, roll)
    return translation, quaternion
```

## 코드 설명

각 코드에 적어두웠습니다.
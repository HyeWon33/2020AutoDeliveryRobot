cmake_minimum_required(VERSION 2.8.3)
project(aruco_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  std_msgs
)

set(CMAKE_CXX_FLAGS "-L/usr/local/lib -pthread")
#set(CMAKE_MODULE_PATH "/home/turtle/opencv_build/CMakeFiles/opencv_modules.dir:${CMAKE_MODULE_PATH}")
#set(OpenCV_DIR "/home/turtle/opencv_build/opencv-4.1.1/build")
#set(OpenCV_LIBRARIES "/opt/ros/kinetic/lib/x86_64-linux-gnu)

find_package(OpenCV REQUIRED)
#find_package( COMPONENTS videoio core highgui)


add_message_files(
  FILES
  ArucoMsg.msg
  ArucoTriggerMsg.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES aruco_pkg
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  /usr/local/include/opencv4/
#  /home/js/opencv/opencv-4.1.1/include/
  /home/turtle/opencv-4.1.1/include
  /home/turtle/opencv_contrib/modules/aruco/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  
)

link_directories(
  /usr/lib/
)

add_executable(aruco_pkg_node1 /home/turtle/catkin_ws/src/aruco_pkg/src/aruco_pkg_node1.cpp)
add_dependencies(aruco_pkg_node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_pkg_node1 ${catkin_LIBRARIES})


add_executable(aruco_pkg_node2 /home/turtle/catkin_ws/src/aruco_pkg/src/aruco_pkg_node2.cpp)
add_dependencies(aruco_pkg_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_pkg_node2 ${catkin_LIBRARIES})



 target_link_libraries(aruco_pkg_node1
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
#   opencv_core opencv_videoio opencv_highgui opencv_aruco
 )
target_link_libraries(aruco_pkg_node2
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
#   opencv_core opencv_videoio opencv_highgui opencv_aruco
 )



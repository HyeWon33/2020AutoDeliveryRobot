# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "multi_robot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imulti_robot:/home/j/catkin_ws/src/multi_robot/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Imove_base_msgs:/opt/ros/melodic/share/move_base_msgs/cmake/../msg;-Imoveit_msgs:/opt/ros/melodic/share/moveit_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg;-Ishape_msgs:/opt/ros/melodic/share/shape_msgs/cmake/../msg;-Iobject_recognition_msgs:/opt/ros/melodic/share/object_recognition_msgs/cmake/../msg;-Ioctomap_msgs:/opt/ros/melodic/share/octomap_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(multi_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" NAME_WE)
add_custom_target(_multi_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "multi_robot" "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(multi_robot
  "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multi_robot
)

### Generating Services

### Generating Module File
_generate_module_cpp(multi_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multi_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(multi_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(multi_robot_generate_messages multi_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" NAME_WE)
add_dependencies(multi_robot_generate_messages_cpp _multi_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multi_robot_gencpp)
add_dependencies(multi_robot_gencpp multi_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multi_robot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(multi_robot
  "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multi_robot
)

### Generating Services

### Generating Module File
_generate_module_eus(multi_robot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multi_robot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(multi_robot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(multi_robot_generate_messages multi_robot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" NAME_WE)
add_dependencies(multi_robot_generate_messages_eus _multi_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multi_robot_geneus)
add_dependencies(multi_robot_geneus multi_robot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multi_robot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(multi_robot
  "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multi_robot
)

### Generating Services

### Generating Module File
_generate_module_lisp(multi_robot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multi_robot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(multi_robot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(multi_robot_generate_messages multi_robot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" NAME_WE)
add_dependencies(multi_robot_generate_messages_lisp _multi_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multi_robot_genlisp)
add_dependencies(multi_robot_genlisp multi_robot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multi_robot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(multi_robot
  "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multi_robot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(multi_robot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multi_robot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(multi_robot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(multi_robot_generate_messages multi_robot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" NAME_WE)
add_dependencies(multi_robot_generate_messages_nodejs _multi_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multi_robot_gennodejs)
add_dependencies(multi_robot_gennodejs multi_robot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multi_robot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(multi_robot
  "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multi_robot
)

### Generating Services

### Generating Module File
_generate_module_py(multi_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multi_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(multi_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(multi_robot_generate_messages multi_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/j/catkin_ws/src/multi_robot/msg/aruco_msgs.msg" NAME_WE)
add_dependencies(multi_robot_generate_messages_py _multi_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multi_robot_genpy)
add_dependencies(multi_robot_genpy multi_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multi_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multi_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multi_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(multi_robot_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(multi_robot_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET move_base_msgs_generate_messages_cpp)
  add_dependencies(multi_robot_generate_messages_cpp move_base_msgs_generate_messages_cpp)
endif()
if(TARGET moveit_msgs_generate_messages_cpp)
  add_dependencies(multi_robot_generate_messages_cpp moveit_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(multi_robot_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(multi_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multi_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multi_robot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(multi_robot_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(multi_robot_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET move_base_msgs_generate_messages_eus)
  add_dependencies(multi_robot_generate_messages_eus move_base_msgs_generate_messages_eus)
endif()
if(TARGET moveit_msgs_generate_messages_eus)
  add_dependencies(multi_robot_generate_messages_eus moveit_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(multi_robot_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(multi_robot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multi_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multi_robot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(multi_robot_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(multi_robot_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET move_base_msgs_generate_messages_lisp)
  add_dependencies(multi_robot_generate_messages_lisp move_base_msgs_generate_messages_lisp)
endif()
if(TARGET moveit_msgs_generate_messages_lisp)
  add_dependencies(multi_robot_generate_messages_lisp moveit_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(multi_robot_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(multi_robot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multi_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multi_robot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(multi_robot_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(multi_robot_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET move_base_msgs_generate_messages_nodejs)
  add_dependencies(multi_robot_generate_messages_nodejs move_base_msgs_generate_messages_nodejs)
endif()
if(TARGET moveit_msgs_generate_messages_nodejs)
  add_dependencies(multi_robot_generate_messages_nodejs moveit_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(multi_robot_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(multi_robot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multi_robot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multi_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multi_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(multi_robot_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(multi_robot_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET move_base_msgs_generate_messages_py)
  add_dependencies(multi_robot_generate_messages_py move_base_msgs_generate_messages_py)
endif()
if(TARGET moveit_msgs_generate_messages_py)
  add_dependencies(multi_robot_generate_messages_py moveit_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(multi_robot_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(multi_robot_generate_messages_py std_msgs_generate_messages_py)
endif()

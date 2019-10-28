# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "reflex_msgs2: 12 messages, 3 services")

set(MSG_I_FLAGS "-Ireflex_msgs2:/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(reflex_msgs2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" "reflex_msgs2/PoseCommand:reflex_msgs2/VelocityCommand"
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" "reflex_msgs2/Finger:reflex_msgs2/Imu:reflex_msgs2/Motor"
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" "reflex_msgs2/FingerPressure"
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" "reflex_msgs2/Imu"
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" ""
)

get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" NAME_WE)
add_custom_target(_reflex_msgs2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_msgs2" "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)

### Generating Services
_generate_srv_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_cpp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
)

### Generating Module File
_generate_module_cpp(reflex_msgs2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(reflex_msgs2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(reflex_msgs2_generate_messages reflex_msgs2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_cpp _reflex_msgs2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_msgs2_gencpp)
add_dependencies(reflex_msgs2_gencpp reflex_msgs2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_msgs2_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)

### Generating Services
_generate_srv_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_eus(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
)

### Generating Module File
_generate_module_eus(reflex_msgs2
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(reflex_msgs2_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(reflex_msgs2_generate_messages reflex_msgs2_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_eus _reflex_msgs2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_msgs2_geneus)
add_dependencies(reflex_msgs2_geneus reflex_msgs2_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_msgs2_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)

### Generating Services
_generate_srv_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_lisp(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
)

### Generating Module File
_generate_module_lisp(reflex_msgs2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(reflex_msgs2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(reflex_msgs2_generate_messages reflex_msgs2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_lisp _reflex_msgs2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_msgs2_genlisp)
add_dependencies(reflex_msgs2_genlisp reflex_msgs2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_msgs2_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)

### Generating Services
_generate_srv_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_nodejs(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
)

### Generating Module File
_generate_module_nodejs(reflex_msgs2
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(reflex_msgs2_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(reflex_msgs2_generate_messages reflex_msgs2_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_nodejs _reflex_msgs2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_msgs2_gennodejs)
add_dependencies(reflex_msgs2_gennodejs reflex_msgs2_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_msgs2_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg;/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_msg_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)

### Generating Services
_generate_srv_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)
_generate_srv_py(reflex_msgs2
  "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
)

### Generating Module File
_generate_module_py(reflex_msgs2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(reflex_msgs2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(reflex_msgs2_generate_messages reflex_msgs2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_msgs2_generate_messages_py _reflex_msgs2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_msgs2_genpy)
add_dependencies(reflex_msgs2_genpy reflex_msgs2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_msgs2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_msgs2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reflex_msgs2_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/reflex_msgs2
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(reflex_msgs2_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_msgs2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(reflex_msgs2_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/reflex_msgs2
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(reflex_msgs2_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_msgs2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reflex_msgs2_generate_messages_py std_msgs_generate_messages_py)
endif()

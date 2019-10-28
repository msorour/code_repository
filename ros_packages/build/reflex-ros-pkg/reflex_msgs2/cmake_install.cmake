# Install script for directory: /home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/business/code_repository/ros_packages/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/msg" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/srv" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/cmake" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/include/reflex_msgs2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/share/roseus/ros/reflex_msgs2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/share/common-lisp/ros/reflex_msgs2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/share/gennodejs/ros/reflex_msgs2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/business/code_repository/ros_packages/devel/lib/python2.7/dist-packages/reflex_msgs2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/lib/python2.7/dist-packages/reflex_msgs2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/cmake" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/cmake" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2Config.cmake"
    "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2" TYPE FILE FILES "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_msgs2/package.xml")
endif()


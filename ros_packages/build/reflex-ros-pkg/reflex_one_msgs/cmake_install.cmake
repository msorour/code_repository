# Install script for directory: /home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/msg" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/srv" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv"
    "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/cmake" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/include/reflex_one_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/share/roseus/ros/reflex_one_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/share/common-lisp/ros/reflex_one_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/share/gennodejs/ros/reflex_one_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/business/code_repository/ros_packages/devel/lib/python2.7/dist-packages/reflex_one_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/business/code_repository/ros_packages/devel/lib/python2.7/dist-packages/reflex_one_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/cmake" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/cmake" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgsConfig.cmake"
    "/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs" TYPE FILE FILES "/home/business/code_repository/ros_packages/src/reflex-ros-pkg/reflex_one_msgs/package.xml")
endif()


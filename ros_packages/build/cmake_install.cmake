# Install script for directory: /home/business/code_repository/ros_packages/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/business/code_repository/ros_packages/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/business/code_repository/ros_packages/install" TYPE PROGRAM FILES "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/business/code_repository/ros_packages/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/business/code_repository/ros_packages/install" TYPE PROGRAM FILES "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/business/code_repository/ros_packages/install/setup.bash;/home/business/code_repository/ros_packages/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/business/code_repository/ros_packages/install" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/setup.bash"
    "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/business/code_repository/ros_packages/install/setup.sh;/home/business/code_repository/ros_packages/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/business/code_repository/ros_packages/install" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/setup.sh"
    "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/business/code_repository/ros_packages/install/setup.zsh;/home/business/code_repository/ros_packages/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/business/code_repository/ros_packages/install" TYPE FILE FILES
    "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/setup.zsh"
    "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/business/code_repository/ros_packages/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/business/code_repository/ros_packages/install" TYPE FILE FILES "/home/business/code_repository/ros_packages/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/business/code_repository/ros_packages/build/gtest/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/dynamixel_motor/dynamixel_driver/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/dynamixel_motor/dynamixel_motor/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/dynamixel_motor/dynamixel_tutorials/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/dynamixel_motor/dynamixel_controllers/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/dynamixel_motor/dynamixel_msgs/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_msgs2/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_one_msgs/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_driver/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_driver2/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex1_visualizer/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_visualizer/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/reflex-ros-pkg/reflex_visualizer2/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/allegro_right_hand_control_gazebo/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/allegro_right_hand_control_real/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/allegro_right_hand_description/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/franka_panda_arm_control_gazebo/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/franka_panda_arm_description/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/franka_panda_arm_plus_allegro_right_hand_control_gazebo/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/franka_panda_arm_plus_own_gripper_control_gazebo/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/franka_panda_gripper_description/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/grasping_application/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/kinect2/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/realsense_d435_description/cmake_install.cmake")
  include("/home/business/code_repository/ros_packages/build/urdf_models/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/business/code_repository/ros_packages/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

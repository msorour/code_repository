# Install script for directory: /home/work/code_repository/ros_packages/src/allegro-hand-ros/allegro_hand_controllers

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/work/code_repository/ros_packages/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/work/code_repository/ros_packages/build/allegro-hand-ros/allegro_hand_controllers/catkin_generated/installspace/allegro_hand_controllers.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/allegro_hand_controllers/cmake" TYPE FILE FILES
    "/home/work/code_repository/ros_packages/build/allegro-hand-ros/allegro_hand_controllers/catkin_generated/installspace/allegro_hand_controllersConfig.cmake"
    "/home/work/code_repository/ros_packages/build/allegro-hand-ros/allegro_hand_controllers/catkin_generated/installspace/allegro_hand_controllersConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/allegro_hand_controllers" TYPE FILE FILES "/home/work/code_repository/ros_packages/src/allegro-hand-ros/allegro_hand_controllers/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liballegro_node.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liballegro_node.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liballegro_node.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/work/code_repository/ros_packages/devel/lib/liballegro_node.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liballegro_node.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liballegro_node.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liballegro_node.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp"
         RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers" TYPE EXECUTABLE FILES "/home/work/code_repository/ros_packages/devel/lib/allegro_hand_controllers/allegro_node_grasp")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp"
         OLD_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_grasp")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd"
         RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers" TYPE EXECUTABLE FILES "/home/work/code_repository/ros_packages/devel/lib/allegro_hand_controllers/allegro_node_pd")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd"
         OLD_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_pd")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque"
         RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers" TYPE EXECUTABLE FILES "/home/work/code_repository/ros_packages/devel/lib/allegro_hand_controllers/allegro_node_torque")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque"
         OLD_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_torque")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat"
         RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers" TYPE EXECUTABLE FILES "/home/work/code_repository/ros_packages/devel/lib/allegro_hand_controllers/allegro_node_velsat")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat"
         OLD_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "/home/work/code_repository/ros_packages/devel/lib:/opt/ros/kinetic/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/allegro_hand_controllers/allegro_node_velsat")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/allegro_hand_controllers/" TYPE DIRECTORY FILES "/home/work/code_repository/ros_packages/src/allegro-hand-ros/allegro_hand_controllers/launch" FILES_MATCHING REGEX "/[^/]*\\.launch$")
endif()


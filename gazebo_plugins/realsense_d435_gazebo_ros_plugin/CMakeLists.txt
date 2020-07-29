cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED PATH /opt/ros/kinetic/lib)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${roscpp_INCLUDE_DIRS})
link_directories(${std_msgs_INCLUDE_DIRS})
link_directories(${Boost_INCLUDE_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(RealSenseCamRosPlugin SHARED RealSenseRosPlugin.cc)
target_link_libraries(RealSenseCamRosPlugin ${GAZEBO_libraries} ${image_transport_LIBRARIES} ${camera_info_manager_LIBRARIES})


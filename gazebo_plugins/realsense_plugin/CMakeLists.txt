cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(RealSensePlugin SHARED RealSensePlugin.cc)
target_link_libraries(RealSensePlugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
#find_package(Eigen REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
#include_directories(${EIGEN_INCLUDE_DIRS})

#add_definitions(${EIGEN_DEFINITIONS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(generic_interface_plugin SHARED generic_interface_plugin.cc)
target_link_libraries(generic_interface_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

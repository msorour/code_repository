cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

find_package(PCL 1.6 REQUIRED COMPONENTS)
find_package(VTK REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set (CMAKE_CXX_FLAGS "-O2 -Wall")
set (CMAKE_CXX_STANDARD 11)


add_executable(grasping_algorithm_real_experiments grasping_algorithm_real_experiments.cpp)
target_link_libraries(grasping_algorithm_real_experiments pthread ${PCL_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

#add_executable (cloud_visualizer cloud_visualizer.cpp)
#target_link_libraries (cloud_visualizer ${PCL_LIBRARIES})

#add_executable (cluster_extraction cluster_extraction.cpp)
#target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

#add_executable (step_1_downsampling_object_cloud step_1_downsampling_object_cloud.cpp)
#target_link_libraries (step_1_downsampling_object_cloud ${PCL_LIBRARIES})

#add_executable (grasping_algorithm grasping_algorithm.cpp)
#target_link_libraries (grasping_algorithm ${PCL_LIBRARIES})



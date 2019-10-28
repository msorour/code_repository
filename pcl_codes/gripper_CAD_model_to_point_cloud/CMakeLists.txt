cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

find_package(PCL 1.8 REQUIRED COMPONENTS)
find_package(VTK REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set (CMAKE_CXX_FLAGS "-O2 -Wall")
set (CMAKE_CXX_STANDARD 11)

add_executable(allegro_right_hand_model_to_point_cloud  allegro_right_hand_model_to_point_cloud.cpp)
add_executable(franka_gripper_model_to_point_cloud      franka_gripper_model_to_point_cloud.cpp)
add_executable(reflex_plus_model_to_point_cloud         reflex_plus_model_to_point_cloud.cpp)
add_executable(cloud_visualizer cloud_visualizer.cpp)

target_link_libraries(allegro_right_hand_model_to_point_cloud ${PCL_LIBRARIES})
target_link_libraries(franka_gripper_model_to_point_cloud ${PCL_LIBRARIES})
target_link_libraries(reflex_plus_model_to_point_cloud ${PCL_LIBRARIES})
target_link_libraries(cloud_visualizer ${PCL_LIBRARIES})


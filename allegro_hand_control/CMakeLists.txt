cmake_minimum_required(VERSION 2.8.3)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
project(AllegroHandControl)

find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(VTK REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

#set (CMAKE_CXX_FLAGS "-O2 -Wall")
set (CMAKE_CXX_STANDARD 11)


add_executable(grasp src/main.cpp src/canAPI.cpp src/interface.cpp)
target_link_libraries(grasp BHand pcanbasic pthread)

add_executable(allegro_right_hand_position_controller src/allegro_right_hand_position_controller.cpp src/canAPI.cpp)
target_link_libraries(allegro_right_hand_position_controller BHand pcanbasic pthread ${roscpp_LIBRARIES})

add_executable(allegro_right_hand_position_controller_traj src/allegro_right_hand_position_controller_traj.cpp src/canAPI.cpp)
target_link_libraries(allegro_right_hand_position_controller_traj BHand pcanbasic pthread ${roscpp_LIBRARIES})

add_executable(publish_finger_tip_position_commands src/publish_finger_tip_position_commands.cpp)
target_link_libraries(publish_finger_tip_position_commands ${roscpp_LIBRARIES})

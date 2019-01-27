cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

#project(MY_GRAND_PROJECT)
find_package(PCL 1.9 REQUIRED COMPONENTS)
find_package(VTK REQUIRED)
#find_package(KinectSDK2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
#include_directories(${KinectSDK2_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
#link_directories(${KinectSDK2_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set (CMAKE_CXX_FLAGS "-O2 -Wall")
set (CMAKE_CXX_STANDARD 11)

add_executable(step_1_allegro_right_hand_model_to_point_cloud step_1_allegro_right_hand_model_to_point_cloud.cpp)
add_executable(step_2_hand_workspace_sampling step_2_hand_workspace_sampling.cpp)
add_executable(step_3_hand_workspace_ellipsoids step_3_hand_workspace_ellipsoids.cpp)
add_executable(step_3_hand_workspace_spheres step_3_hand_workspace_spheres.cpp)
add_executable(step_4_object_pose_approximation step_4_object_pose_approximation.cpp)
add_executable(step_5_region_of_interest step_5_region_of_interest.cpp)
add_executable(step_6_allign_hand_with_roi step_6_allign_hand_with_roi.cpp)
add_executable(step_7_touch_the_object_with_palm step_7_touch_the_object_with_palm.cpp)
add_executable(step_8_filter_hand_workspace_merging_with_object_surface step_8_filter_hand_workspace_merging_with_object_surface.cpp)

add_executable(cloud_visualizer cloud_visualizer.cpp)

add_executable(region_of_interest region_of_interest.cpp)
add_executable(region_of_interest_spheres region_of_interest_spheres.cpp)



target_link_libraries(step_1_allegro_right_hand_model_to_point_cloud ${PCL_LIBRARIES})
target_link_libraries(step_2_hand_workspace_sampling ${PCL_LIBRARIES})
target_link_libraries(step_3_hand_workspace_ellipsoids ${PCL_LIBRARIES})
target_link_libraries(step_3_hand_workspace_spheres ${PCL_LIBRARIES})
target_link_libraries(step_4_object_pose_approximation ${PCL_LIBRARIES})
target_link_libraries(step_5_region_of_interest ${PCL_LIBRARIES})
target_link_libraries(step_6_allign_hand_with_roi ${PCL_LIBRARIES})
target_link_libraries(step_7_touch_the_object_with_palm ${PCL_LIBRARIES})
target_link_libraries(step_8_filter_hand_workspace_merging_with_object_surface ${PCL_LIBRARIES})

target_link_libraries(cloud_visualizer ${PCL_LIBRARIES})

target_link_libraries(region_of_interest ${PCL_LIBRARIES} quadprog)
target_link_libraries(region_of_interest_spheres ${PCL_LIBRARIES} quadprog)

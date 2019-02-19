/*
run this program using:
reset && cmake .. && make && ./grasping_algorithm allegro_right_hand_model_cloud_plus_camera.pcd 0.01 thermos.pcd 0.007 table.pcd
reset && cmake .. && make && ./grasping_algorithm franka_gripper_model_cloud_plus_camera.pcd 0.01 thermos.pcd 0.007 table.pcd
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "include/useful_implementations.h"
#include "include/grasping_algorithm.h"
#include <math.h>
#include "/home/work/software/QuadProgpp-master/src/QuadProg++.hh"

/*
#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h>   // for sleep
#include "../../ros_packages/include/Eigen/Dense"
#include "../../ros_packages/include/useful_implementations.h"
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <time.h>
#include "/home/work/software/QuadProgpp-master/src/QuadProg++.hh"
*/


int main (int argc, char** argv){
  std::string gripper_file_name      = argv[1];
  double gripper_leaf_size           = std::stof( argv[2] );
  std::string object_file_name       = argv[3];
  double object_leaf_size            = std::stof( argv[4] );
  std::string object_plane_file_name = argv[5];
  
  clock_t begin, begin2, begin3, begin4, begin5, end;
	double time_spent;
	
	begin = clock();
	std::string gripper_model;
	if(gripper_file_name.find("allegro_right_hand")!=std::string::npos){
    gripper_model = "allegro_right_hand";
  }
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){
    gripper_model = "franka_gripper";
  }
  
  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_cloud_in_camera_frame_xyz                                   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_in_camera_frame_xyzrgb                                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_in_object_frame_xyzrgb                                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_cloud_in_gripper_frame_xyz                                  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_in_gripper_frame_xyzrgb                               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_transformed_in_gripper_frame_xyzrgb                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_sampling_in_object_frame_xyzrgb                             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_sampling_in_camera_optical_frame_xyzrgb                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_sampling_in_gripper_frame_xyzrgb                            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_cloud_downsampled_in_camera_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_downsampled_in_camera_frame_xyzrgb                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_cloud_downsampled_in_gripper_frame_xyz                      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_downsampled_in_gripper_frame_xyzrgb                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_downsampled_inside_workspace_in_gripper_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_plane_cloud_in_camera_frame_xyz           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_plane_cloud_in_camera_frame_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_plane_cloud_in_object_plane_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_plane_cloud_in_object_plane_frame_xyz     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_plane_cloud_in_gripper_frame_xyz          (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_plane_transformed_cloud_xyzrgb            (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    gripper_cloud_in_gripper_frame_xyz                      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_cloud_in_gripper_frame_xyzrgb                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_cloud_transformed_in_gripper_frame_xyzrgb       (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_cloud_transformed_in_object_plane_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_cloud_in_arm_link8_frame_xyzrgb                 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_cloud_in_workspace_centroid_frame_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_xyzrgb                               (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_workspace_spheres_best           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr index_workspace_spheres_best           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_workspace_spheres_best          (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pinky_workspace_spheres_best           (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_finger_workspace_spheres_best    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_finger_workspace_spheres_best     (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> black_color       (gripper_cloud_transformed_in_gripper_frame_xyzrgb, 0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> brown_color       (object_plane_transformed_cloud_xyzrgb, 165, 42, 42);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> magenta_color     (object_cloud_in_gripper_frame_xyzrgb, 255, 0, 255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color         (thumb_workspace_spheres_best, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color       (index_workspace_spheres_best, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color        (middle_workspace_spheres_best, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color        (pinky_workspace_spheres_best, 100, 100, 100);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color_again   (right_finger_workspace_spheres_best, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color_again (left_finger_workspace_spheres_best, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color_again  (object_sampling_in_gripper_frame_xyzrgb, 0, 0, 255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> black_color_again (gripper_cloud_transformed_in_object_plane_frame_xyzrgb, 0, 0, 0);
  
  
  
  
  Eigen::Vector3f dummy_translation;   dummy_translation << 0, 0, 0;
  Eigen::Matrix3f dummy_rotation = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f dummy_transform = Eigen::Matrix4f::Identity();
  
  // Read in the cloud data
  pcl::PCDReader reader;
  reader.read(gripper_file_name, *gripper_cloud_in_gripper_frame_xyz);
  reader.read(object_file_name, *object_cloud_in_camera_frame_xyz);
  reader.read(object_plane_file_name, *object_plane_cloud_in_camera_frame_xyz);
  copyPointCloud(*object_cloud_in_camera_frame_xyz, *object_cloud_in_camera_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  copyPointCloud(*object_plane_cloud_in_camera_frame_xyz, *object_plane_cloud_in_camera_frame_xyzrgb);
  //copyPointCloud(*gripper_cloud_in_gripper_frame_xyz, *gripper_cloud_in_gripper_frame_xyzrgb);
    
  std::cout << "Object point cloud before downsampling has:      " << object_cloud_in_camera_frame_xyz->points.size()        << " data points." << std::endl; //*
  std::cout << "Table plane point cloud before downsampling has: " << object_plane_cloud_in_camera_frame_xyz->points.size()  << " data points." << std::endl; //*
  std::cout << "Gripper point cloud before downsampling has:     " << gripper_cloud_in_gripper_frame_xyz->points.size()       << " data points." << std::endl; //*
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(object_cloud_in_camera_frame_xyz);
  vg.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  vg.filter (*object_cloud_downsampled_in_camera_frame_xyz);
  std::cout << "Object point cloud after downsampling has:       " << object_cloud_downsampled_in_camera_frame_xyz->points.size()  << " data points." << std::endl; //*
  
  std::string downsampled_object_file_name;
  downsampled_object_file_name = object_file_name.substr(0, object_file_name.size()-4);
  downsampled_object_file_name = downsampled_object_file_name + "_downsampled.pcd";
  
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>(downsampled_object_file_name, *object_cloud_downsampled_in_camera_frame_xyz, false);
  
  copyPointCloud(*object_cloud_downsampled_in_camera_frame_xyz, *object_cloud_downsampled_in_camera_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  
  
  // downsampling gripper point cloud
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(gripper_cloud_in_gripper_frame_xyz);
  vg.setLeafSize(gripper_leaf_size, gripper_leaf_size, gripper_leaf_size);
  vg.filter (*gripper_cloud_downsampled);
  std::cout << "Gripper point cloud after downsampling has:       " << gripper_cloud_downsampled->points.size()  << " data points." << std::endl; //*
  
  copyPointCloud(*gripper_cloud_downsampled, *gripper_cloud_in_gripper_frame_xyzrgb);
  
  
  
  
  
  
  // for point cloud visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_cloud_viewer  (new pcl::visualization::PCLVisualizer ("scene cloud viewer"));
  scene_cloud_viewer->addCoordinateSystem(0.2);   // this is arm link8 frame (the origin)
  scene_cloud_viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
  scene_cloud_viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>scene_cloud_rgb(scene_cloud_xyzrgb);
  scene_cloud_viewer->addPointCloud(scene_cloud_xyzrgb, scene_cloud_rgb, "scene cloud viewer");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud viewer");
  
  
  
  
  
  
  
  
  
  
  
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  
  // all processing should be done with respect to arm link8 frame, so we consider it as the origin for the moment
  // frame 8 is 45 degrees in +ve z axis
  
  // THIS IS NOT CORRECT !!!
  // I MAKE THIS MANUALLY TO SAVE TIME !!!
  // camera optical frame wrt arm link8 frame
  //Eigen::Vector3f camera_optical_frame_wrt_arm_link8_frame_translation;   camera_optical_frame_wrt_arm_link8_frame_translation << -0.03338934, 0.07033271, -0.02732752;   // original !? not working
  Eigen::Vector3f camera_optical_frame_wrt_arm_link8_frame_translation;   camera_optical_frame_wrt_arm_link8_frame_translation << 0.03338934, -0.07033271, -0.02732752;
  //Eigen::Matrix3f camera_optical_frame_wrt_arm_link8_frame_rotation = Eigen::Quaternionf{Eigen::AngleAxisf{0.3659975, Eigen::Vector3f{-0.01241378, 0.01459942, -0.9304185}}}.toRotationMatrix();   // original !? not working
  Eigen::Matrix3f camera_optical_frame_wrt_arm_link8_frame_rotation = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f camera_optical_frame_wrt_arm_link8_frame_transform;
  camera_optical_frame_wrt_arm_link8_frame_transform << camera_optical_frame_wrt_arm_link8_frame_rotation, camera_optical_frame_wrt_arm_link8_frame_translation,
                                                        0,0,0,1;
  dummy_translation << 0,0,0;
  dummy_rotation = Rotz_float(M_PI/4);
  dummy_transform << dummy_rotation, dummy_translation,
                     0,0,0,1;
  camera_optical_frame_wrt_arm_link8_frame_transform = camera_optical_frame_wrt_arm_link8_frame_transform*dummy_transform;
  //std::cout << "camera_optical_frame_wrt_arm_link8_frame_transform = " << std::endl << camera_optical_frame_wrt_arm_link8_frame_transform << std::endl;
  auto euler = camera_optical_frame_wrt_arm_link8_frame_rotation.eulerAngles(0, 1, 2);
  //std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler.transpose() << std::endl;
  // show frame
  transform.matrix() = camera_optical_frame_wrt_arm_link8_frame_transform;
  scene_cloud_viewer->addCoordinateSystem(0.07, transform, "camera optical frame", 0);
  
  
  
  
  //
  // gripper frame wrt arm link8 frame
  Eigen::Vector3f gripper_wrt_arm_link8_frame_translation;
  Eigen::Matrix3f gripper_wrt_arm_link8_frame_rotation = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f gripper_wrt_arm_link8_frame_transform;
  Eigen::Matrix4f gripper_wrt_arm_link8_frame_inverse_transform;
  
  if(gripper_model == "allegro_right_hand"){
    gripper_wrt_arm_link8_frame_translation << 0.0204, 0, 0.02;
    gripper_wrt_arm_link8_frame_transform << gripper_wrt_arm_link8_frame_rotation, gripper_wrt_arm_link8_frame_translation,
                                             0,0,0,1;
    dummy_translation << 0,0,0;
    dummy_rotation = Rotz_float(-M_PI/4);
    dummy_transform << dummy_rotation, dummy_translation,
                       0,0,0,1;
    gripper_wrt_arm_link8_frame_transform = dummy_transform*gripper_wrt_arm_link8_frame_transform;
  }
  else if(gripper_model == "franka_gripper"){
    gripper_wrt_arm_link8_frame_translation << 0.0204, 0, 0.02;
    gripper_wrt_arm_link8_frame_transform << gripper_wrt_arm_link8_frame_rotation, gripper_wrt_arm_link8_frame_translation,
                                             0,0,0,1;
    dummy_translation << 0,0,0;
    dummy_rotation = Rotz_float(-M_PI/4);
    dummy_transform << dummy_rotation, dummy_translation,
                       0,0,0,1;
    gripper_wrt_arm_link8_frame_transform = dummy_transform*gripper_wrt_arm_link8_frame_transform;
  }
  std::cout<< "******* gripper_wrt_arm_link8_frame_transform = " << std::endl << gripper_wrt_arm_link8_frame_transform << std::endl;
  
  // DEACTIVATE TEMPORARY !!!!
  // BELOW IS THE TRUE VALUE TO BE USED IN EXPERIMENTS
  /*
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){
    gripper_wrt_arm_link8_frame_translation << 0, 0, 0.005;
    gripper_wrt_arm_link8_frame_rotation = Rotz_float(-M_PI/4);
    gripper_wrt_arm_link8_frame_transform << gripper_wrt_arm_link8_frame_rotation, gripper_wrt_arm_link8_frame_translation,
                                             0,0,0,1;
  }
  */
  // show frame
  transform.matrix() = gripper_wrt_arm_link8_frame_transform;
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "gripper frame", 0);
  
  // gripper frame wrt arm link8 frame [INVERSE]
  gripper_wrt_arm_link8_frame_inverse_transform << gripper_wrt_arm_link8_frame_rotation.transpose(), -gripper_wrt_arm_link8_frame_rotation.transpose()*gripper_wrt_arm_link8_frame_translation,  // from khalil's book page 21
                                                   0, 0, 0, 1;
  
  
  ///
  ///
  ///
  // THE problem with different object plane orienations come from here:
  // gripper_wrt_arm_link8_frame_rotation
  // the above equations are valid at experiments where the camera frame with respect to gripper frame are different from allegro hand to franka gripper
  // but here since we got only one real point cloud using allegro hand mounted camera we have to use the allegro hand transform
  //
  //
  //
  
  
  
  
  //
  // camera optical frame wrt gripper frame
  Eigen::Vector3f camera_optical_frame_wrt_gripper_frame_translation;
  Eigen::Matrix3f camera_optical_frame_wrt_gripper_frame_rotation;
  Eigen::Matrix4f camera_optical_frame_wrt_gripper_frame_transform;
  camera_optical_frame_wrt_gripper_frame_transform = gripper_wrt_arm_link8_frame_inverse_transform*camera_optical_frame_wrt_arm_link8_frame_transform;
  
  
  
  
  
  
  
  // transforming point clouds
  //pcl::transformPointCloud(*gripper_cloud_in_gripper_frame_xyzrgb, *gripper_cloud_in_arm_link8_frame_xyzrgb, gripper_wrt_arm_link8_frame_transform);
  pcl::transformPointCloud(*object_cloud_in_camera_frame_xyzrgb, *object_cloud_in_gripper_frame_xyzrgb, camera_optical_frame_wrt_gripper_frame_transform);
  pcl::transformPointCloud(*object_cloud_in_camera_frame_xyz, *object_cloud_in_gripper_frame_xyz, camera_optical_frame_wrt_gripper_frame_transform);
  //pcl::transformPointCloud(*object_plane_cloud_in_camera_frame_xyzrgb, *object_plane_transformed_cloud_xyzrgb, camera_optical_frame_wrt_gripper_frame_transform);
  
  
  
  
  
  
  
  
  
  
  
  
  
  // GRASPING ALGORITHM
  // object pose
  // STEP#4 : compute object transformation matrix
  Eigen::Matrix3f object_rotation;
  Eigen::Vector3f object_translation;
  Eigen::Matrix4f object_transform;
  object_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector4f object_far_point_in_pos_direction_in_global_frame;
  Eigen::Vector4f object_far_point_in_neg_direction_in_global_frame;
  Eigen::Vector3f object_major_dimensions;
  //step_4_object_pose_approximation( *object_cloud_in_camera_frame_xyz, object_transform, object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions );
  step_4_object_pose_approximation( *object_cloud_in_gripper_frame_xyz, object_transform, object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions );
  transform.matrix() = object_transform;
  scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  object_rotation    << object_transform(0,0), object_transform(0,1), object_transform(0,2),
                        object_transform(1,0), object_transform(1,1), object_transform(1,2),
                        object_transform(2,0), object_transform(2,1), object_transform(2,2);
  object_translation << object_transform(0,3), object_transform(1,3), object_transform(2,3);
  
  // object cloud in its own frame
  Eigen::Matrix4f inverse_object_transform;
  inverse_object_transform << object_rotation.transpose(), -object_rotation.transpose()*object_translation,  // from khalil's book page 21
                              0, 0, 0, 1;
  
  // object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid;
  pcl::PointXYZ object_centroid_point;
  for(unsigned int i=0;i<object_cloud_in_camera_frame_xyz->points.size();i++)
    object_centroid.add( object_cloud_in_camera_frame_xyz->points[i] );
  object_centroid.get(object_centroid_point);
  
  
  
  
  
  
  
  
  
  
  
  // transforming point clouds
  pcl::transformPointCloud(*gripper_cloud_in_gripper_frame_xyzrgb, *gripper_cloud_in_arm_link8_frame_xyzrgb, gripper_wrt_arm_link8_frame_transform);
  pcl::transformPointCloud(*object_cloud_in_camera_frame_xyzrgb, *object_cloud_in_gripper_frame_xyzrgb, camera_optical_frame_wrt_gripper_frame_transform);
  
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_frame_xyzrgb, *object_cloud_downsampled_in_gripper_frame_xyzrgb, camera_optical_frame_wrt_gripper_frame_transform);
  
  pcl::transformPointCloud(*object_plane_cloud_in_camera_frame_xyzrgb, *object_plane_transformed_cloud_xyzrgb, camera_optical_frame_wrt_gripper_frame_transform);
  pcl::transformPointCloud(*object_plane_cloud_in_camera_frame_xyz, *object_plane_cloud_in_gripper_frame_xyz, camera_optical_frame_wrt_gripper_frame_transform);
  
  
  //pcl::transformPointCloud(*object_cloud_in_camera_frame_xyzrgb, *object_cloud_in_object_frame_xyzrgb, inverse_object_transform);
  pcl::transformPointCloud(*object_cloud_in_gripper_frame_xyzrgb, *object_cloud_in_object_frame_xyzrgb, inverse_object_transform);
  
  //*object_cloud_in_object_frame_xyzrgb += *object_cloud_in_camera_frame_xyzrgb;
  
  
  for(unsigned int i=0; i<object_cloud_in_gripper_frame_xyzrgb->points.size(); i++){
    object_cloud_in_gripper_frame_xyzrgb->points[i].r=255;
  }
  for(unsigned int i=0; i<object_plane_transformed_cloud_xyzrgb->points.size(); i++){
    object_plane_transformed_cloud_xyzrgb->points[i].r=255;
  }
  
  
  scene_cloud_viewer->addPointCloud(object_plane_transformed_cloud_xyzrgb, brown_color, "table cloud");
  scene_cloud_viewer->addPointCloud(object_cloud_in_gripper_frame_xyzrgb, magenta_color, "object cloud");
  
  scene_cloud_viewer->addPointCloud(object_cloud_in_gripper_frame_xyzrgb, magenta_color, "object cloud");
  
  
  
  
  *scene_cloud_xyzrgb += *gripper_cloud_in_arm_link8_frame_xyzrgb;
  //*scene_cloud_xyzrgb += *object_sampling_in_gripper_frame_xyzrgb;
  
  
  
  
  
  
  
  
  
  
  
  
  
  // gripper support region special ellipsoid
  double gripper_support_x; double gripper_support_y; double gripper_support_z;
  double gripper_support_offset_x; double gripper_support_offset_y; double gripper_support_offset_z;
  pcl::PointCloud<pcl::PointXYZRGB> gripper_support_point_cloud_in_gripper_frame;
  pcl::PointXYZRGB point_xyzrgb;
  pcl::PointXYZ point_xyz;
  
  if(gripper_model == "allegro_right_hand"){
    gripper_support_x = 0.003; gripper_support_y = 0.047; gripper_support_z = 0.047;
    gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.0475;
  }
  else if(gripper_model == "franka_gripper"){
    gripper_support_x = 0.02; gripper_support_y = 0.08; gripper_support_z = 0.001;
    gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.065;
  }
  // draw the special ellipsoid
  // centered at origin with orientation coincident to that of origin frame
  gripper_support_point_cloud_in_gripper_frame.clear();
  int point_cloud_samples = 250;
  double value_x, value_y, value_z;
  for(unsigned int k=0; k<point_cloud_samples; k++){
    value_x = (-gripper_support_x + gripper_support_offset_x) + k*2*gripper_support_x/point_cloud_samples;
    for(unsigned int l=0; l<point_cloud_samples; l++){
      value_y = (-gripper_support_y + gripper_support_offset_y) + l*2*gripper_support_y/point_cloud_samples;
      value_z = gripper_support_offset_z + gripper_support_z*sqrt( 1 - pow(value_x-gripper_support_offset_x, 10)/pow(gripper_support_x, 10) - pow(value_y-gripper_support_offset_y, 10)/pow(gripper_support_y, 10) );
      point_xyzrgb.x = value_x;
      point_xyzrgb.y = value_y;
      point_xyzrgb.z = value_z;
      point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
      gripper_support_point_cloud_in_gripper_frame.points.push_back( point_xyzrgb );
      
      value_z = gripper_support_offset_z - gripper_support_z*sqrt( 1 - pow(value_x-gripper_support_offset_x, 10)/pow(gripper_support_x, 10) - pow(value_y-gripper_support_offset_y, 10)/pow(gripper_support_y, 10) );
      point_xyzrgb.x = value_x;
      point_xyzrgb.y = value_y;
      point_xyzrgb.z = value_z;
      point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
      gripper_support_point_cloud_in_gripper_frame.points.push_back( point_xyzrgb );    
    }
  }
  
  //*gripper_cloud_in_gripper_frame_xyzrgb += gripper_support_point_cloud_in_gripper_frame;
  
  
  
  
  
  
  // object plane special ellipsoid
  double object_plane_x; double object_plane_y; double object_plane_z;
  double object_plane_offset_x; double object_plane_offset_y; double object_plane_offset_z;
  pcl::PointCloud<pcl::PointXYZRGB> object_plane_special_ellipsoid_point_cloud_in_object_plane_frame;
  
  // object plane pose
  // compute object plane transformation matrix
  Eigen::Matrix3f object_plane_rotation;
  Eigen::Vector3f object_plane_translation;
  Eigen::Matrix4f object_plane_transform;
  object_plane_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector4f object_plane_far_point_in_pos_direction_in_global_frame;
  Eigen::Vector4f object_plane_far_point_in_neg_direction_in_global_frame;
  Eigen::Vector3f object_plane_major_dimensions;
  //step_4_object_pose_approximation( *object_plane_cloud_in_camera_frame_xyz, object_plane_transform, object_plane_far_point_in_pos_direction_in_global_frame, object_plane_far_point_in_neg_direction_in_global_frame, object_plane_major_dimensions );
  step_4_object_pose_approximation( *object_plane_cloud_in_gripper_frame_xyz, object_plane_transform, object_plane_far_point_in_pos_direction_in_global_frame, object_plane_far_point_in_neg_direction_in_global_frame, object_plane_major_dimensions );
  //transform.matrix() = object_plane_transform;
  //scene_cloud_viewer->addCoordinateSystem(0.3, transform, "object plane frame", 0);
  
  object_plane_rotation    << object_plane_transform(0,0), object_plane_transform(0,1), object_plane_transform(0,2),
                              object_plane_transform(1,0), object_plane_transform(1,1), object_plane_transform(1,2),
                              object_plane_transform(2,0), object_plane_transform(2,1), object_plane_transform(2,2);
  object_plane_translation << object_plane_transform(0,3), object_plane_transform(1,3), object_plane_transform(2,3);
  
  //std::cout << "object_plane_transform = " << std::endl << object_plane_transform << std::endl;
  // object plane cloud in its own frame
  Eigen::Matrix4f inverse_object_plane_transform;
  inverse_object_plane_transform << object_plane_rotation.transpose(), -object_plane_rotation.transpose()*object_plane_translation,  // from khalil's book page 21
                                    0, 0, 0, 1;
  pcl::transformPointCloud(*object_plane_cloud_in_gripper_frame_xyz, *object_plane_cloud_in_object_plane_frame_xyz, inverse_object_plane_transform);
  copyPointCloud(*object_plane_cloud_in_object_plane_frame_xyz, *object_plane_cloud_in_object_plane_frame_xyzrgb);
  
  
  // object plane centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_plane_centroid;
  pcl::PointXYZ object_plane_centroid_point;
  for(unsigned int i=0;i<object_plane_cloud_in_camera_frame_xyz->points.size();i++)
    object_plane_centroid.add( object_plane_cloud_in_camera_frame_xyz->points[i] );
  object_plane_centroid.get(object_plane_centroid_point);
  
  object_plane_x = 0.35; object_plane_y = 0.05; object_plane_z = 0.35;
  object_plane_offset_x = 0.0; object_plane_offset_y = 0.05; object_plane_offset_z = 0.0;
  
  // draw the special ellipsoid
  // centered at origin with orientation coincident to that of origin frame
  object_plane_special_ellipsoid_point_cloud_in_object_plane_frame.clear();
  point_cloud_samples = 500;
  for(unsigned int k=0; k<point_cloud_samples; k++){
    value_x = (-object_plane_x + object_plane_offset_x) + k*2*object_plane_x/point_cloud_samples;
    for(unsigned int l=0; l<point_cloud_samples; l++){
      value_y = (-object_plane_y + object_plane_offset_y) + l*2*object_plane_y/point_cloud_samples;
      value_z = object_plane_offset_z + object_plane_z*sqrt( 1 - pow(value_x-object_plane_offset_x, 10)/pow(object_plane_x, 10) - pow(value_y-object_plane_offset_y, 10)/pow(object_plane_y, 10) );
      point_xyzrgb.x = value_x;
      point_xyzrgb.y = value_y;
      point_xyzrgb.z = value_z;
      point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
      object_plane_special_ellipsoid_point_cloud_in_object_plane_frame.points.push_back( point_xyzrgb );
      
      value_z = object_plane_offset_z - object_plane_z*sqrt( 1 - pow(value_x-object_plane_offset_x, 10)/pow(object_plane_x, 10) - pow(value_y-object_plane_offset_y, 10)/pow(object_plane_y, 10) );
      point_xyzrgb.x = value_x;
      point_xyzrgb.y = value_y;
      point_xyzrgb.z = value_z;
      point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
      object_plane_special_ellipsoid_point_cloud_in_object_plane_frame.points.push_back( point_xyzrgb );    
    }
  }
  *object_plane_cloud_in_object_plane_frame_xyzrgb += object_plane_special_ellipsoid_point_cloud_in_object_plane_frame;
  //*object_plane_cloud_in_camera_frame_xyzrgb += object_plane_special_ellipsoid_point_cloud_in_object_plane_frame;
  
  
  
  
  
  
  std::vector<double> gripper_x, gripper_y, gripper_z;
  std::vector<double> gripper_offset_x, gripper_offset_y, gripper_offset_z;
  
  if(gripper_model == "allegro_right_hand"){
    // approximating the allegro hand into a set of special ellipsoids
    // palm
    gripper_x.push_back(0.017);
    gripper_y.push_back(0.059);
    gripper_z.push_back(0.059);
    gripper_offset_x.push_back(-0.017);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(0.057);
    /*
    // index
    gripper_x.push_back(0.014);
    gripper_y.push_back(0.016);
    gripper_z.push_back(0.07);
    gripper_offset_x.push_back(-0.014);
    gripper_offset_y.push_back(0.05);
    gripper_offset_z.push_back(0.17);
    
    // middle
    gripper_x.push_back(0.014);
    gripper_y.push_back(0.016);
    gripper_z.push_back(0.07);
    gripper_offset_x.push_back(-0.014);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(0.17);
    
    // pinky
    gripper_x.push_back(0.014);
    gripper_y.push_back(0.016);
    gripper_z.push_back(0.07);
    gripper_offset_x.push_back(-0.014);
    gripper_offset_y.push_back(-0.05);
    gripper_offset_z.push_back(0.17);
    */
    
    // index, middle, pinky
    gripper_x.push_back(0.014);
    gripper_y.push_back(0.070);
    gripper_z.push_back(0.07);
    gripper_offset_x.push_back(-0.014);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(0.17);
    
    
    // thumb
    gripper_x.push_back(0.014);
    gripper_y.push_back(0.08);
    gripper_z.push_back(0.012);
    gripper_offset_x.push_back(-0.022);
    gripper_offset_y.push_back(0.11);
    gripper_offset_z.push_back(0.022);
    
    // connection
    gripper_x.push_back(0.04);
    gripper_y.push_back(0.04);
    gripper_z.push_back(0.02);
    gripper_offset_x.push_back(-0.02);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(-0.01);
    
    // realsense camera
    gripper_x.push_back(0.015);
    gripper_y.push_back(0.045);
    gripper_z.push_back(0.015);
    gripper_offset_x.push_back(0.05);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(-0.05);
    
  }
  else if(gripper_model == "franka_gripper"){
    // approximating the panda gripper into a set of special ellipsoids
    // gripper base
    gripper_x.push_back(0.025);
    gripper_y.push_back(0.11);
    gripper_z.push_back(0.045);
    gripper_offset_x.push_back(0.0);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(0.0225);
    
    // right finger
    gripper_x.push_back(0.01);
    gripper_y.push_back(0.01);
    gripper_z.push_back(0.03);
    gripper_offset_x.push_back(0.0);
    gripper_offset_y.push_back(0.05);
    gripper_offset_z.push_back(0.09);
    
    // left finger
    gripper_x.push_back(0.01);
    gripper_y.push_back(0.01);
    gripper_z.push_back(0.03);
    gripper_offset_x.push_back(0.0);
    gripper_offset_y.push_back(-0.05);
    gripper_offset_z.push_back(0.09);
    
    // connection
    gripper_x.push_back(0.05);
    gripper_y.push_back(0.04);
    gripper_z.push_back(0.02);
    gripper_offset_x.push_back(0.01);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(-0.02);
    
    // realsense camera
    gripper_x.push_back(0.015);
    gripper_y.push_back(0.045);
    gripper_z.push_back(0.015);
    gripper_offset_x.push_back(0.07);
    gripper_offset_y.push_back(0.0);
    gripper_offset_z.push_back(-0.03);
  }
  
  
  pcl::PointCloud<pcl::PointXYZRGB> gripper_as_set_of_special_ellipsoids;
  point_cloud_samples = 100;
  // draw the special ellipsoid
  for(unsigned int j=0; j<gripper_x.size(); j++){
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-gripper_x[j] + gripper_offset_x[j]) + k*2*gripper_x[j]/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-gripper_y[j] + gripper_offset_y[j]) + l*2*gripper_y[j]/point_cloud_samples;
        value_z = gripper_offset_z[j] + gripper_z[j]*sqrt( 1 - pow(value_x-gripper_offset_x[j], 10)/pow(gripper_x[j], 10) - pow(value_y-gripper_offset_y[j], 10)/pow(gripper_y[j], 10) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        gripper_as_set_of_special_ellipsoids.points.push_back( point_xyzrgb );
        
        value_z = gripper_offset_z[j] - gripper_z[j]*sqrt( 1 - pow(value_x-gripper_offset_x[j], 10)/pow(gripper_x[j], 10) - pow(value_y-gripper_offset_y[j], 10)/pow(gripper_y[j], 10) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        gripper_as_set_of_special_ellipsoids.points.push_back( point_xyzrgb );    
      }
    }
  }
  //*gripper_cloud_in_gripper_frame_xyzrgb += gripper_as_set_of_special_ellipsoids;
  
  
  
  
  
  
  
  
  
  
  // sampling the object around its z-axis for scanning
  int object_sampling_in_x_axis = 10;   int object_sampling_in_y_axis = 10;   int object_sampling_in_z_axis = 1;
  object_sampling_in_object_frame_xyzrgb->clear();
  object_major_dimensions(0) = 0.95*object_major_dimensions(0);
  object_major_dimensions(1) = 0.95*object_major_dimensions(1);
  object_major_dimensions(2) = 0.70*object_major_dimensions(2);
  for(unsigned int i=0; i<object_sampling_in_x_axis ;i++){
    for(unsigned int j=0; j<object_sampling_in_y_axis ;j++){
      for(unsigned int k=0; k<object_sampling_in_z_axis ;k++){
        point_xyzrgb.x = -object_major_dimensions(0)/2 + object_major_dimensions(0)*i/object_sampling_in_x_axis;
        point_xyzrgb.y = -object_major_dimensions(1)/2 + object_major_dimensions(1)*j/object_sampling_in_y_axis;
        point_xyzrgb.z = -object_major_dimensions(2)/2 + object_major_dimensions(2)*k/object_sampling_in_z_axis;
        point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;
        object_sampling_in_object_frame_xyzrgb->points.push_back( point_xyzrgb );
      }
    }
  }
  pcl::transformPointCloud(*object_sampling_in_object_frame_xyzrgb, *object_sampling_in_gripper_frame_xyzrgb, object_transform);
  *object_cloud_in_object_frame_xyzrgb += *object_sampling_in_object_frame_xyzrgb;
  
  scene_cloud_viewer->addPointCloud(object_sampling_in_gripper_frame_xyzrgb, blue_color_again, "object sampling cloud");
  scene_cloud_viewer->updatePointCloud(object_sampling_in_gripper_frame_xyzrgb, blue_color_again, "object sampling cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object sampling cloud");
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  //
  // STEP
  // load workspace spheres and compute the centroid in the gripper frame
  
  //
  // declarations for allegro hand
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr index_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_workspace_convex_xyzrgb       (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pinky_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // offset of convex shape (sphere)
  pcl::PointCloud<pcl::PointXYZ> thumb_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> index_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> middle_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> pinky_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> workspace_convex_offset;
  
  // parameter of convex shape = {a,b,c} for ellipsoid or {r} for sphere
  pcl::PointCloud<pcl::PointXYZ> thumb_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> index_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> middle_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> pinky_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> workspace_convex_parameter;
  
  
  //
  // declarations for panda gripper
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_finger_workspace_convex_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_finger_workspace_convex_xyzrgb   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // offset of convex shape (sphere)
  pcl::PointCloud<pcl::PointXYZ> right_finger_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> left_finger_workspace_convex_offset;
  
  // parameter of convex shape = {a,b,c} for ellipsoid or {r} for sphere
  pcl::PointCloud<pcl::PointXYZ> right_finger_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> left_finger_workspace_convex_parameter;
  
  
  //
  // declarations for both
  pcl::PointXYZ convex_shape_offset;
  pcl::PointXYZ convex_shape_parameter;
  pcl::PointXYZ offset;
  pcl::PointXYZ parameter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_augmented_workspace_xyzrgb   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  
  
  if(gripper_model == "allegro_right_hand"){
  std::vector<std::string> finger_list;
  finger_list.push_back("thumb");
  finger_list.push_back("index");
  finger_list.push_back("middle");
  finger_list.push_back("pinky");
  
  for(int i=0; i<finger_list.size(); i++){
    std::vector<double> data;
    std::string line;
    ifstream convex_workspace_file("../gripper_workspace/"+finger_list[i]+"_workspace_spheres_3.txt");
    if(convex_workspace_file.is_open()){
      while(getline( convex_workspace_file, line ) ){
        std::istringstream string_stream( line );
        std::string field;
        data.clear();
        while(string_stream){
          if(!getline( string_stream, field, ',' )) break;
          std::stringstream fs( field );
          double f = 0.0;  // (default value is 0.0)
          fs >> f;
          data.push_back( f );
        }
        
        parameter.x = data[0];
        parameter.y = data[0];
        parameter.z = data[0];
        offset.x = data[1];
        offset.y = data[2];
        offset.z = data[3];
                
        if(finger_list[i]=="thumb"){
          thumb_workspace_convex_offset.push_back( offset );
          thumb_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="index"){
          index_workspace_convex_offset.push_back( offset );
          index_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="middle"){
          middle_workspace_convex_offset.push_back( offset );
          middle_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="pinky"){
          pinky_workspace_convex_offset.push_back( offset );
          pinky_workspace_convex_parameter.push_back( parameter );
        }
      }
    }
    convex_workspace_file.close();
  }
  
  
  // generate and view the point cloud of the convex workspace
  pcl::PointXYZ convex_shape_offset, convex_shape_parameter;
  point_cloud_samples = 30;
  for(int i=0; i<finger_list.size(); i++){
    for(unsigned int j=0; j<thumb_workspace_convex_offset.size(); j++){
      if(finger_list[i]=="thumb"){
        convex_shape_offset.x = thumb_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = thumb_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = thumb_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = thumb_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = thumb_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = thumb_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="index"){
        convex_shape_offset.x = index_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = index_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = index_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = index_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = index_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = index_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="middle"){
        convex_shape_offset.x = middle_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = middle_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = middle_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = middle_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = middle_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = middle_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="pinky"){
        convex_shape_offset.x = pinky_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = pinky_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = pinky_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = pinky_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = pinky_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = pinky_workspace_convex_parameter.points[j].z;
      }
      for(unsigned int k=0; k<point_cloud_samples; k++){
        value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
        for(unsigned int l=0; l<point_cloud_samples; l++){
          value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
          value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          if(!std::isnan(value_z))
            point_xyzrgb.z = value_z;
          
          if(finger_list[i]=="thumb"){
            point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
            thumb_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          else if(finger_list[i]=="index"){
            point_xyzrgb.r = 0;  point_xyzrgb.g = 255;  point_xyzrgb.b = 0;
            index_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          else if(finger_list[i]=="middle"){
            point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;
            middle_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          else if(finger_list[i]=="pinky"){
            point_xyzrgb.r = 100;  point_xyzrgb.g = 100;  point_xyzrgb.b = 100;
            pinky_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          
          value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          if(!std::isnan(value_z))
            point_xyzrgb.z = value_z;
          
          if(finger_list[i]=="thumb"){
            point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
            thumb_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          else if(finger_list[i]=="index"){
            point_xyzrgb.r = 0;  point_xyzrgb.g = 255;  point_xyzrgb.b = 0;
            index_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          else if(finger_list[i]=="middle"){
            point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;
            middle_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
          else if(finger_list[i]=="pinky"){
            point_xyzrgb.r = 100;  point_xyzrgb.g = 100;  point_xyzrgb.b = 100;
            pinky_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
          }
        }
      }
    }
  }
  
  *gripper_augmented_workspace_xyzrgb = *thumb_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *index_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *middle_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *pinky_workspace_convex_xyzrgb;
  
  }
  else if(gripper_model == "franka_gripper"){
  
  std::vector<std::string> finger_list;
  finger_list.push_back("franka_right_finger");
  finger_list.push_back("franka_left_finger");
  
  for(int i=0; i<finger_list.size(); i++){
    std::vector<double> data;
    std::string line;
    ifstream convex_workspace_file("../gripper_workspace/"+finger_list[i]+"_workspace_spheres_10.txt");
    if(convex_workspace_file.is_open()){
      while(getline( convex_workspace_file, line ) ){
        std::istringstream string_stream( line );
        std::string field;
        data.clear();
        while(string_stream){
          if(!getline( string_stream, field, ',' )) break;
          std::stringstream fs( field );
          double f = 0.0;  // (default value is 0.0)
          fs >> f;
          data.push_back( f );
        }
        
        parameter.x = data[0];
        parameter.y = data[0];
        parameter.z = data[0];
        offset.x = data[1];
        offset.y = data[2];
        offset.z = data[3];
                
        if(finger_list[i]=="franka_right_finger"){
          right_finger_workspace_convex_offset.push_back( offset );
          right_finger_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="franka_left_finger"){
          left_finger_workspace_convex_offset.push_back( offset );
          left_finger_workspace_convex_parameter.push_back( parameter );
        }
      }
    }
    convex_workspace_file.close();
  }
  
  
  
  // generate and view the point cloud of the convex workspace
  point_cloud_samples = 30;
  for(int i=0; i<finger_list.size(); i++){
    for(unsigned int j=0; j<right_finger_workspace_convex_offset.size(); j++){
      if(finger_list[i]=="franka_right_finger"){
        convex_shape_offset.x = right_finger_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = right_finger_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = right_finger_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = right_finger_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = right_finger_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = right_finger_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="franka_left_finger"){
        convex_shape_offset.x = left_finger_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = left_finger_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = left_finger_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = left_finger_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = left_finger_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = left_finger_workspace_convex_parameter.points[j].z;
      }
      for(unsigned int k=0; k<point_cloud_samples; k++){
        value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
        for(unsigned int l=0; l<point_cloud_samples; l++){
          value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
          value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          if(!std::isnan(value_z))
            point_xyzrgb.z = value_z;
          
          if(finger_list[i]=="franka_right_finger"){
            right_finger_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
          }
          else if(finger_list[i]=="franka_left_finger"){
            left_finger_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 0;  point_xyzrgb.g = 255;  point_xyzrgb.b = 0;
          }
          
          value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          if(!std::isnan(value_z))
            point_xyzrgb.z = value_z;
          
          if(finger_list[i]=="franka_right_finger"){
            right_finger_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
          }
          else if(finger_list[i]=="franka_left_finger"){
            left_finger_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 0;  point_xyzrgb.g = 255;  point_xyzrgb.b = 0;
          } 
        }
      }
    }
  }
  
  *gripper_augmented_workspace_xyzrgb = *right_finger_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *left_finger_workspace_convex_xyzrgb;
  
  }
  
  
  // gripper augmented workspace centroid location
  pcl::CentroidPoint<pcl::PointXYZRGB> gripper_workspace_centroid;
  pcl::PointXYZRGB gripper_workspace_centroid_point_in_gripper_frame;
  for(unsigned int i=0;i<gripper_augmented_workspace_xyzrgb->points.size();i++)
    gripper_workspace_centroid.add( gripper_augmented_workspace_xyzrgb->points[i] );
  gripper_workspace_centroid.get(gripper_workspace_centroid_point_in_gripper_frame);
  
  //std::cout << "gripper centroid = " << gripper_workspace_centroid_point_in_gripper_frame << std::endl;
  
  
  // in case of allegro right hand
  // remove worspace spheres from gripper finger that passes the centroid
  if(gripper_model == "allegro_right_hand"){
    // filtering thumb workspace spheres
    workspace_convex_offset.clear();
    workspace_convex_parameter.clear();
    for(unsigned int i=0;i<thumb_workspace_convex_offset.points.size();i++){
      if(thumb_workspace_convex_offset.points[i].z < gripper_workspace_centroid_point_in_gripper_frame.z){
        workspace_convex_offset.points.push_back( thumb_workspace_convex_offset.points[i] );
        workspace_convex_parameter.points.push_back( thumb_workspace_convex_parameter.points[i] );
      }
    }
    thumb_workspace_convex_offset.clear();
    thumb_workspace_convex_parameter.clear();
    for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
      thumb_workspace_convex_offset.points.push_back( workspace_convex_offset.points[i] );
      thumb_workspace_convex_parameter.points.push_back( workspace_convex_parameter.points[i] );
    }
    
    // filtering index workspace spheres
    workspace_convex_offset.clear();
    workspace_convex_parameter.clear();
    for(unsigned int i=0;i<index_workspace_convex_offset.points.size();i++){
      if(index_workspace_convex_offset.points[i].z > gripper_workspace_centroid_point_in_gripper_frame.z){
        workspace_convex_offset.points.push_back( index_workspace_convex_offset.points[i] );
        workspace_convex_parameter.points.push_back( index_workspace_convex_parameter.points[i] );
      }
    }
    index_workspace_convex_offset.clear();
    index_workspace_convex_parameter.clear();
    for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
      index_workspace_convex_offset.points.push_back( workspace_convex_offset.points[i] );
      index_workspace_convex_parameter.points.push_back( workspace_convex_parameter.points[i] );
    }
    
    // filtering middle workspace spheres
    workspace_convex_offset.clear();
    workspace_convex_parameter.clear();
    for(unsigned int i=0;i<middle_workspace_convex_offset.points.size();i++){
      if(middle_workspace_convex_offset.points[i].z > gripper_workspace_centroid_point_in_gripper_frame.z){
        workspace_convex_offset.points.push_back( middle_workspace_convex_offset.points[i] );
        workspace_convex_parameter.points.push_back( middle_workspace_convex_parameter.points[i] );
      }
    }
    middle_workspace_convex_offset.clear();
    middle_workspace_convex_parameter.clear();
    for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
      middle_workspace_convex_offset.points.push_back( workspace_convex_offset.points[i] );
      middle_workspace_convex_parameter.points.push_back( workspace_convex_parameter.points[i] );
    }
    
    // filtering pinky workspace spheres
    workspace_convex_offset.clear();
    workspace_convex_parameter.clear();
    for(unsigned int i=0;i<pinky_workspace_convex_offset.points.size();i++){
      if(pinky_workspace_convex_offset.points[i].z > gripper_workspace_centroid_point_in_gripper_frame.z){
        workspace_convex_offset.points.push_back( pinky_workspace_convex_offset.points[i] );
        workspace_convex_parameter.points.push_back( pinky_workspace_convex_parameter.points[i] );
      }
    }
    pinky_workspace_convex_offset.clear();
    pinky_workspace_convex_parameter.clear();
    for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
      pinky_workspace_convex_offset.points.push_back( workspace_convex_offset.points[i] );
      pinky_workspace_convex_parameter.points.push_back( workspace_convex_parameter.points[i] );
    }
    
  }
  
  
  
  begin2 = clock();
  
  
  Eigen::Vector3f gripper_translation;
  Eigen::Matrix3f gripper_rotation;
  Eigen::Matrix4f gripper_transform;
  Eigen::Matrix4f inverse_gripper_transform;
  
  Eigen::Vector3f workspace_centroid_wrt_gripper_frame_translation;
  Eigen::Matrix3f workspace_centroid_wrt_gripper_frame_rotation;
  Eigen::Matrix4f workspace_centroid_wrt_gripper_frame_transform;
  Eigen::Matrix4f workspace_centroid_wrt_gripper_frame_transform_inverse;
  workspace_centroid_wrt_gripper_frame_translation << gripper_workspace_centroid_point_in_gripper_frame.x, gripper_workspace_centroid_point_in_gripper_frame.y, gripper_workspace_centroid_point_in_gripper_frame.z;
  workspace_centroid_wrt_gripper_frame_rotation = Eigen::Matrix3f::Identity();
  workspace_centroid_wrt_gripper_frame_transform << workspace_centroid_wrt_gripper_frame_rotation, workspace_centroid_wrt_gripper_frame_translation,
                                                    0,0,0,1;
  workspace_centroid_wrt_gripper_frame_transform_inverse << workspace_centroid_wrt_gripper_frame_rotation.transpose(), -workspace_centroid_wrt_gripper_frame_rotation.transpose()*workspace_centroid_wrt_gripper_frame_translation,
                                                            0, 0, 0, 1;
  
  
  Eigen::Matrix4f initial_gripper_orientation_transform;
  Eigen::Matrix4f gripper_transform_before_orientation_loop;
  scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color, "gripper cloud");
  
  scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
  
  
  Eigen::Vector4f gripper_workspace_centroid_point_in_gripper_frame_transformed;
  int orientation_samples = 20;
  double orientation_range = 2*M_PI;
  double orientation_step = orientation_range/orientation_samples;
  
  double special_ellipsoid_value;
  double sphere_value;
  bool gripper_collides_with_object_plane = false;
  bool gripper_collides_with_object = false;
  bool object_touches_gripper_support = false;
  
  Eigen::Vector3f best_gripper_translation;
  Eigen::Matrix3f best_gripper_rotation;
  Eigen::Matrix4f best_gripper_transform;
  Eigen::Matrix4f best_gripper_transform_inverse;
  
  int counter;
  
  //
  // declarations for allegro right hand
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_thumb_workspace                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_index_workspace                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_middle_workspace               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_pinky_workspace                (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_thumb_workspace_best           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_index_workspace_best           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_middle_workspace_best          (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_pinky_workspace_best           (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    thumb_workspace_active_spheres_offset           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    thumb_workspace_active_spheres_parameter        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    thumb_workspace_active_spheres_offset_best      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    thumb_workspace_active_spheres_parameter_best   (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    index_workspace_active_spheres_offset           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    index_workspace_active_spheres_parameter        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    index_workspace_active_spheres_offset_best      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    index_workspace_active_spheres_parameter_best   (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    middle_workspace_active_spheres_offset          (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    middle_workspace_active_spheres_parameter       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    middle_workspace_active_spheres_offset_best     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    middle_workspace_active_spheres_parameter_best  (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    pinky_workspace_active_spheres_offset           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    pinky_workspace_active_spheres_parameter        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    pinky_workspace_active_spheres_offset_best      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    pinky_workspace_active_spheres_parameter_best   (new pcl::PointCloud<pcl::PointXYZ>);
  
  //
  // declarations for panda gripper
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_right_finger_workspace              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_left_finger_workspace               (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_right_finger_workspace_best         (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points_in_left_finger_workspace_best          (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    right_finger_workspace_active_spheres_offset         (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    right_finger_workspace_active_spheres_parameter      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    right_finger_workspace_active_spheres_offset_best    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    right_finger_workspace_active_spheres_parameter_best (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    left_finger_workspace_active_spheres_offset          (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    left_finger_workspace_active_spheres_parameter       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    left_finger_workspace_active_spheres_offset_best     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr    left_finger_workspace_active_spheres_parameter_best  (new pcl::PointCloud<pcl::PointXYZ>);
  
  
  //*scene_cloud_xyzrgb += *object_plane_cloud_in_object_plane_frame_xyzrgb;
  
  double time_elapsed_average_1 = 0.0;
  double time_elapsed_average_2 = 0.0;
  double time_elapsed_average_3 = 0.0;
  double time_elapsed_average_4 = 0.0;
  
  int gripper_collide_with_table   = 0;
  int gripper_collide_with_object  = 0;
  int gripper_contacts_with_object = 0;
  
  double distance_between_gripper_support_and_object_centroid;
  double distance_between_gripper_support_and_object_centroid_best = 1000.0;
  
  Eigen::Vector4f object_centroid_point_transformed;
  
  // GRASPING CODE
  // iterate through all points in the "object sampling cloud"
  for(unsigned int i=0; i<object_sampling_in_gripper_frame_xyzrgb->points.size(); i++){
  //for(unsigned int i=0; i<1; i++){
    // STEP
    // translate the gripper workspace centroid
    // place gripper workspace centroid at the current "object sampled point"
    dummy_translation << object_sampling_in_gripper_frame_xyzrgb->points[i].x,
                         object_sampling_in_gripper_frame_xyzrgb->points[i].y,
                         object_sampling_in_gripper_frame_xyzrgb->points[i].z;
    dummy_transform << Eigen::Matrix3f::Identity(), dummy_translation,
                       0,0,0,1;
    gripper_transform = dummy_transform;
    
    // rotate the gripper workspace centroid to be alligned with the object orientation
    // then rotate it to be perpendicular to the object major axis
    dummy_translation << 0,0,0;
    if(gripper_model == "allegro_right_hand"){
      initial_gripper_orientation_transform << object_rotation*Rotx_float(M_PI/2), dummy_translation,
                                               0,0,0,1;
    }
    else if(gripper_model == "franka_gripper"){
      initial_gripper_orientation_transform << object_rotation*Roty_float(M_PI/2), dummy_translation,
                                               0,0,0,1;
    }
    gripper_transform *= initial_gripper_orientation_transform;
    
    
    
    // STEP
    // iterate through the range of possible orientations about object's major axis (z-axis)
    gripper_transform_before_orientation_loop = gripper_transform;
    dummy_translation << 0,0,0;
    for(unsigned int j=0; j<orientation_samples; j++){
      if(gripper_model == "allegro_right_hand"){
        dummy_rotation = Roty_float(M_PI/4+j*orientation_range/orientation_samples);
      }
      else if(gripper_model == "franka_gripper"){
        dummy_rotation = -Rotx_float(M_PI/4+j*orientation_range/orientation_samples);
      }
      dummy_transform << dummy_rotation, dummy_translation,
                         0,0,0,1;
      gripper_transform = gripper_transform_before_orientation_loop*dummy_transform;
      
      // all the past transforms were performed on the workspace centroid frame
      // transform it back to the gripper frame
      gripper_transform *= workspace_centroid_wrt_gripper_frame_transform_inverse;
      
      gripper_rotation << gripper_transform(0,0), gripper_transform(0,1), gripper_transform(0,2),
                          gripper_transform(1,0), gripper_transform(1,1), gripper_transform(1,2),
                          gripper_transform(2,0), gripper_transform(2,1), gripper_transform(2,2);
      gripper_translation << gripper_transform(0,3), gripper_transform(1,3), gripper_transform(2,3);
      // transform the gripper to the pose candidate
      pcl::transformPointCloud(*gripper_cloud_in_gripper_frame_xyzrgb, *gripper_cloud_transformed_in_gripper_frame_xyzrgb, gripper_transform);
      
      
      
      
      begin3 = clock();
      // STEP
      // check if the current gripper pose collides with object_plane
      // transform the gripper point cloud to object_plane frame to be able to use the special ellipsoid
      pcl::transformPointCloud(*gripper_cloud_transformed_in_gripper_frame_xyzrgb, *gripper_cloud_transformed_in_object_plane_frame_xyzrgb, inverse_object_plane_transform);
      gripper_collides_with_object_plane = false;
      for(unsigned int k=0; k<gripper_cloud_transformed_in_object_plane_frame_xyzrgb->size(); k++){
        special_ellipsoid_value =  pow(gripper_cloud_transformed_in_object_plane_frame_xyzrgb->points[k].x - object_plane_offset_x, 10)/pow(object_plane_x, 10) 
                                 + pow(gripper_cloud_transformed_in_object_plane_frame_xyzrgb->points[k].y - object_plane_offset_y, 10)/pow(object_plane_y, 10) 
                                 + pow(gripper_cloud_transformed_in_object_plane_frame_xyzrgb->points[k].z - object_plane_offset_z, 2) /pow(object_plane_z, 2);
        if( special_ellipsoid_value < 1 ){
          gripper_collides_with_object_plane = true;
          break;
        }
      }
      //std::cout << "gripper collides with object plane = " << gripper_collides_with_object_plane << std::endl;
      end = clock();
	    time_spent = (double)( end - begin3 )/ CLOCKS_PER_SEC;
	    time_elapsed_average_1 += time_spent;
	    
      
      
      
      begin4 = clock();
      // STEP
      // if the gripper doesn't collide with the table (object plane)
      // check that the gripper doesn't collide with object
      // this is done by checking gripper special ellipsoids
      // first apply the inverse gripper transform on the object cloud
      // to make it light : we use downsampled object cloud
      inverse_gripper_transform << gripper_rotation.transpose(), -gripper_rotation.transpose()*gripper_translation,  // from khalil's book page 21
                                   0, 0, 0, 1;
      //pcl::transformPointCloud(*object_cloud_in_gripper_frame_xyzrgb, *object_cloud_transformed_in_gripper_frame_xyzrgb, inverse_gripper_transform);            // using whole object cloud
      pcl::transformPointCloud(*object_cloud_downsampled_in_gripper_frame_xyzrgb, *object_cloud_transformed_in_gripper_frame_xyzrgb, inverse_gripper_transform);  // using downsampled object cloud
      gripper_collides_with_object = false;
      if(!gripper_collides_with_object_plane){
        for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
          // for each point check it is not inside any of the gripper special ellipsoids
          for(unsigned int l=0; l<gripper_x.size(); l++){
            special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - gripper_offset_x[l], 10)/pow(gripper_x[l], 10) 
                                     + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - gripper_offset_y[l], 10)/pow(gripper_y[l], 10) 
                                     + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - gripper_offset_z[l], 2) /pow(gripper_z[l], 2);
            if( special_ellipsoid_value < 1 ){
              gripper_collides_with_object = true;
              gripper_collide_with_object++;
              break;
            }
          }
          if(gripper_collides_with_object)
            break;
        }
        //std::cout << "gripper collides with object = " << gripper_collides_with_object << std::endl;
      }
      else
        gripper_collide_with_table++;
      end = clock();
	    time_spent = (double)( end - begin4 )/ CLOCKS_PER_SEC;
	    time_elapsed_average_2 += time_spent;
	    
      
      
      
      //object_cloud_downsampled_inside_workspace_in_gripper_frame_xyzrgb
      // STEP
      // if gripper doesn't collide with object
      // if gripper doesn't collide with object plane (table)
      // 
      if(gripper_model == "allegro_right_hand"){
        if(!gripper_collides_with_object_plane and !gripper_collides_with_object){
          
          
          // Evaluation Metric#1 : number of object points && number of workspace spheres
          // thumb
          object_points_in_thumb_workspace->clear();
          thumb_workspace_active_spheres_offset->clear();
          thumb_workspace_active_spheres_parameter->clear();
          // for each thumb workspace sphere
          for(unsigned int l=0; l<thumb_workspace_convex_offset.size(); l++){
            counter = 0;
            // for each point in object downsampled cloud
            for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
              sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - thumb_workspace_convex_offset.points[l].x, 2)
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - thumb_workspace_convex_offset.points[l].y, 2) 
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - thumb_workspace_convex_offset.points[l].z, 2) )/pow(thumb_workspace_convex_parameter.points[l].x, 2);
              // add all object points inside this workspace sphere
              if( sphere_value < 1 ){
                object_points_in_thumb_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
                counter++;
              }
            } // end of iteration through all object points
            if(counter > 0){
              //add this sphere to set of active spheres
              thumb_workspace_active_spheres_offset->points.push_back( thumb_workspace_convex_offset.points[l] );
              thumb_workspace_active_spheres_parameter->points.push_back( thumb_workspace_convex_parameter.points[l] );
            }    
          } // end of iteration through all thumb workspace spheres
          
          // index
          object_points_in_index_workspace->clear();
          index_workspace_active_spheres_offset->clear();
          index_workspace_active_spheres_parameter->clear();
          // for each index workspace sphere
          for(unsigned int l=0; l<index_workspace_convex_offset.size(); l++){
            counter = 0;
            // for each point in object downsampled cloud
            for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
              sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - index_workspace_convex_offset.points[l].x, 2)
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - index_workspace_convex_offset.points[l].y, 2) 
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - index_workspace_convex_offset.points[l].z, 2) )/pow(index_workspace_convex_parameter.points[l].x, 2);
              // add all object points inside this workspace sphere
              if( sphere_value < 1 ){
                object_points_in_index_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
                counter++;
              }
            } // end of iteration through all object points
            if(counter > 0){
              //add this sphere to set of active spheres
              index_workspace_active_spheres_offset->points.push_back( index_workspace_convex_offset.points[l] );
              index_workspace_active_spheres_parameter->points.push_back( index_workspace_convex_parameter.points[l] );
            }    
          } // end of iteration through all index workspace spheres
          
          
          // middle
          object_points_in_middle_workspace->clear();
          middle_workspace_active_spheres_offset->clear();
          middle_workspace_active_spheres_parameter->clear();
          // for each middle workspace sphere
          for(unsigned int l=0; l<middle_workspace_convex_offset.size(); l++){
            counter = 0;
            // for each point in object downsampled cloud
            for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
              sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - middle_workspace_convex_offset.points[l].x, 2)
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - middle_workspace_convex_offset.points[l].y, 2) 
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - middle_workspace_convex_offset.points[l].z, 2) )/pow(middle_workspace_convex_parameter.points[l].x, 2);
              // add all object points inside this workspace sphere
              if( sphere_value < 1 ){
                object_points_in_middle_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
                counter++;
              }
            } // end of iteration through all object points
            if(counter > 0){
              //add this sphere to set of active spheres
              middle_workspace_active_spheres_offset->points.push_back( middle_workspace_convex_offset.points[l] );
              middle_workspace_active_spheres_parameter->points.push_back( middle_workspace_convex_parameter.points[l] );
            }    
          } // end of iteration through all middle workspace spheres
          
          
          // pinky
          object_points_in_pinky_workspace->clear();
          pinky_workspace_active_spheres_offset->clear();
          pinky_workspace_active_spheres_parameter->clear();
          // for each pinky workspace sphere
          for(unsigned int l=0; l<pinky_workspace_convex_offset.size(); l++){
            counter = 0;
            // for each point in object downsampled cloud
            for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
              sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - pinky_workspace_convex_offset.points[l].x, 2)
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - pinky_workspace_convex_offset.points[l].y, 2) 
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - pinky_workspace_convex_offset.points[l].z, 2) )/pow(pinky_workspace_convex_parameter.points[l].x, 2);
              // add all object points inside this workspace sphere
              if( sphere_value < 1 ){
                object_points_in_pinky_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
                counter++;
              }
            } // end of iteration through all object points
            if(counter > 0){
              //add this sphere to set of active spheres
              pinky_workspace_active_spheres_offset->points.push_back( pinky_workspace_convex_offset.points[l] );
              pinky_workspace_active_spheres_parameter->points.push_back( pinky_workspace_convex_parameter.points[l] );
            }    
          } // end of iteration through all pinky workspace spheres
          
          
          
          
          
          // Evaluation Metric#2 : distance to object centroid
          
          
          
          
          /*
          // Evaluation Metric#3 : number of object points lie in gripper support (special ellipsoid)
          begin5 = clock();
          // check if any point on object lies in the gripper support region (special ellipsoid) at this gripper pose
          object_touches_gripper_support = false;
          for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
            special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - gripper_support_offset_x, 10)/pow(gripper_support_x, 10) 
                                     + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - gripper_support_offset_y, 10)/pow(gripper_support_y, 10) 
                                     + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - gripper_support_offset_z, 2) /pow(gripper_support_z, 2);
            if( special_ellipsoid_value < 1 ){
              object_touches_gripper_support = true;
              gripper_contacts_with_object++;
              break;
            }
          }
          //std::cout << "gripper collides with object plane = " << gripper_collides_with_object_plane << std::endl;
          end = clock();
	        time_spent = (double)( end - begin5 )/ CLOCKS_PER_SEC;
	        time_elapsed_average_3 += time_spent;
          */
          
          // Evaluation Metric#3 : how close the object centroid to gripper support offset (special ellipsoid)
          begin5 = clock();
          object_centroid_point_transformed << object_centroid_point.x, object_centroid_point.y, object_centroid_point.z, 1;
          object_centroid_point_transformed = inverse_gripper_transform*object_centroid_point_transformed;
          distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_x-object_centroid_point_transformed(0)),2)+
                                                                      pow(fabs(gripper_support_offset_y-object_centroid_point_transformed(1)),2)+
                                                                      pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
          
          //distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_y-object_centroid_point_transformed(1)),2)+
          //                                                            pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
          
          //distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
          end = clock();
	        time_spent = (double)( end - begin5 )/ CLOCKS_PER_SEC;
	        time_elapsed_average_3 += time_spent;
          
          
          
          
          
          
          // Evaluation
          // select the best gripper pose
          // for the moment we select the pose having the most object points intersecting with gripper workspace
          //if( object_touches_gripper_support and ( (object_points_in_thumb_workspace->size() + object_points_in_index_workspace->size() + object_points_in_middle_workspace->size() + object_points_in_pinky_workspace->size()) > 
          //    (object_points_in_thumb_workspace_best->size() + object_points_in_index_workspace_best->size() + object_points_in_middle_workspace_best->size() + object_points_in_pinky_workspace_best->size() ) ) ){
          
          // this condition ensures all fingers must have solution to touch object
          if( object_points_in_thumb_workspace->size()!=0 and object_points_in_index_workspace->size()!=0 and object_points_in_middle_workspace->size()!=0 and object_points_in_pinky_workspace->size()!=0 ){
            // this condition maximizes the number of object points inside the gripper workspace
            //if( (object_points_in_thumb_workspace->size() + object_points_in_index_workspace->size() + object_points_in_middle_workspace->size() + object_points_in_pinky_workspace->size()) > 
            //  (object_points_in_thumb_workspace_best->size() + object_points_in_index_workspace_best->size() + object_points_in_middle_workspace_best->size() + object_points_in_pinky_workspace_best->size() ) ){
            
            // this condition minimizes distance between object centroid and gripper support region
            if( distance_between_gripper_support_and_object_centroid < distance_between_gripper_support_and_object_centroid_best ){
              distance_between_gripper_support_and_object_centroid_best = distance_between_gripper_support_and_object_centroid;
              
              // saving best object points
              *object_points_in_thumb_workspace_best  = *object_points_in_thumb_workspace;
              *object_points_in_index_workspace_best  = *object_points_in_index_workspace;
              *object_points_in_middle_workspace_best = *object_points_in_middle_workspace;
              *object_points_in_pinky_workspace_best  = *object_points_in_pinky_workspace;
              
              // saving the best workspace spheres
              *thumb_workspace_active_spheres_offset_best     = *thumb_workspace_active_spheres_offset;
              *thumb_workspace_active_spheres_parameter_best  = *thumb_workspace_active_spheres_parameter;
              *index_workspace_active_spheres_offset_best     = *index_workspace_active_spheres_offset;
              *index_workspace_active_spheres_parameter_best  = *index_workspace_active_spheres_parameter;
              *middle_workspace_active_spheres_offset_best    = *middle_workspace_active_spheres_offset;
              *middle_workspace_active_spheres_parameter_best = *middle_workspace_active_spheres_parameter;
              *pinky_workspace_active_spheres_offset_best     = *pinky_workspace_active_spheres_offset;
              *pinky_workspace_active_spheres_parameter_best  = *pinky_workspace_active_spheres_parameter;
              
              //std::cout<<"metric#1 : "<<(object_points_in_thumb_workspace_best->size() + object_points_in_index_workspace_best->size() + object_points_in_middle_workspace_best->size() + object_points_in_pinky_workspace_best->size() )<<std::endl;
              std::cout<<"metric#1 : "<<distance_between_gripper_support_and_object_centroid_best<<std::endl;
              best_gripper_transform = gripper_transform;
            }
          }
        
        
        }
      }
      else if(gripper_model == "franka_gripper"){
        if(!gripper_collides_with_object_plane and !gripper_collides_with_object){
          
          // Evaluation Metric#1 : number of object points && number of workspace spheres
          // right finger
          object_points_in_right_finger_workspace->clear();
          right_finger_workspace_active_spheres_offset->clear();
          right_finger_workspace_active_spheres_parameter->clear();
          // for each right_finger workspace sphere
          for(unsigned int l=0; l<right_finger_workspace_convex_offset.size(); l++){
            counter = 0;
            // for each point in object downsampled cloud
            for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
              sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - right_finger_workspace_convex_offset.points[l].x, 2)
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - right_finger_workspace_convex_offset.points[l].y, 2) 
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - right_finger_workspace_convex_offset.points[l].z, 2) )/pow(right_finger_workspace_convex_parameter.points[l].x, 2);
              // add all object points inside this workspace sphere
              if( sphere_value < 1 ){
                object_points_in_right_finger_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
                counter++;
              }
            } // end of iteration through all object points
            if(counter > 0){
              //add this sphere to set of active spheres
              right_finger_workspace_active_spheres_offset->points.push_back( right_finger_workspace_convex_offset.points[l] );
              right_finger_workspace_active_spheres_parameter->points.push_back( right_finger_workspace_convex_parameter.points[l] );
            }    
          } // end of iteration through all right_finger workspace spheres
          
          // left_finger
          object_points_in_left_finger_workspace->clear();
          left_finger_workspace_active_spheres_offset->clear();
          left_finger_workspace_active_spheres_parameter->clear();
          // for each left_finger workspace sphere
          for(unsigned int l=0; l<left_finger_workspace_convex_offset.size(); l++){
            counter = 0;
            // for each point in object downsampled cloud
            for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
              sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - left_finger_workspace_convex_offset.points[l].x, 2)
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - left_finger_workspace_convex_offset.points[l].y, 2) 
                             + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - left_finger_workspace_convex_offset.points[l].z, 2) )/pow(left_finger_workspace_convex_parameter.points[l].x, 2);
              // add all object points inside this workspace sphere
              if( sphere_value < 1 ){
                object_points_in_left_finger_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
                counter++;
              }
            } // end of iteration through all object points
            if(counter > 0){
              //add this sphere to set of active spheres
              left_finger_workspace_active_spheres_offset->points.push_back( left_finger_workspace_convex_offset.points[l] );
              left_finger_workspace_active_spheres_parameter->points.push_back( left_finger_workspace_convex_parameter.points[l] );
            }    
          } // end of iteration through all left_finger workspace spheres
          
          
        
        
        
          // Evaluation Metric#2 : distance to object centroid
          
          
          
          
          
          // Evaluation Metric#3 : how close the object centroid to gripper support offset (special ellipsoid)
          begin5 = clock();
          object_centroid_point_transformed << object_centroid_point.x, object_centroid_point.y, object_centroid_point.z, 1;
          object_centroid_point_transformed = inverse_gripper_transform*object_centroid_point_transformed;
          //distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_x-object_centroid_point_transformed(0)),2)+
          //                                                            pow(fabs(gripper_support_offset_y-object_centroid_point_transformed(1)),2)+
          //                                                            pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
          distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_y-object_centroid_point_transformed(1)),2)+
                                                                      pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
          end = clock();
	        time_spent = (double)( end - begin5 )/ CLOCKS_PER_SEC;
	        time_elapsed_average_3 += time_spent;
          
          
          
          
          // this condition ensures all fingers are in contact with object
          if( object_points_in_right_finger_workspace->size()!=0 and object_points_in_left_finger_workspace->size()!=0 ){
            // this condition maximizes the number of object points inside the gripper workspace
            //if( (object_points_in_thumb_workspace->size() + object_points_in_index_workspace->size() + object_points_in_middle_workspace->size() + object_points_in_pinky_workspace->size()) > 
            //  (object_points_in_thumb_workspace_best->size() + object_points_in_index_workspace_best->size() + object_points_in_middle_workspace_best->size() + object_points_in_pinky_workspace_best->size() ) ){
            
            // this condition minimizes distance between object centroid and gripper support region
            if( distance_between_gripper_support_and_object_centroid < distance_between_gripper_support_and_object_centroid_best ){
              distance_between_gripper_support_and_object_centroid_best = distance_between_gripper_support_and_object_centroid;
              
              // saving best object points
            *object_points_in_right_finger_workspace_best = *object_points_in_right_finger_workspace;
            *object_points_in_left_finger_workspace_best  = *object_points_in_left_finger_workspace;
            
            // saving the best workspace spheres
            *right_finger_workspace_active_spheres_offset_best     = *right_finger_workspace_active_spheres_offset;
            *right_finger_workspace_active_spheres_parameter_best  = *right_finger_workspace_active_spheres_parameter;
            *left_finger_workspace_active_spheres_offset_best     = *left_finger_workspace_active_spheres_offset;
            *left_finger_workspace_active_spheres_parameter_best  = *left_finger_workspace_active_spheres_parameter;
            
            //std::cout<<"metric#1 : "<<object_points_in_right_finger_workspace_best->size() << ", " << object_points_in_left_finger_workspace_best->size()<<std::endl;
            std::cout<<"metric#1 : "<<distance_between_gripper_support_and_object_centroid_best<<std::endl;
            best_gripper_transform = gripper_transform;
            }
          }
          
          
          
          
        
        
        }
      }
      
      
      //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
      //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color, "gripper cloud");
      //scene_cloud_viewer->updatePointCloud(scene_cloud_xyzrgb, scene_cloud_rgb, "scene cloud viewer");
      //scene_cloud_viewer->spinOnce();
      
      
    }
    
  }
  
  end = clock();
	time_spent = (double)( end - begin2 )/ CLOCKS_PER_SEC;
	std::cout << "time spent in checking gripper collision with table        = " << time_elapsed_average_1 << std::endl;
	std::cout << "time spent in checking gripper collision with object       = " << time_elapsed_average_2 << std::endl;
  std::cout << "time spent in checking object contact with gripper support = " << time_elapsed_average_3 << std::endl;
  std::cout << "time spent in scanning for best gripper pose               = " << time_spent << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_gripper_frame_xyzrgb->points.size() << " iterations, gripper collide with table  : " << gripper_collide_with_table << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_gripper_frame_xyzrgb->points.size() << " iterations, gripper collide with object : " << gripper_collide_with_object << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_gripper_frame_xyzrgb->points.size() << " iterations, gripper contacts with object: " << gripper_contacts_with_object << " times." << std::endl;
  
  
  
  // CASE#1 : ALLEGRO RIGHT HAND
  if(gripper_model == "allegro_right_hand"){
  // draw the workspace spheres of the best gripper pose
  // thumb
  scene_cloud_viewer->addPointCloud(thumb_workspace_spheres_best, red_color, "thumb workspace spheres");
  for(unsigned int j=0; j<thumb_workspace_active_spheres_offset_best->size(); j++){
    convex_shape_offset.x = thumb_workspace_active_spheres_offset_best->points[j].x;
    convex_shape_offset.y = thumb_workspace_active_spheres_offset_best->points[j].y;
    convex_shape_offset.z = thumb_workspace_active_spheres_offset_best->points[j].z;
    convex_shape_parameter.x = thumb_workspace_active_spheres_parameter_best->points[j].x;
    convex_shape_parameter.y = thumb_workspace_active_spheres_parameter_best->points[j].y;
    convex_shape_parameter.z = thumb_workspace_active_spheres_parameter_best->points[j].z;
    
    point_cloud_samples = 40;
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        thumb_workspace_spheres_best->points.push_back(point_xyzrgb);
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        thumb_workspace_spheres_best->points.push_back(point_xyzrgb);
      }
    }
  }
  pcl::transformPointCloud(*thumb_workspace_spheres_best, *thumb_workspace_spheres_best, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(thumb_workspace_spheres_best, red_color, "thumb workspace spheres");
  
  // index
  scene_cloud_viewer->addPointCloud(index_workspace_spheres_best, green_color, "index workspace spheres");
  for(unsigned int j=0; j<index_workspace_active_spheres_offset_best->size(); j++){
    convex_shape_offset.x = index_workspace_active_spheres_offset_best->points[j].x;
    convex_shape_offset.y = index_workspace_active_spheres_offset_best->points[j].y;
    convex_shape_offset.z = index_workspace_active_spheres_offset_best->points[j].z;
    convex_shape_parameter.x = index_workspace_active_spheres_parameter_best->points[j].x;
    convex_shape_parameter.y = index_workspace_active_spheres_parameter_best->points[j].y;
    convex_shape_parameter.z = index_workspace_active_spheres_parameter_best->points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        index_workspace_spheres_best->points.push_back(point_xyzrgb);
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        index_workspace_spheres_best->points.push_back(point_xyzrgb);
      }
    }
  }
  pcl::transformPointCloud(*index_workspace_spheres_best, *index_workspace_spheres_best, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(index_workspace_spheres_best, green_color, "index workspace spheres");
  
  // middle
  scene_cloud_viewer->addPointCloud(middle_workspace_spheres_best, blue_color, "middle workspace spheres");
  for(unsigned int j=0; j<middle_workspace_active_spheres_offset_best->size(); j++){
    convex_shape_offset.x = middle_workspace_active_spheres_offset_best->points[j].x;
    convex_shape_offset.y = middle_workspace_active_spheres_offset_best->points[j].y;
    convex_shape_offset.z = middle_workspace_active_spheres_offset_best->points[j].z;
    convex_shape_parameter.x = middle_workspace_active_spheres_parameter_best->points[j].x;
    convex_shape_parameter.y = middle_workspace_active_spheres_parameter_best->points[j].y;
    convex_shape_parameter.z = middle_workspace_active_spheres_parameter_best->points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        middle_workspace_spheres_best->points.push_back(point_xyzrgb);
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        middle_workspace_spheres_best->points.push_back(point_xyzrgb);
      }
    }
  }
  pcl::transformPointCloud(*middle_workspace_spheres_best, *middle_workspace_spheres_best, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(middle_workspace_spheres_best, blue_color, "middle workspace spheres");
  
  // pinky
  scene_cloud_viewer->addPointCloud(pinky_workspace_spheres_best, grey_color, "pinky workspace spheres");
  for(unsigned int j=0; j<pinky_workspace_active_spheres_offset_best->size(); j++){
    convex_shape_offset.x = pinky_workspace_active_spheres_offset_best->points[j].x;
    convex_shape_offset.y = pinky_workspace_active_spheres_offset_best->points[j].y;
    convex_shape_offset.z = pinky_workspace_active_spheres_offset_best->points[j].z;
    convex_shape_parameter.x = pinky_workspace_active_spheres_parameter_best->points[j].x;
    convex_shape_parameter.y = pinky_workspace_active_spheres_parameter_best->points[j].y;
    convex_shape_parameter.z = pinky_workspace_active_spheres_parameter_best->points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        pinky_workspace_spheres_best->points.push_back(point_xyzrgb);
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        pinky_workspace_spheres_best->points.push_back(point_xyzrgb);
      }
    }
  }
  pcl::transformPointCloud(*pinky_workspace_spheres_best, *pinky_workspace_spheres_best, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(pinky_workspace_spheres_best, grey_color, "pinky workspace spheres");
  }
  else if(gripper_model == "franka_gripper"){
  // draw the workspace spheres of the best gripper pose
  // right_finger
  scene_cloud_viewer->addPointCloud(right_finger_workspace_spheres_best, red_color_again, "right_finger workspace spheres");
  for(unsigned int j=0; j<right_finger_workspace_active_spheres_offset_best->size(); j++){
    convex_shape_offset.x = right_finger_workspace_active_spheres_offset_best->points[j].x;
    convex_shape_offset.y = right_finger_workspace_active_spheres_offset_best->points[j].y;
    convex_shape_offset.z = right_finger_workspace_active_spheres_offset_best->points[j].z;
    convex_shape_parameter.x = right_finger_workspace_active_spheres_parameter_best->points[j].x;
    convex_shape_parameter.y = right_finger_workspace_active_spheres_parameter_best->points[j].y;
    convex_shape_parameter.z = right_finger_workspace_active_spheres_parameter_best->points[j].z;
    
    point_cloud_samples = 40;
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        right_finger_workspace_spheres_best->points.push_back(point_xyzrgb);
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        right_finger_workspace_spheres_best->points.push_back(point_xyzrgb);
      }
    }
  }
  pcl::transformPointCloud(*right_finger_workspace_spheres_best, *right_finger_workspace_spheres_best, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(right_finger_workspace_spheres_best, red_color_again, "right_finger workspace spheres");
  
  // left_finger
  scene_cloud_viewer->addPointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
  for(unsigned int j=0; j<left_finger_workspace_active_spheres_offset_best->size(); j++){
    convex_shape_offset.x = left_finger_workspace_active_spheres_offset_best->points[j].x;
    convex_shape_offset.y = left_finger_workspace_active_spheres_offset_best->points[j].y;
    convex_shape_offset.z = left_finger_workspace_active_spheres_offset_best->points[j].z;
    convex_shape_parameter.x = left_finger_workspace_active_spheres_parameter_best->points[j].x;
    convex_shape_parameter.y = left_finger_workspace_active_spheres_parameter_best->points[j].y;
    convex_shape_parameter.z = left_finger_workspace_active_spheres_parameter_best->points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        left_finger_workspace_spheres_best->points.push_back(point_xyzrgb);
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        //point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        left_finger_workspace_spheres_best->points.push_back(point_xyzrgb);
      }
    }
  }
  pcl::transformPointCloud(*left_finger_workspace_spheres_best, *left_finger_workspace_spheres_best, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
  }
  
  
  
  
  
  
  
  
  pcl::transformPointCloud(*gripper_cloud_in_gripper_frame_xyzrgb, *gripper_cloud_transformed_in_gripper_frame_xyzrgb, best_gripper_transform);
  
  scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color, "gripper cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gripper cloud");
  
  scene_cloud_viewer->updatePointCloud(object_plane_transformed_cloud_xyzrgb, brown_color, "table cloud");
  
  //scene_cloud_viewer->updatePointCloud(object_cloud_in_gripper_frame_xyzrgb, magenta_color, "object cloud");
  scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_gripper_frame_xyzrgb, magenta_color, "object cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object cloud");
  
  scene_cloud_viewer->updatePointCloud(scene_cloud_xyzrgb, scene_cloud_rgb, "scene cloud viewer");
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent = " << time_spent << std::endl;
  
  
  while ( !scene_cloud_viewer->wasStopped() ){
    scene_cloud_viewer->spinOnce();
  }
  return (0);
}

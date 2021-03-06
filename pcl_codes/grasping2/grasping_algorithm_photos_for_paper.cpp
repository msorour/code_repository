/*
run this program using:
reset && cmake .. && make && ./grasping_algorithm allegro_right_hand_model_cloud_plus_camera.pcd 0.01 thermos.pcd 0.007 table.pcd
reset && cmake .. && make && ./grasping_algorithm franka_gripper_model_cloud_plus_camera.pcd 0.01 thermos.pcd 0.007 table.pcd

reset && cmake .. && make && ./grasping_algorithm ../gripper_pcd_model/franka_gripper_model_cloud_plus_camera.pcd 0.01 ../../../real_experiments/build/pcd_files/mug.pcd 0.005 ../../../real_experiments/build/pcd_files/mug_table.pcd
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
//#include "/home/franka3/grasping_msorour/code_repository/QuadProgpp/src/QuadProg++.hh"
//#include "/home/work/software/QuadProgpp-master/src/QuadProg++.hh"
#include "QuadProg++.hh"


int main (int argc, char** argv){
  clock_t begin, begin2, begin3, begin4, begin5, end;
	begin = clock();
	double time_spent;
	
  std::string gripper_file_name      = argv[1];
  double gripper_leaf_size           = std::stof( argv[2] );
  std::string object_file_name       = argv[3];
  double object_leaf_size            = std::stof( argv[4] );
  std::string object_plane_file_name = argv[5];
  
  std::string gripper_model;
	if(gripper_file_name.find("allegro_right_hand" )!=std::string::npos){gripper_model = "allegro_right_hand";}
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){gripper_model = "franka_gripper";}
  
  // point clouds declarations
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_in_camera_depth_optical_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_cloud_in_camera_depth_optical_frame_xyzrgb                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_in_arm_hand_frame_xyz                                   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_cloud_transformed_in_gripper_frame_xyzrgb                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_cloud_transformed_in_arm_hand_frame_xyzrgb                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_sampling_in_object_frame_xyzrgb                               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_sampling_in_gripper_frame_xyzrgb                              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_sampling_in_gripper_centroid_frame_xyzrgb                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_sampling_in_arm_hand_frame_xyzrgb                             (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_downsampled_in_camera_depth_optical_frame_xyz           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_downsampled_in_gripper_frame_xyz                        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_cloud_downsampled_in_gripper_frame_xyzrgb                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_downsampled_in_arm_hand_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_cloud_downsampled_in_arm_hand_frame_xyzrgb                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_in_camera_depth_optical_frame_xyz                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_in_camera_depth_optical_frame_xyzrgb              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_gripper_frame_xyz                  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_gripper_frame_xyzrgb               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_arm_hand_frame_xyz                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_object_plane_frame_xyzrgb          (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_object_plane_frame_xyz             (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_in_gripper_frame_xyz                                   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_in_gripper_frame_xyzrgb                                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_transformed_in_gripper_frame_xyzrgb                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_transformed_in_arm_hand_frame_xyzrgb                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_transformed_in_object_plane_frame_xyzrgb               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_downsampled_in_gripper_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_downsampled_in_gripper_frame_xyzrgb                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_downsampled_in_gripper_centroid_frame_xyzrgb           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_cloud_transformed_in_gripper_centroid_frame_xyzrgb           (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene_cloud_xyzrgb                                                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  thumb_workspace_spheres_best                                         (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  index_workspace_spheres_best                                         (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  middle_workspace_spheres_best                                        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pinky_workspace_spheres_best                                         (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  right_finger_workspace_spheres_best                                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  left_finger_workspace_spheres_best                                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // declarations for allegro right hand
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_thumb_workspace                                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_index_workspace                                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_middle_workspace                                    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_pinky_workspace                                     (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_thumb_workspace_best                                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_index_workspace_best                                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_middle_workspace_best                               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_pinky_workspace_best                                (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_offset                                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_parameter                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_offset_best                           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_active_spheres_parameter_best                        (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_offset                                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_parameter                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_offset_best                           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_active_spheres_parameter_best                        (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_offset                               (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_parameter                            (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_offset_best                          (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_active_spheres_parameter_best                       (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_offset                                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_parameter                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_offset_best                           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_active_spheres_parameter_best                        (new pcl::PointCloud<pcl::PointXYZ>);
  
  // declarations for panda gripper
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_right_finger_workspace                              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_left_finger_workspace                               (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_right_finger_workspace_best                         (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_points_in_left_finger_workspace_best                          (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_offset                         (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_parameter                      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_offset_best                    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_active_spheres_parameter_best                 (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_offset                          (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_parameter                       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_offset_best                     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_active_spheres_parameter_best                  (new pcl::PointCloud<pcl::PointXYZ>);
  
  // for point cloud visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_cloud_viewer  (new pcl::visualization::PCLVisualizer ("scene cloud viewer"));
  scene_cloud_viewer->addCoordinateSystem(0.1);   // this is arm hand frame (the origin)
  
  scene_cloud_viewer->setBackgroundColor(255,255,255);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>scene_cloud_rgb(scene_cloud_xyzrgb);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> black_color       (gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, 0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> brown_color       (object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, 165, 42, 42);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> magenta_color     (object_cloud_downsampled_in_arm_hand_frame_xyzrgb, 255, 0, 255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color         (thumb_workspace_spheres_best, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color       (index_workspace_spheres_best, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color        (middle_workspace_spheres_best, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color        (pinky_workspace_spheres_best, 100, 100, 100);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color_again   (right_finger_workspace_spheres_best, 254, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color_again (left_finger_workspace_spheres_best, 0, 254, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color_again  (object_sampling_in_arm_hand_frame_xyzrgb, 0, 0, 254);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> black_color_again (gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, 1, 0, 0);
  
  /*
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,          "object cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,      "object cloud");
  
  scene_cloud_viewer->addPointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                "object sampling cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,      "object sampling cloud");
  
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,      "table cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,       "table cloud");
  
  scene_cloud_viewer->addPointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, black_color,           "gripper cloud in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,       "gripper cloud in arm hand frame");
  
  scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, black_color_again,     "gripper cloud transformed in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,       "gripper cloud transformed in arm hand frame");
  
  scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,       "gripper cloud in object plane frame");
  */
  
  
  // other declarations
  Eigen::Vector3f dummy_translation;  Eigen::Matrix3f dummy_rotation;  Eigen::Matrix4f dummy_transform;
  Eigen::Vector3f parameter_vector;  Eigen::Vector3f offset_vector;
  pcl::PointXYZRGB point_xyzrgb;  pcl::PointXYZ point_xyz;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  int point_cloud_samples = 250;
  double value_x, value_y, value_z;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Reading and downsampling point clouds
  // Read in the cloud data
  pcl::PCDReader reader;
  reader.read(gripper_file_name, *gripper_cloud_in_gripper_frame_xyz);
  reader.read(object_file_name, *object_cloud_in_camera_depth_optical_frame_xyz);
  reader.read(object_plane_file_name, *object_plane_cloud_in_camera_depth_optical_frame_xyz);
  
  // downsampling object point cloud
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(object_cloud_in_camera_depth_optical_frame_xyz);
  vg.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  vg.filter (*object_cloud_downsampled_in_camera_depth_optical_frame_xyz);
  std::cout << "Object point cloud before downsampling has:      " << object_cloud_in_camera_depth_optical_frame_xyz->points.size()              << " data points." << std::endl;
  std::cout << "Object point cloud after downsampling has :      " << object_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyz, *object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  copyPointCloud(*object_cloud_in_camera_depth_optical_frame_xyz, *object_cloud_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  // downsampling object plane point cloud
  vg.setInputCloud(object_plane_cloud_in_camera_depth_optical_frame_xyz);
  vg.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  vg.filter (*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz);
  std::cout << "Table point cloud before downsampling has:      " << object_plane_cloud_in_camera_depth_optical_frame_xyz->points.size()              << " data points." << std::endl;
  std::cout << "Table point cloud after downsampling has :      " << object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz, *object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  copyPointCloud(*object_plane_cloud_in_camera_depth_optical_frame_xyz, *object_plane_cloud_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  // downsampling gripper point cloud
  vg.setInputCloud(gripper_cloud_in_gripper_frame_xyz);
  vg.setLeafSize(gripper_leaf_size, gripper_leaf_size, gripper_leaf_size);
  vg.filter (*gripper_cloud_downsampled_in_gripper_frame_xyz);
  std::cout << "Gripper point cloud before downsampling has:     " << gripper_cloud_in_gripper_frame_xyz->points.size()              << " data points." << std::endl;
  std::cout << "Gripper point cloud after downsampling has :     " << gripper_cloud_downsampled_in_gripper_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*gripper_cloud_downsampled_in_gripper_frame_xyz, *gripper_cloud_downsampled_in_gripper_frame_xyzrgb);
  copyPointCloud(*gripper_cloud_in_gripper_frame_xyz, *gripper_cloud_in_gripper_frame_xyzrgb);
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Defining transformations
  // camera optical frame wrt arm hand frame
  Eigen::Vector3f camera_depth_optical_frame_wrt_arm_hand_frame_translation;   camera_depth_optical_frame_wrt_arm_hand_frame_translation << -0.067, 0.033, -0.027;
  Eigen::Matrix3f camera_depth_optical_frame_wrt_arm_hand_frame_rotation;
  Eigen::Matrix4f camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  Eigen::Matrix4f camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse;
  camera_depth_optical_frame_wrt_arm_hand_frame_transform << Rotz_float(-M_PI/2), camera_depth_optical_frame_wrt_arm_hand_frame_translation,
                                                             0,0,0,1;
  camera_depth_optical_frame_wrt_arm_hand_frame_rotation << camera_depth_optical_frame_wrt_arm_hand_frame_transform(0,0), camera_depth_optical_frame_wrt_arm_hand_frame_transform(0,1), camera_depth_optical_frame_wrt_arm_hand_frame_transform(0,2),
                                                            camera_depth_optical_frame_wrt_arm_hand_frame_transform(1,0), camera_depth_optical_frame_wrt_arm_hand_frame_transform(1,1), camera_depth_optical_frame_wrt_arm_hand_frame_transform(1,2),
                                                            camera_depth_optical_frame_wrt_arm_hand_frame_transform(2,0), camera_depth_optical_frame_wrt_arm_hand_frame_transform(2,1), camera_depth_optical_frame_wrt_arm_hand_frame_transform(2,2);
  
  camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse << camera_depth_optical_frame_wrt_arm_hand_frame_rotation.transpose(), -camera_depth_optical_frame_wrt_arm_hand_frame_rotation.transpose()*camera_depth_optical_frame_wrt_arm_hand_frame_translation,
                                                                     0,0,0,1;
  transform.matrix() = camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  //scene_cloud_viewer->addCoordinateSystem(0.07, transform, "camera depth optical frame", 0);
  
  // gripper frame wrt arm hand frame
  Eigen::Vector3f gripper_wrt_arm_hand_frame_translation;
  Eigen::Matrix3f gripper_wrt_arm_hand_frame_rotation;
  Eigen::Matrix4f gripper_wrt_arm_hand_frame_transform;
  Eigen::Matrix4f gripper_wrt_arm_hand_frame_inverse_transform;
  if(gripper_model == "allegro_right_hand"){
    gripper_wrt_arm_hand_frame_translation << -0.0204, 0, 0.02;
    gripper_wrt_arm_hand_frame_transform << Rotz_float(M_PI), gripper_wrt_arm_hand_frame_translation,
                                            0,0,0,1;}
  else if(gripper_model == "franka_gripper"){
    gripper_wrt_arm_hand_frame_translation << 0, 0, 0.005;
    gripper_wrt_arm_hand_frame_transform << Eigen::Matrix3f::Identity(), gripper_wrt_arm_hand_frame_translation,
                                            0,0,0,1;}
  transform.matrix() = gripper_wrt_arm_hand_frame_transform;
  //scene_cloud_viewer->addCoordinateSystem(0.1, transform, "gripper frame", 0);
  // gripper frame wrt arm hand frame [INVERSE]
  gripper_wrt_arm_hand_frame_rotation << gripper_wrt_arm_hand_frame_transform(0,0), gripper_wrt_arm_hand_frame_transform(0,1), gripper_wrt_arm_hand_frame_transform(0,2),
                                         gripper_wrt_arm_hand_frame_transform(1,0), gripper_wrt_arm_hand_frame_transform(1,1), gripper_wrt_arm_hand_frame_transform(1,2),
                                         gripper_wrt_arm_hand_frame_transform(2,0), gripper_wrt_arm_hand_frame_transform(2,1), gripper_wrt_arm_hand_frame_transform(2,2);
  gripper_wrt_arm_hand_frame_inverse_transform << gripper_wrt_arm_hand_frame_rotation.transpose(), -gripper_wrt_arm_hand_frame_rotation.transpose()*gripper_wrt_arm_hand_frame_translation,  // from khalil's book page 21
                                                  0, 0, 0, 1;
  
  // camera optical frame wrt gripper frame
  Eigen::Vector3f camera_depth_optical_frame_wrt_gripper_frame_translation;
  Eigen::Matrix3f camera_depth_optical_frame_wrt_gripper_frame_rotation;
  Eigen::Matrix4f camera_depth_optical_frame_wrt_gripper_frame_transform;
  camera_depth_optical_frame_wrt_gripper_frame_transform = gripper_wrt_arm_hand_frame_inverse_transform*camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Transforming point clouds
  // clouds in arm hand frame
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyz,    *object_cloud_downsampled_in_arm_hand_frame_xyz,    camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, *object_cloud_downsampled_in_arm_hand_frame_xyzrgb, camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  pcl::transformPointCloud(*object_cloud_in_camera_depth_optical_frame_xyz,                *object_cloud_in_arm_hand_frame_xyz,                camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz,    *object_plane_cloud_downsampled_in_arm_hand_frame_xyz,    camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, *object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_gripper_frame_xyzrgb, *gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, gripper_wrt_arm_hand_frame_transform);
  
  // clouds in gripper frame
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyz,    *object_cloud_downsampled_in_gripper_frame_xyz,       camera_depth_optical_frame_wrt_gripper_frame_transform);
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, *object_cloud_downsampled_in_gripper_frame_xyzrgb,    camera_depth_optical_frame_wrt_gripper_frame_transform);
  
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz,    *object_plane_cloud_downsampled_in_gripper_frame_xyz,    camera_depth_optical_frame_wrt_gripper_frame_transform);
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, *object_plane_cloud_downsampled_in_gripper_frame_xyzrgb, camera_depth_optical_frame_wrt_gripper_frame_transform);
  
  
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Object pose approximation and sampling
  // compute object transformation matrix
  Eigen::Matrix3f object_rotation_wrt_arm_hand_frame;
  Eigen::Vector3f object_translation_wrt_arm_hand_frame;
  Eigen::Matrix4f object_transform_wrt_arm_hand_frame;
  Eigen::Vector4f object_far_point_in_pos_direction_in_global_frame;
  Eigen::Vector4f object_far_point_in_neg_direction_in_global_frame;
  Eigen::Vector3f object_major_dimensions;
  step_4_object_pose_approximation( *object_cloud_downsampled_in_arm_hand_frame_xyz, object_transform_wrt_arm_hand_frame, object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions );
  transform.matrix() = object_transform_wrt_arm_hand_frame;
  //scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  object_rotation_wrt_arm_hand_frame    << object_transform_wrt_arm_hand_frame(0,0), object_transform_wrt_arm_hand_frame(0,1), object_transform_wrt_arm_hand_frame(0,2),
                                           object_transform_wrt_arm_hand_frame(1,0), object_transform_wrt_arm_hand_frame(1,1), object_transform_wrt_arm_hand_frame(1,2),
                                           object_transform_wrt_arm_hand_frame(2,0), object_transform_wrt_arm_hand_frame(2,1), object_transform_wrt_arm_hand_frame(2,2);
  object_translation_wrt_arm_hand_frame << object_transform_wrt_arm_hand_frame(0,3), object_transform_wrt_arm_hand_frame(1,3), object_transform_wrt_arm_hand_frame(2,3);
  
  // to remove
  // object cloud in its own frame
  Eigen::Matrix4f object_transform_wrt_arm_hand_frame_inverse;
  object_transform_wrt_arm_hand_frame_inverse << object_rotation_wrt_arm_hand_frame.transpose(), -object_rotation_wrt_arm_hand_frame.transpose()*object_translation_wrt_arm_hand_frame,  // from khalil's book page 21
                                                 0, 0, 0, 1;
  
  // sampling the object around its z-axis for scanning
  int object_sampling_in_x_axis = 5;   int object_sampling_in_y_axis = 5;   int object_sampling_in_z_axis = 5;
  object_sampling_in_object_frame_xyzrgb->clear();
  object_major_dimensions(0) = 0.95*object_major_dimensions(0);
  object_major_dimensions(1) = 0.95*object_major_dimensions(1);
  object_major_dimensions(2) = 0.95*object_major_dimensions(2);
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
  // to be removed !
  object_sampling_in_object_frame_xyzrgb->clear();
  
  if(gripper_model == "allegro_right_hand"){
    point_xyzrgb.x = -0.0284149;   point_xyzrgb.y = -0.0408458;   point_xyzrgb.z = -0.0144885;   point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;}
  else if(gripper_model == "franka_gripper"){
    point_xyzrgb.x = -0.0473582;   point_xyzrgb.y = -0.00583511;   point_xyzrgb.z = -0.0144885;   point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;}
  object_sampling_in_object_frame_xyzrgb->points.push_back( point_xyzrgb );
  pcl::transformPointCloud(*object_sampling_in_object_frame_xyzrgb, *object_sampling_in_arm_hand_frame_xyzrgb, object_transform_wrt_arm_hand_frame);
  
  // to remove
  // object centroid location in gripper frame
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid_in_gripper_frame;
  pcl::PointXYZ object_centroid_point_in_gripper_frame;
  for(unsigned int i=0;i<object_cloud_downsampled_in_gripper_frame_xyz->points.size();i++)
    object_centroid_in_gripper_frame.add( object_cloud_downsampled_in_gripper_frame_xyz->points[i] );
  object_centroid_in_gripper_frame.get(object_centroid_point_in_gripper_frame);
  
  // object centroid location in arm hand frame
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid_in_arm_hand_frame;
  pcl::PointXYZ object_centroid_point_in_arm_hand_frame;
  for(unsigned int i=0;i<object_cloud_downsampled_in_arm_hand_frame_xyz->points.size();i++)
    object_centroid_in_arm_hand_frame.add( object_cloud_downsampled_in_arm_hand_frame_xyz->points[i] );
  object_centroid_in_arm_hand_frame.get(object_centroid_point_in_arm_hand_frame);
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Object plane pose approximation
  // compute object plane transformation matrix
  Eigen::Matrix3f object_plane_rotation_wrt_arm_hand_frame;
  Eigen::Vector3f object_plane_translation_wrt_arm_hand_frame;
  Eigen::Matrix4f object_plane_transform_wrt_arm_hand_frame;
  Eigen::Vector4f object_plane_far_point_in_pos_direction_in_global_frame;
  Eigen::Vector4f object_plane_far_point_in_neg_direction_in_global_frame;
  Eigen::Vector3f object_plane_major_dimensions;
  step_4_object_pose_approximation( *object_plane_cloud_downsampled_in_arm_hand_frame_xyz, object_plane_transform_wrt_arm_hand_frame, object_plane_far_point_in_pos_direction_in_global_frame, object_plane_far_point_in_neg_direction_in_global_frame, object_plane_major_dimensions );
  transform.matrix() = object_plane_transform_wrt_arm_hand_frame;
  //scene_cloud_viewer->addCoordinateSystem(0.1, transform, "object plane frame", 0);
  
  object_plane_rotation_wrt_arm_hand_frame    << object_plane_transform_wrt_arm_hand_frame(0,0), object_plane_transform_wrt_arm_hand_frame(0,1), object_plane_transform_wrt_arm_hand_frame(0,2),
                                                 object_plane_transform_wrt_arm_hand_frame(1,0), object_plane_transform_wrt_arm_hand_frame(1,1), object_plane_transform_wrt_arm_hand_frame(1,2),
                                                 object_plane_transform_wrt_arm_hand_frame(2,0), object_plane_transform_wrt_arm_hand_frame(2,1), object_plane_transform_wrt_arm_hand_frame(2,2);
  object_plane_translation_wrt_arm_hand_frame << object_plane_transform_wrt_arm_hand_frame(0,3), object_plane_transform_wrt_arm_hand_frame(1,3), object_plane_transform_wrt_arm_hand_frame(2,3);
  
  // object plane cloud in its own frame
  Eigen::Matrix4f object_plane_transform_wrt_arm_hand_frame_inverse;
  object_plane_transform_wrt_arm_hand_frame_inverse << object_plane_rotation_wrt_arm_hand_frame.transpose(), -object_plane_rotation_wrt_arm_hand_frame.transpose()*object_plane_translation_wrt_arm_hand_frame,  // from khalil's book page 21
                                                       0, 0, 0, 1;
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_arm_hand_frame_xyz, *object_plane_cloud_downsampled_in_object_plane_frame_xyz, object_plane_transform_wrt_arm_hand_frame_inverse);
  copyPointCloud(*object_plane_cloud_downsampled_in_object_plane_frame_xyz, *object_plane_cloud_downsampled_in_object_plane_frame_xyzrgb);
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Defining and drawing special ellipsoids
  // gripper support region special ellipsoid
  double gripper_support_x; double gripper_support_y; double gripper_support_z;
  double gripper_support_offset_x; double gripper_support_offset_y; double gripper_support_offset_z;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_support_point_cloud_in_gripper_frame (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if(gripper_model == "allegro_right_hand"){
    gripper_support_x = 0.003; gripper_support_y = 0.047; gripper_support_z = 0.047;
    gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.0475;
  }
  else if(gripper_model == "franka_gripper"){
    gripper_support_x = 0.02; gripper_support_y = 0.08; gripper_support_z = 0.001;
    gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.065;
  }
  // draw
  parameter_vector << gripper_support_x, gripper_support_y, gripper_support_z;
  offset_vector    << gripper_support_offset_x, gripper_support_offset_y, gripper_support_offset_z;
  construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
  
  // object plane special ellipsoid
  double object_plane_x; double object_plane_y; double object_plane_z;
  double object_plane_offset_x; double object_plane_offset_y; double object_plane_offset_z;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_plane_special_ellipsoid_point_cloud_in_object_plane_frame  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame      (new pcl::PointCloud<pcl::PointXYZRGB>);
  // define and draw object plane special ellipsoid
  object_plane_x = 0.15;       object_plane_y = 0.05;         object_plane_z = 0.3;
  object_plane_offset_x = 0.0; object_plane_offset_y = -0.05; object_plane_offset_z = 0.0;
  parameter_vector << object_plane_x, object_plane_y, object_plane_z;
  offset_vector    << object_plane_offset_x, object_plane_offset_y, object_plane_offset_z;
  construct_special_ellipsoid_point_cloud( object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, parameter_vector, offset_vector, 500, 30, 255, 0, 0 );
  pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, object_plane_transform_wrt_arm_hand_frame);
  
  // gripper approximation as a set of special ellipsoids
  std::vector<double> gripper_x, gripper_y, gripper_z;
  std::vector<double> gripper_offset_x, gripper_offset_y, gripper_offset_z;
  if(gripper_model == "allegro_right_hand"){
    gripper_x.push_back(0.04);     gripper_y.push_back(0.04);     gripper_z.push_back(0.02);     gripper_offset_x.push_back(-0.02);     gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.01);  // connection
    gripper_x.push_back(0.015);    gripper_y.push_back(0.045);    gripper_z.push_back(0.015);    gripper_offset_x.push_back(0.05);      gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.05);  // realsense camera
    gripper_x.push_back(0.017);    gripper_y.push_back(0.059);    gripper_z.push_back(0.059);    gripper_offset_x.push_back(-0.017);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.057);  // palm
    gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.05);    gripper_offset_z.push_back(0.17); // index
    gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17); // middle
    gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(-0.05);   gripper_offset_z.push_back(0.17); // pinky
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.070);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17);   // index, middle, pinky
    gripper_x.push_back(0.014);    gripper_y.push_back(0.08);     gripper_z.push_back(0.012);    gripper_offset_x.push_back(-0.022);    gripper_offset_y.push_back(0.11);    gripper_offset_z.push_back(0.022);  // thumb
  }
  else if(gripper_model == "franka_gripper"){
    gripper_x.push_back(0.05);     gripper_y.push_back(0.04);     gripper_z.push_back(0.02);     gripper_offset_x.push_back(-0.01);     gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.02);   // connection
    gripper_x.push_back(0.015);    gripper_y.push_back(0.045);    gripper_z.push_back(0.015);    gripper_offset_x.push_back(-0.07);     gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.03);   // realsense camera
    gripper_x.push_back(0.025);    gripper_y.push_back(0.11);     gripper_z.push_back(0.045);    gripper_offset_x.push_back(0.0);       gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.0225);  // gripper base
    gripper_x.push_back(0.01);     gripper_y.push_back(0.01);     gripper_z.push_back(0.03);     gripper_offset_x.push_back(0.0);       gripper_offset_y.push_back(0.05);    gripper_offset_z.push_back(0.09);    // right finger
    gripper_x.push_back(0.01);     gripper_y.push_back(0.01);     gripper_z.push_back(0.03);     gripper_offset_x.push_back(0.0);       gripper_offset_y.push_back(-0.05);   gripper_offset_z.push_back(0.09);    // left finger
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_as_set_of_special_ellipsoids_in_gripper_frame    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_as_set_of_special_ellipsoids_in_arm_hand_frame   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  dummy_cloud_xyzrgb                                       (new pcl::PointCloud<pcl::PointXYZRGB>);
  for(unsigned int j=0; j<gripper_x.size(); j++){
    parameter_vector << gripper_x[j], gripper_y[j], gripper_z[j];
    offset_vector    << gripper_offset_x[j], gripper_offset_y[j], gripper_offset_z[j];
    if(j<=1)
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 20, 255, 0, 0 );
    else if(j>2)
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 200, 20, 255, 0, 0 );
    else
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 300, 20, 255, 0, 0 );
    
    if(j==6)
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 20, 255, 0, 0 );
    *gripper_as_set_of_special_ellipsoids_in_gripper_frame += *dummy_cloud_xyzrgb;
  }
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_gripper_frame, *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, gripper_wrt_arm_hand_frame_transform);
  
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // load gripper workspace spheres and compute the centroid in the gripper frame
  // declarations for allegro hand
  pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_convex_offset                   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_convex_offset                   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_convex_offset                  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_convex_offset                   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     thumb_workspace_convex_parameter                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     index_workspace_convex_parameter                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     middle_workspace_convex_parameter               (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     pinky_workspace_convex_parameter                (new pcl::PointCloud<pcl::PointXYZ>);
  
  // declarations for panda gripper
  pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_convex_offset            (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_convex_offset             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     right_finger_workspace_convex_parameter         (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     left_finger_workspace_convex_parameter          (new pcl::PointCloud<pcl::PointXYZ>);
  
  // declarations for both
  pcl::PointXYZ convex_shape_offset;
  pcl::PointXYZ convex_shape_parameter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_augmented_workspace_xyzrgb              (new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector3f gripper_workspace_centroid_point_in_gripper_frame;
  
  if(gripper_model == "allegro_right_hand"){
    load_allegro_right_hand_workspace_spheres( gripper_augmented_workspace_xyzrgb,  gripper_workspace_centroid_point_in_gripper_frame,
                                               thumb_workspace_convex_parameter,    thumb_workspace_convex_offset, 
                                               index_workspace_convex_parameter,    index_workspace_convex_offset, 
                                               middle_workspace_convex_parameter,   middle_workspace_convex_offset, 
                                               pinky_workspace_convex_parameter,    pinky_workspace_convex_offset );}
  else if(gripper_model == "franka_gripper"){
    load_franka_gripper_workspace_spheres( gripper_augmented_workspace_xyzrgb,      gripper_workspace_centroid_point_in_gripper_frame,
                                           right_finger_workspace_convex_parameter, right_finger_workspace_convex_offset, 
                                           left_finger_workspace_convex_parameter,  left_finger_workspace_convex_offset );}
  
  
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  begin2 = clock();
  
  int orientation_samples = 30;
  double orientation_range = 2*M_PI;
  double orientation_step = orientation_range/orientation_samples;
  
  double special_ellipsoid_value;
  double sphere_value;
  
  bool gripper_collides_with_object_plane = false;
  bool gripper_collides_with_object       = false;
  bool object_touches_gripper_support     = false;
  
  int counter;
  
  double time_elapsed_checking_gripper_collision_with_table        = 0.0;
  double time_elapsed_checking_gripper_collision_with_object       = 0.0;
  double time_elapsed_checking_object_contact_with_gripper_support = 0.0;
  
  int gripper_collide_with_table   = 0;
  int gripper_collide_with_object  = 0;
  int gripper_contacts_with_object = 0;
  
  double distance_between_gripper_support_and_object_centroid;
  double distance_between_gripper_support_and_object_centroid_best = 1000.0;
  
  Eigen::Vector3f workspace_centroid_wrt_gripper_frame_translation;
  Eigen::Matrix4f workspace_centroid_wrt_gripper_frame_transform;
  Eigen::Matrix4f workspace_centroid_wrt_gripper_frame_transform_inverse;
  workspace_centroid_wrt_gripper_frame_translation << gripper_workspace_centroid_point_in_gripper_frame(0), gripper_workspace_centroid_point_in_gripper_frame(1), gripper_workspace_centroid_point_in_gripper_frame(2);
  workspace_centroid_wrt_gripper_frame_transform << Eigen::Matrix3f::Identity(), workspace_centroid_wrt_gripper_frame_translation,
                                                    0,0,0,1;
  workspace_centroid_wrt_gripper_frame_transform_inverse << Eigen::Matrix3f::Identity(), -Eigen::Matrix3f::Identity()*workspace_centroid_wrt_gripper_frame_translation,
                                                            0, 0, 0, 1;
  
  // we will do translation to each object sample point in the gripper centroid frame
  Eigen::Vector3f object_frame_wrt_gripper_centroid_frame_translation;
  Eigen::Matrix3f object_frame_wrt_gripper_centroid_frame_rotation;
  Eigen::Matrix4f object_frame_wrt_gripper_centroid_frame_transform;
  object_frame_wrt_gripper_centroid_frame_transform = workspace_centroid_wrt_gripper_frame_transform_inverse*gripper_wrt_arm_hand_frame_inverse_transform*object_transform_wrt_arm_hand_frame;
  pcl::transformPointCloud(*object_sampling_in_object_frame_xyzrgb, *object_sampling_in_gripper_centroid_frame_xyzrgb, object_frame_wrt_gripper_centroid_frame_transform);
  
  Eigen::Vector3f gripper_centroid_translation_in_gripper_centroid_frame;
  Eigen::Matrix3f gripper_centroid_rotation_in_gripper_centroid_frame;
  Eigen::Matrix4f gripper_centroid_transform_in_gripper_centroid_frame;
  Eigen::Matrix4f gripper_centroid_transform_in_gripper_centroid_frame_inverse;
  
  Eigen::Matrix4f gripper_centroid_transform_before_orientation_loop;
  
  
  
  
  
  
  
  
  
  Eigen::Vector3f gripper_translation;
  Eigen::Matrix3f gripper_rotation;
  Eigen::Matrix4f gripper_transform;
  Eigen::Matrix4f inverse_gripper_transform;
  
  Eigen::Vector4f object_centroid_point_transformed;
  Eigen::Matrix4f best_gripper_transform;
  
  
  
  
  
  
  
  // GRASPING CODE
  // iterate through all points in the "object sampling cloud"
  for(unsigned int i=0; i<object_sampling_in_arm_hand_frame_xyzrgb->points.size(); i++){
  //for(unsigned int i=0; i<1; i++){
    
    // STEP : OK
    // place gripper workspace centroid at the current "object sampled point"
    // iterating through sample points in the gripper centroid frame
    // rotate the gripper workspace centroid to be alligned with the object orientation
    // then rotate it to be perpendicular to the object major axis
    if(gripper_model == "allegro_right_hand"){
      gripper_centroid_rotation_in_gripper_centroid_frame << object_rotation_wrt_arm_hand_frame*Rotx_float(M_PI/2);}
    else if(gripper_model == "franka_gripper"){
      gripper_centroid_rotation_in_gripper_centroid_frame << object_rotation_wrt_arm_hand_frame*Roty_float(M_PI/2);}
    gripper_centroid_translation_in_gripper_centroid_frame << object_sampling_in_arm_hand_frame_xyzrgb->points[i].x,   object_sampling_in_arm_hand_frame_xyzrgb->points[i].y,   object_sampling_in_arm_hand_frame_xyzrgb->points[i].z;
    gripper_centroid_transform_in_gripper_centroid_frame   << gripper_centroid_rotation_in_gripper_centroid_frame, gripper_centroid_translation_in_gripper_centroid_frame,
                                                              0,0,0,1;
    
    
    
    
    // STEP : OK
    // iterate through the range of possible orientations about object's major axis (z-axis)
    gripper_centroid_transform_before_orientation_loop = gripper_centroid_transform_in_gripper_centroid_frame;
    dummy_translation << 0,0,0;
    for(unsigned int j=0; j<orientation_samples; j++){
    //for(unsigned int j=0; j<=5; j++){
      
      if(gripper_model == "allegro_right_hand"){
        dummy_rotation = Roty_float(M_PI/4+j*orientation_range/orientation_samples);}
      else if(gripper_model == "franka_gripper"){
        dummy_rotation = Rotx_float(M_PI/4+j*orientation_range/orientation_samples);}
      dummy_transform << dummy_rotation, dummy_translation,
                         0,0,0,1;
      gripper_centroid_transform_in_gripper_centroid_frame = gripper_centroid_transform_before_orientation_loop*dummy_transform;
      
      // then transform this motion to the arm hand frame
      gripper_transform = gripper_centroid_transform_in_gripper_centroid_frame*workspace_centroid_wrt_gripper_frame_transform_inverse*gripper_wrt_arm_hand_frame_inverse_transform;
      pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, gripper_transform);
      
      gripper_rotation    << gripper_transform(0,0), gripper_transform(0,1), gripper_transform(0,2),
                             gripper_transform(1,0), gripper_transform(1,1), gripper_transform(1,2),
                             gripper_transform(2,0), gripper_transform(2,1), gripper_transform(2,2);
      gripper_translation << gripper_transform(0,3), gripper_transform(1,3), gripper_transform(2,3);
      
      
      
      
      
      // STEP : OK
      // check if the current gripper pose collides with object_plane
      // transform the gripper point cloud to object_plane frame to be able to use the special ellipsoid
      begin3 = clock();
      pcl::transformPointCloud(*gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_object_plane_frame_xyzrgb, object_plane_transform_wrt_arm_hand_frame_inverse);
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
	    time_elapsed_checking_gripper_collision_with_table += time_spent;
      
      
      
      
      
      // STEP : OK
      // if the gripper doesn't collide with the table (object plane)
      // check that the gripper doesn't collide with object
      // this is done by checking gripper special ellipsoids
      // first apply the inverse gripper transform on the object cloud
      // to make it light : we use downsampled object cloud
      begin4 = clock();
      inverse_gripper_transform << gripper_rotation.transpose(), -gripper_rotation.transpose()*gripper_translation,  // from khalil's book page 21
                                   0, 0, 0, 1;
      pcl::transformPointCloud(*object_cloud_downsampled_in_arm_hand_frame_xyzrgb, *object_cloud_transformed_in_arm_hand_frame_xyzrgb, inverse_gripper_transform);
      pcl::transformPointCloud(*object_cloud_transformed_in_arm_hand_frame_xyzrgb, *object_cloud_transformed_in_gripper_frame_xyzrgb,  gripper_wrt_arm_hand_frame_inverse_transform);
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
	    time_elapsed_checking_gripper_collision_with_object += time_spent;
	    
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      // STEP
      // if gripper doesn't collide with object
      // if gripper doesn't collide with object plane (table)
      // 
      if(gripper_model == "allegro_right_hand"){
      if(!gripper_collides_with_object_plane and !gripper_collides_with_object){
        
        
        // Evaluation Metric#1 : number of object points && number of workspace spheres
        // thumb
        metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyzrgb,                                                                       // input : object related
                                                                                      thumb_workspace_convex_offset, thumb_workspace_convex_parameter,                                                        // input : gripper finger related
                                                                                      object_points_in_thumb_workspace, thumb_workspace_active_spheres_offset,   thumb_workspace_active_spheres_parameter );  // output
        // index
        metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyzrgb,                                                                       // input : object related
                                                                                      index_workspace_convex_offset, index_workspace_convex_parameter,                                                        // input : gripper finger related
                                                                                      object_points_in_index_workspace, index_workspace_active_spheres_offset,   index_workspace_active_spheres_parameter );  // output
        // middle
        metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyzrgb,                                                                       // input : object related
                                                                                      middle_workspace_convex_offset, middle_workspace_convex_parameter,                                                      // input : gripper finger related
                                                                                      object_points_in_middle_workspace, middle_workspace_active_spheres_offset, middle_workspace_active_spheres_parameter ); // output
        // pinky
        metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyzrgb,                                                                       // input : object related
                                                                                      pinky_workspace_convex_offset, pinky_workspace_convex_parameter,                                                        // input : gripper finger related
                                                                                      object_points_in_pinky_workspace, pinky_workspace_active_spheres_offset,   pinky_workspace_active_spheres_parameter );  // output
        
        // Evaluation Metric#2 : how close the object centroid to gripper support offset (special ellipsoid)
        begin5 = clock();
        object_centroid_point_transformed << object_centroid_point_in_arm_hand_frame.x, object_centroid_point_in_arm_hand_frame.y, object_centroid_point_in_arm_hand_frame.z, 1;
        object_centroid_point_transformed = inverse_gripper_transform*object_centroid_point_transformed;  // now we have the object centroid point transformed in arm hand frame
        object_centroid_point_transformed = gripper_wrt_arm_hand_frame_inverse_transform*object_centroid_point_transformed;  // now we have the object centroid point transformed in gripper frame
        distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_x-object_centroid_point_transformed(0)),2)+
                                                                    pow(fabs(gripper_support_offset_y-object_centroid_point_transformed(1)),2)+
                                                                    pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
        end = clock();
        time_spent = (double)( end - begin5 )/ CLOCKS_PER_SEC;
        time_elapsed_checking_object_contact_with_gripper_support += time_spent;
        
        
        // Evaluation
        // select the best gripper pose
        // for the moment we select the pose having the most object points intersecting with gripper workspace
        //if( object_touches_gripper_support and ( (object_points_in_thumb_workspace->size() + object_points_in_index_workspace->size() + object_points_in_middle_workspace->size() + object_points_in_pinky_workspace->size()) > 
        //    (object_points_in_thumb_workspace_best->size() + object_points_in_index_workspace_best->size() + object_points_in_middle_workspace_best->size() + object_points_in_pinky_workspace_best->size() ) ) ){
        
        // this condition ensures all fingers must have solution to touch object
        if( object_points_in_thumb_workspace->size()!=0 and object_points_in_index_workspace->size()!=0 and object_points_in_middle_workspace->size()!=0 and object_points_in_pinky_workspace->size()!=0 ){
        //if( object_points_in_thumb_workspace->size()!=0 and object_points_in_middle_workspace->size()!=0 ){
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
            std::cout<<"metric#2 : "<<distance_between_gripper_support_and_object_centroid_best<<std::endl;
            best_gripper_transform = gripper_transform;
          }
        }
      
      
      }
      }
      else if(gripper_model == "franka_gripper"){
      if(!gripper_collides_with_object_plane and !gripper_collides_with_object){
        
        // Evaluation Metric#1 : number of object points && number of workspace spheres
        // right finger
        metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyzrgb,                                                                                         // input : object related
                                                                                      right_finger_workspace_convex_offset, right_finger_workspace_convex_parameter,                                                            // input : gripper finger related
                                                                                      object_points_in_right_finger_workspace, right_finger_workspace_active_spheres_offset, right_finger_workspace_active_spheres_parameter ); // output
        // left_finger
        metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyzrgb,                                                                                         // input : object related
                                                                                      left_finger_workspace_convex_offset, left_finger_workspace_convex_parameter,                                                              // input : gripper finger related
                                                                                      object_points_in_left_finger_workspace, left_finger_workspace_active_spheres_offset, left_finger_workspace_active_spheres_parameter );    // output
        
        // Evaluation Metric#2 : how close the object centroid to gripper support offset (special ellipsoid)
        begin5 = clock();
        object_centroid_point_transformed << object_centroid_point_in_arm_hand_frame.x, object_centroid_point_in_arm_hand_frame.y, object_centroid_point_in_arm_hand_frame.z, 1;
        object_centroid_point_transformed = inverse_gripper_transform*object_centroid_point_transformed;  // now we have the object centroid point transformed in arm hand frame
        object_centroid_point_transformed = gripper_wrt_arm_hand_frame_inverse_transform*object_centroid_point_transformed;  // now we have the object centroid point transformed in gripper frame
        distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_x-object_centroid_point_transformed(0)),2)+
                                                                    pow(fabs(gripper_support_offset_y-object_centroid_point_transformed(1)),2)+
                                                                    pow(fabs(gripper_support_offset_z-object_centroid_point_transformed(2)),2));
        end = clock();
        time_spent = (double)( end - begin5 )/ CLOCKS_PER_SEC;
        time_elapsed_checking_object_contact_with_gripper_support += time_spent;
        
        
        // Evaluation
        // select the best gripper pose
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
          std::cout<<"metric#2 : "<<distance_between_gripper_support_and_object_centroid_best<<std::endl;
          best_gripper_transform = gripper_transform;
          }
        }
      
      }
      }
      
      
      
      
      
      /*
      // object cloud
      scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,          "object cloud");
      
      // object plane cloud
      *object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb += *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame;
      scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,      "table cloud");
      
      // object sampling cloud
      scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                "object sampling cloud");
      
      // gripper cloud
      //*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb += *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame;
      *gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb += *gripper_as_set_of_special_ellipsoids_in_gripper_frame;
      
      scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, black_color,           "gripper cloud in arm hand frame");
      scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, black_color_again,     "gripper cloud transformed in arm hand frame");
      //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
      
      scene_cloud_viewer->spinOnce();
	    
      while ( !scene_cloud_viewer->wasStopped() ){scene_cloud_viewer->spinOnce();}
      */
      
      
      //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
      //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color, "gripper cloud in gripper frame");
      //scene_cloud_viewer->updatePointCloud(scene_cloud_xyzrgb, scene_cloud_rgb, "scene cloud viewer");
      
      usleep(1);
      
    }
  }
  
  end = clock();
	time_spent = (double)( end - begin2 )/ CLOCKS_PER_SEC;
	std::cout << "time spent in checking gripper collision with table        = " << time_elapsed_checking_gripper_collision_with_table        << std::endl;
	std::cout << "time spent in checking gripper collision with object       = " << time_elapsed_checking_gripper_collision_with_object       << std::endl;
  std::cout << "time spent in checking object contact with gripper support = " << time_elapsed_checking_object_contact_with_gripper_support << std::endl;
  std::cout << "time spent in scanning for best gripper pose               = " << time_spent << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyzrgb->points.size() << " iterations, gripper collide with table  : " << gripper_collide_with_table   << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyzrgb->points.size() << " iterations, gripper collide with object : " << gripper_collide_with_object  << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyzrgb->points.size() << " iterations, gripper contacts with object: " << gripper_contacts_with_object << " times." << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Draw the workspace spheres of the best gripper pose
  if(gripper_model == "allegro_right_hand"){
    // thumb
    scene_cloud_viewer->addPointCloud(thumb_workspace_spheres_best, red_color, "thumb workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_thumb_workspace_best, red_color, "thumb workspace points");
    for(unsigned int j=0; j<thumb_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << thumb_workspace_active_spheres_parameter_best->points[j].x, thumb_workspace_active_spheres_parameter_best->points[j].y, thumb_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << thumb_workspace_active_spheres_offset_best->points[j].x, thumb_workspace_active_spheres_offset_best->points[j].y, thumb_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *thumb_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*thumb_workspace_spheres_best, *thumb_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*thumb_workspace_spheres_best, *thumb_workspace_spheres_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(thumb_workspace_spheres_best, red_color, "thumb workspace spheres");
    pcl::transformPointCloud(*object_points_in_thumb_workspace_best, *object_points_in_thumb_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*object_points_in_thumb_workspace_best, *object_points_in_thumb_workspace_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(object_points_in_thumb_workspace_best, red_color, "thumb workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "thumb workspace points");
    
    // index
    scene_cloud_viewer->addPointCloud(index_workspace_spheres_best, green_color, "index workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_index_workspace_best, green_color, "index workspace points");
    for(unsigned int j=0; j<index_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << index_workspace_active_spheres_parameter_best->points[j].x, index_workspace_active_spheres_parameter_best->points[j].y, index_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << index_workspace_active_spheres_offset_best->points[j].x, index_workspace_active_spheres_offset_best->points[j].y, index_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *index_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*index_workspace_spheres_best, *index_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*index_workspace_spheres_best, *index_workspace_spheres_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(index_workspace_spheres_best, green_color, "index workspace spheres");
    pcl::transformPointCloud(*object_points_in_index_workspace_best, *object_points_in_index_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*object_points_in_index_workspace_best, *object_points_in_index_workspace_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(object_points_in_index_workspace_best, green_color, "index workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "index workspace points");
    
    // middle
    scene_cloud_viewer->addPointCloud(middle_workspace_spheres_best, blue_color, "middle workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_middle_workspace_best, blue_color, "middle workspace points");
    for(unsigned int j=0; j<middle_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << middle_workspace_active_spheres_parameter_best->points[j].x, middle_workspace_active_spheres_parameter_best->points[j].y, middle_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << middle_workspace_active_spheres_offset_best->points[j].x, middle_workspace_active_spheres_offset_best->points[j].y, middle_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *middle_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*middle_workspace_spheres_best, *middle_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*middle_workspace_spheres_best, *middle_workspace_spheres_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(middle_workspace_spheres_best, blue_color, "middle workspace spheres");
    pcl::transformPointCloud(*object_points_in_middle_workspace_best, *object_points_in_middle_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*object_points_in_middle_workspace_best, *object_points_in_middle_workspace_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(object_points_in_middle_workspace_best, blue_color, "middle workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "middle workspace points");
    
    // pinky
    scene_cloud_viewer->addPointCloud(pinky_workspace_spheres_best, grey_color, "pinky workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_pinky_workspace_best, grey_color, "pinky workspace points");
    for(unsigned int j=0; j<pinky_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << pinky_workspace_active_spheres_parameter_best->points[j].x, pinky_workspace_active_spheres_parameter_best->points[j].y, pinky_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << pinky_workspace_active_spheres_offset_best->points[j].x, pinky_workspace_active_spheres_offset_best->points[j].y, pinky_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *pinky_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*pinky_workspace_spheres_best, *pinky_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*pinky_workspace_spheres_best, *pinky_workspace_spheres_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(pinky_workspace_spheres_best, grey_color, "pinky workspace spheres");
    pcl::transformPointCloud(*object_points_in_pinky_workspace_best, *object_points_in_pinky_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*object_points_in_pinky_workspace_best, *object_points_in_pinky_workspace_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(object_points_in_pinky_workspace_best, grey_color, "pinky workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "pinky workspace points");
  }
  else if(gripper_model == "franka_gripper"){
    // right_finger
    scene_cloud_viewer->addPointCloud(right_finger_workspace_spheres_best, red_color_again, "right_finger workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_right_finger_workspace_best, red_color_again, "right_finger workspace points");
    for(unsigned int j=0; j<right_finger_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << right_finger_workspace_active_spheres_parameter_best->points[j].x, right_finger_workspace_active_spheres_parameter_best->points[j].y, right_finger_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << right_finger_workspace_active_spheres_offset_best->points[j].x, right_finger_workspace_active_spheres_offset_best->points[j].y, right_finger_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *right_finger_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*right_finger_workspace_spheres_best, *right_finger_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*right_finger_workspace_spheres_best, *right_finger_workspace_spheres_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(right_finger_workspace_spheres_best, red_color_again, "right_finger workspace spheres");
    pcl::transformPointCloud(*object_points_in_right_finger_workspace_best, *object_points_in_right_finger_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*object_points_in_right_finger_workspace_best, *object_points_in_right_finger_workspace_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(object_points_in_right_finger_workspace_best, red_color_again, "right_finger workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "right_finger workspace points");
    
    // left_finger
    scene_cloud_viewer->addPointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_left_finger_workspace_best, green_color_again, "left_finger workspace points");
    for(unsigned int j=0; j<left_finger_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << left_finger_workspace_active_spheres_parameter_best->points[j].x, left_finger_workspace_active_spheres_parameter_best->points[j].y, left_finger_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << left_finger_workspace_active_spheres_offset_best->points[j].x, left_finger_workspace_active_spheres_offset_best->points[j].y, left_finger_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *left_finger_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*left_finger_workspace_spheres_best, *left_finger_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*left_finger_workspace_spheres_best, *left_finger_workspace_spheres_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
    pcl::transformPointCloud(*object_points_in_left_finger_workspace_best, *object_points_in_left_finger_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    pcl::transformPointCloud(*object_points_in_left_finger_workspace_best, *object_points_in_left_finger_workspace_best, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
    scene_cloud_viewer->updatePointCloud(object_points_in_left_finger_workspace_best, green_color_again, "left_finger workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "left_finger workspace points");
  }
  
  
  
  
  
  
  // object cloud
  //scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,          "object cloud");
  
  // object plane cloud
  *object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb += *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame;
  //scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,      "table cloud");
  
  // object sampling cloud
  //scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                "object sampling cloud");
  
  // gripper cloud
  //*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb += *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame;
  //scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, black_color,           "gripper cloud in arm hand frame");
  //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color,            "gripper cloud in gripper frame");
  //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
  
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, best_gripper_transform);
  //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, black_color_again,     "gripper cloud transformed in arm hand frame");
  
  //
  //scene_cloud_viewer->updatePointCloud(scene_cloud_xyzrgb, scene_cloud_rgb, "scene cloud viewer");
  
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by this program = " << time_spent << std::endl;
  
  /*
  if(gripper_model == "allegro_right_hand")   // for allegro hand
    scene_cloud_viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
  else if(gripper_model == "franka_gripper")  // for franka gripper
    scene_cloud_viewer->setCameraPosition(-0.569223, -0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, 0.38203, 0.231229, 0.894755, 0);
  */
  //scene_cloud_viewer->setCameraPosition(0.3593, -0.548793, -0.0455987, 0.114555, -0.105053, 0.506815, 0.21746, 0.805748, -0.550892, 0);
  scene_cloud_viewer->setCameraPosition(0.160332, -0.648051, 0.210412, 0.0889137, -0.0998327, 0.490031, 0.314008, 0.463524, -0.82858, 0);
  
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (gripper_cloud_in_gripper_frame_xyzrgb, 0, 0, 0);
  //scene_cloud_viewer->addPointCloud(gripper_cloud_in_gripper_frame_xyzrgb, custom_color,          "custom cloud");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,      "custom cloud");
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (gripper_cloud_downsampled_in_gripper_frame_xyzrgb, 0, 0, 0);
  //scene_cloud_viewer->addPointCloud(gripper_cloud_downsampled_in_gripper_frame_xyzrgb, custom_color,           "custom cloud");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,      "custom cloud");
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (gripper_as_set_of_special_ellipsoids_in_gripper_frame, 255, 0, 0);
  //scene_cloud_viewer->addPointCloud(gripper_as_set_of_special_ellipsoids_in_gripper_frame, custom_color2,           "custom cloud2");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,      "custom cloud2");
  
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> custom_color2(gripper_augmented_workspace_xyzrgb);
  //scene_cloud_viewer->addPointCloud(gripper_augmented_workspace_xyzrgb, custom_color2,           "custom cloud2");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,      "custom cloud2");
  
  // object + table clouds
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (object_cloud_in_camera_depth_optical_frame_xyzrgb, 255, 0, 255);
  //scene_cloud_viewer->addPointCloud(object_cloud_in_camera_depth_optical_frame_xyzrgb, custom_color,          "custom cloud");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,      "custom cloud");
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (object_plane_cloud_in_camera_depth_optical_frame_xyzrgb, 165, 42, 42);
  //scene_cloud_viewer->addPointCloud(object_plane_cloud_in_camera_depth_optical_frame_xyzrgb, custom_color2,          "custom cloud2");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,      "custom cloud2");
  
  // object + table downsampled clouds
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 255, 0, 255);
  //scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color,          "custom cloud");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9,      "custom cloud");
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 165, 42, 42);
  //scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color2,          "custom cloud2");
  //scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,      "custom cloud2");
  
  /*
  // object + table downsampled clouds + frames
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 255, 0, 255);
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color,          "custom cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9,      "custom cloud");
  transform.matrix() = object_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 165, 42, 42);
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color2,          "custom cloud2");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,      "custom cloud2");
  transform.matrix() = object_plane_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "object plane frame", 0);
  */
  
  /*
  // object + table downsampled clouds + frames + plane ellipsoid
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 255, 0, 255);
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color,          "custom cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9,      "custom cloud");
  transform.matrix() = object_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 165, 42, 42);
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color2,          "custom cloud2");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,      "custom cloud2");
  transform.matrix() = object_plane_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "object plane frame", 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color3 (object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, 255, 165, 0);
  scene_cloud_viewer->addPointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, custom_color3,          "custom cloud3");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,      "custom cloud3");
  */
  /*
  // object + table downsampled clouds + frames + object sampling
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 255, 0, 255);
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color,          "custom cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9,      "custom cloud");
  transform.matrix() = object_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 165, 42, 42);
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color2,          "custom cloud2");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,      "custom cloud2");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color3 (object_sampling_in_arm_hand_frame_xyzrgb, 0, 0, 255);
  scene_cloud_viewer->addPointCloud(object_sampling_in_arm_hand_frame_xyzrgb, custom_color3,          "custom cloud3");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15,      "custom cloud3");
  */
  
  
  
  
  
  // object + table downsampled clouds + object sampling + gripper colliding with table
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color (object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 255, 0, 255);
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color,          "custom cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9,      "custom cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color2 (object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, 165, 42, 42);
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb, custom_color2,          "custom cloud2");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,      "custom cloud2");
  
  pcl::transformPointCloud(*object_sampling_in_arm_hand_frame_xyzrgb, *object_sampling_in_arm_hand_frame_xyzrgb, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color3 (object_sampling_in_arm_hand_frame_xyzrgb, 0, 0, 255);
  scene_cloud_viewer->addPointCloud(object_sampling_in_arm_hand_frame_xyzrgb, custom_color3,          "custom cloud3");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 40,      "custom cloud3");
  
  //best_gripper_transform = gripper_transform;
  //*gripper_as_set_of_special_ellipsoids_in_arm_hand_frame += *gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb;
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, best_gripper_transform);
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color4 (gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, 0, 0, 0);
  scene_cloud_viewer->addPointCloud(gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, custom_color4,          "custom cloud4");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,      "custom cloud4");
  
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, best_gripper_transform);
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color5 (gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, 0, 255, 255);
  scene_cloud_viewer->addPointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, custom_color5,          "custom cloud5");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,      "custom cloud5");
  
  
  pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, camera_depth_optical_frame_wrt_arm_hand_frame_transform_inverse);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> custom_color6 (object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, 255, 165, 0);
  scene_cloud_viewer->addPointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, custom_color6,          "custom cloud6");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,      "custom cloud6");
  
  
  
  //std::string save_file_name = "gripper_cloud_as_special_ellipsoids";
  //std::string save_file_name = "gripper_cloud_plus_workspace_spheres";
  //std::string save_file_name = "gripper_cloud_downsampled_plus_workspace_spheres";
  //std::string save_file_name = "object_plus_table_clouds";
  //std::string save_file_name = "object_plus_table_downsampled_clouds";
  //std::string save_file_name = "object_plus_table_downsampled_clouds_plus_frames";
  //std::string save_file_name = "object_plus_table_downsampled_clouds_plus_frames_plus_table_ellipsoid";
  //std::string save_file_name = "object_plus_table_downsampled_clouds_plus_object_sampling";
  
  //std::string save_file_name = "gripper_colliding_with_table";
  //std::string save_file_name = "gripper_colliding_with_object";
  std::string save_file_name = "gripper_best";
  
  
  std::vector<pcl::visualization::Camera> cam;
  while ( !scene_cloud_viewer->wasStopped() ){
    scene_cloud_viewer->spinOnce();
    
    if(gripper_model == "allegro_right_hand")
      scene_cloud_viewer->saveScreenshot(save_file_name+"_allegro.png");
    else if(gripper_model == "franka_gripper")
      scene_cloud_viewer->saveScreenshot(save_file_name+"_franka.png");
    
    //scene_cloud_viewer->saveScreenshot(save_file_name+".png");
    
    scene_cloud_viewer->getCameras(cam);
    //Print recorded points on the screen: 
    cout << "Cam: " << endl 
                 << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
                 << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
                 << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  }
  return (0);
}

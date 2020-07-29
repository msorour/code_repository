/*
run this program using:

reset && cmake .. && make && ./grasping_algorithm ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/storage_bin_0.pcd ../raw_object_pcd_files/storage_bin_1.pcd ../raw_object_pcd_files/storage_bin_2.pcd ../raw_object_pcd_files/storage_bin_tf.txt

reset && cmake .. && make && ./grasping_algorithm ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/dish_brush_0.pcd ../raw_object_pcd_files/dish_brush_1.pcd ../raw_object_pcd_files/dish_brush_2.pcd ../raw_object_pcd_files/dish_brush_tf.txt

reset && cmake .. && make && ./grasping_algorithm ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/sprayer_0.pcd ../raw_object_pcd_files/sprayer_1.pcd ../raw_object_pcd_files/sprayer_2.pcd ../raw_object_pcd_files/sprayer_tf.txt

reset && cmake .. && make && ./grasping_algorithm ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/cup_without_handle_0.pcd ../raw_object_pcd_files/cup_without_handle_1.pcd ../raw_object_pcd_files/cup_without_handle_2.pcd ../raw_object_pcd_files/cup_without_handle_tf.txt

*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "include/declarations.h"
#include "include/useful_implementations.h"
#include "include/grasping_algorithm.h"
#include <math.h>
//#include "/home/franka3/grasping_msorour/code_repository/QuadProgpp/src/QuadProg++.hh"
#include "/home/work/software/QuadProgpp-master/src/QuadProg++.hh"


int main (int argc, char** argv){
  begin = clock();
	time_to_load_clouds_begin = clock();
	
  std::string gripper_file_name      = argv[1];
  std::string file_name1             = argv[2];
  std::string file_name2             = argv[3];
  std::string file_name3             = argv[4];
  std::string tf_matrix_file_name    = argv[5];
  
  if(gripper_file_name.find("allegro_right_hand" )!=std::string::npos){gripper_model = "allegro_right_hand";}
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){gripper_model = "franka_gripper";}
  
  
  // visualization of point cloud
  scene_cloud_viewer->addCoordinateSystem(0.2);   // this is arm hand frame (the origin)
  scene_cloud_viewer->setCameraPosition(-1.53884, 0.506528, -0.636167, -0.171077, 0.068023, 0.333948, 0.522106, -0.203709, -0.828196, 0);
  scene_cloud_viewer->setBackgroundColor(255,255,255);
  
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,                 "object cloud");
  scene_cloud_viewer->addPointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                       "object sampling cloud");
  
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,             "table cloud");
  scene_cloud_viewer->addPointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, orange_color,       "table special ellipsoid");
  
  scene_cloud_viewer->addPointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, cyan_color_again,             "gripper cloud in arm hand frame");
  scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, cyan_color,                   "gripper cloud transformed in arm hand frame");
  //scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again,      "gripper cloud in object plane frame");
  scene_cloud_viewer->addPointCloud(gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, black_color,  "gripper special ellipsoids transformed");
  scene_cloud_viewer->addPointCloud(gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, black_color_again,        "gripper special ellipsoids");
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Reading and downsampling point clouds
  // Read in the cloud data
  pcl::PCDReader reader;
  reader.read(gripper_file_name, *gripper_cloud_downsampled_in_gripper_frame_xyz);
  
  Eigen::MatrixXd tf = readMatrix(tf_matrix_file_name.c_str());
  Eigen::Matrix4d tf1, tf2, tf3;
  tf1 << tf(0,0),  tf(0,1),  tf(0,2),  tf(0,3),
         tf(1,0),  tf(1,1),  tf(1,2),  tf(1,3),
         tf(2,0),  tf(2,1),  tf(2,2),  tf(2,3),
         tf(3,0),  tf(3,1),  tf(3,2),  tf(3,3);
  tf2 << tf(4,0),  tf(4,1),  tf(4,2),  tf(4,3),
         tf(5,0),  tf(5,1),  tf(5,2),  tf(5,3),
         tf(6,0),  tf(6,1),  tf(6,2),  tf(6,3),
         tf(7,0),  tf(7,1),  tf(7,2),  tf(7,3);
  tf3 << tf(8,0),  tf(8,1),  tf(8,2),  tf(8,3),
         tf(9,0),  tf(9,1),  tf(9,2),  tf(9,3),
         tf(10,0), tf(10,1), tf(10,2), tf(10,3),
         tf(11,0), tf(11,1), tf(11,2), tf(11,3);
  
  // load the 3 view point clouds
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name1, *scene_cloud_xyz_1);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name2, *scene_cloud_xyz_2);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name3, *scene_cloud_xyz_3);
  
  initial_overhead_begin = clock();
	registering_downsampling_segmenting_3_view_point_clouds(scene_cloud_xyz_1, tf1,   scene_cloud_xyz_2, tf2,   scene_cloud_xyz_3, tf3,
                                                          scene_cloud_xyz_1_transformed, scene_cloud_xyz_2_transformed, scene_cloud_xyz_3_transformed,
                                                          scene_cloud_xyz_1_transformed_downsampled, scene_cloud_xyz_2_transformed_downsampled, scene_cloud_xyz_3_transformed_downsampled,
                                                          object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz, object_cloud_downsampled_in_camera_depth_optical_frame_xyz );
  
  // downsampled object point cloud
  std::cout << "Object (downsampled) point cloud has:  " << object_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyz, *object_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  // downsampled object plane point cloud
  std::cout << "Table (downsampled) point cloud has:   " << object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz, *object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  // downsampled gripper point cloud
  std::cout << "Gripper (downsampled) point cloud has: " << gripper_cloud_downsampled_in_gripper_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*gripper_cloud_downsampled_in_gripper_frame_xyz, *gripper_cloud_downsampled_in_gripper_frame_xyzrgb);
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Defining transformations
  // camera optical frame wrt arm hand frame
  camera_depth_optical_frame_wrt_arm_hand_frame_translation << -0.051, 0.023, -0.017; // from calibration
  camera_depth_optical_frame_wrt_arm_hand_frame_transform << Rotz_float(-M_PI/2), camera_depth_optical_frame_wrt_arm_hand_frame_translation, 0,0,0,1;
  transform.matrix() = camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  scene_cloud_viewer->addCoordinateSystem(0.07, transform, "camera depth optical frame", 0);
  
  // gripper frame wrt arm hand frame
  if(gripper_model == "allegro_right_hand"){
    gripper_wrt_arm_hand_frame_translation << -0.0204, 0, 0.02;
    gripper_wrt_arm_hand_frame_transform << Rotz_float(M_PI), gripper_wrt_arm_hand_frame_translation, 0,0,0,1;}
  else if(gripper_model == "franka_gripper"){
    gripper_wrt_arm_hand_frame_translation << 0, 0, 0.005;
    gripper_wrt_arm_hand_frame_transform << Eigen::Matrix3f::Identity(), gripper_wrt_arm_hand_frame_translation, 0,0,0,1;}
  transform.matrix() = gripper_wrt_arm_hand_frame_transform;
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "gripper frame", 0);
  // gripper frame wrt arm hand frame [INVERSE]
  gripper_wrt_arm_hand_frame_rotation << gripper_wrt_arm_hand_frame_transform(0,0), gripper_wrt_arm_hand_frame_transform(0,1), gripper_wrt_arm_hand_frame_transform(0,2),
                                         gripper_wrt_arm_hand_frame_transform(1,0), gripper_wrt_arm_hand_frame_transform(1,1), gripper_wrt_arm_hand_frame_transform(1,2),
                                         gripper_wrt_arm_hand_frame_transform(2,0), gripper_wrt_arm_hand_frame_transform(2,1), gripper_wrt_arm_hand_frame_transform(2,2);
  gripper_wrt_arm_hand_frame_inverse_transform << gripper_wrt_arm_hand_frame_rotation.transpose(), -gripper_wrt_arm_hand_frame_rotation.transpose()*gripper_wrt_arm_hand_frame_translation, 0,0,0,1; // from khalil's book page 21
  
  // camera optical frame wrt gripper frame
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
  step_4_object_pose_approximation( *object_cloud_downsampled_in_arm_hand_frame_xyz, object_transform_wrt_arm_hand_frame, object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions );
  transform.matrix() = object_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  object_rotation_wrt_arm_hand_frame    << object_transform_wrt_arm_hand_frame(0,0), object_transform_wrt_arm_hand_frame(0,1), object_transform_wrt_arm_hand_frame(0,2),
                                           object_transform_wrt_arm_hand_frame(1,0), object_transform_wrt_arm_hand_frame(1,1), object_transform_wrt_arm_hand_frame(1,2),
                                           object_transform_wrt_arm_hand_frame(2,0), object_transform_wrt_arm_hand_frame(2,1), object_transform_wrt_arm_hand_frame(2,2);
  object_translation_wrt_arm_hand_frame << object_transform_wrt_arm_hand_frame(0,3), object_transform_wrt_arm_hand_frame(1,3), object_transform_wrt_arm_hand_frame(2,3);
  
  // sampling the object around its z-axis for scanning
  object_sampling_in_object_frame_xyzrgb->clear();
  object_major_dimensions(0) = 0.90*object_major_dimensions(0);
  object_major_dimensions(1) = 0.90*object_major_dimensions(1);
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
  pcl::transformPointCloud(*object_sampling_in_object_frame_xyzrgb, *object_sampling_in_arm_hand_frame_xyzrgb, object_transform_wrt_arm_hand_frame);
  
  // object centroid location in arm hand frame
  for(unsigned int i=0;i<object_cloud_downsampled_in_arm_hand_frame_xyz->points.size();i++)
    object_centroid_in_arm_hand_frame.add( object_cloud_downsampled_in_arm_hand_frame_xyz->points[i] );
  object_centroid_in_arm_hand_frame.get(object_centroid_point_in_arm_hand_frame);
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Object plane pose approximation
  // compute object plane transformation matrix
  step_4_object_pose_approximation( *object_plane_cloud_downsampled_in_arm_hand_frame_xyz, object_plane_transform_wrt_arm_hand_frame, object_plane_far_point_in_pos_direction_in_global_frame, object_plane_far_point_in_neg_direction_in_global_frame, object_plane_major_dimensions );
  
  transform.matrix() = object_plane_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "object plane frame", 0);
  
  object_plane_rotation_wrt_arm_hand_frame    << object_plane_transform_wrt_arm_hand_frame(0,0), object_plane_transform_wrt_arm_hand_frame(0,1), object_plane_transform_wrt_arm_hand_frame(0,2),
                                                 object_plane_transform_wrt_arm_hand_frame(1,0), object_plane_transform_wrt_arm_hand_frame(1,1), object_plane_transform_wrt_arm_hand_frame(1,2),
                                                 object_plane_transform_wrt_arm_hand_frame(2,0), object_plane_transform_wrt_arm_hand_frame(2,1), object_plane_transform_wrt_arm_hand_frame(2,2);
  object_plane_translation_wrt_arm_hand_frame << object_plane_transform_wrt_arm_hand_frame(0,3), object_plane_transform_wrt_arm_hand_frame(1,3), object_plane_transform_wrt_arm_hand_frame(2,3);
  
  // object plane cloud in its own frame
  object_plane_transform_wrt_arm_hand_frame_inverse << object_plane_rotation_wrt_arm_hand_frame.transpose(), -object_plane_rotation_wrt_arm_hand_frame.transpose()*object_plane_translation_wrt_arm_hand_frame, 0,0,0,1; // from khalil's book page 21
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_arm_hand_frame_xyz, *object_plane_cloud_downsampled_in_object_plane_frame_xyz, object_plane_transform_wrt_arm_hand_frame_inverse);
  copyPointCloud(*object_plane_cloud_downsampled_in_object_plane_frame_xyz, *object_plane_cloud_downsampled_in_object_plane_frame_xyzrgb);
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Defining and drawing special ellipsoids
  // gripper support region special ellipsoid
  if(gripper_model == "allegro_right_hand"){
    gripper_support_x = 0.003; gripper_support_y = 0.047; gripper_support_z = 0.047;
    //gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.0475;
    gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.07;
  }
  else if(gripper_model == "franka_gripper"){
    gripper_support_x = 0.02; gripper_support_y = 0.08; gripper_support_z = 0.001;
    gripper_support_offset_x = 0.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.075;
  }
  // draw
  parameter_vector << gripper_support_x, gripper_support_y, gripper_support_z;
  offset_vector    << gripper_support_offset_x, gripper_support_offset_y, gripper_support_offset_z;
  construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
  
  // object plane special ellipsoid
  // define and draw object plane special ellipsoid
  object_plane_x = 0.35;       object_plane_y = 0.10;         object_plane_z = 0.35;
  object_plane_offset_x = 0.0; object_plane_offset_y = -0.10; object_plane_offset_z = 0.0;
  parameter_vector << object_plane_x, object_plane_y, object_plane_z;
  offset_vector    << object_plane_offset_x, object_plane_offset_y, object_plane_offset_z;
  construct_special_ellipsoid_point_cloud( object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
  pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, object_plane_transform_wrt_arm_hand_frame);
  
  // gripper approximation as a set of special ellipsoids
  if(gripper_model == "allegro_right_hand"){
    gripper_x.push_back(0.017);    gripper_y.push_back(0.059);    gripper_z.push_back(0.059);    gripper_offset_x.push_back(-0.017);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.057);  // palm
    gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.05);    gripper_offset_z.push_back(0.17); // index
    gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17); // middle
    gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(-0.05);   gripper_offset_z.push_back(0.17); // pinky
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.070);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17);   // index, middle, pinky
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.08);     gripper_z.push_back(0.012);    gripper_offset_x.push_back(-0.022);    gripper_offset_y.push_back(0.11);    gripper_offset_z.push_back(0.022);  // thumb
    gripper_x.push_back(0.028);    gripper_y.push_back(0.06);     gripper_z.push_back(0.045);    gripper_offset_x.push_back(0.03);    gripper_offset_y.push_back(0.07);    gripper_offset_z.push_back(0.04);  // thumb
    gripper_x.push_back(0.04);     gripper_y.push_back(0.04);     gripper_z.push_back(0.02);     gripper_offset_x.push_back(-0.02);     gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.01);  // connection
    gripper_x.push_back(0.015);    gripper_y.push_back(0.045);    gripper_z.push_back(0.015);    gripper_offset_x.push_back(0.05);      gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.05);  // realsense camera
  }
  else if(gripper_model == "franka_gripper"){
    gripper_x.push_back(0.025);    gripper_y.push_back(0.11);     gripper_z.push_back(0.045);    gripper_offset_x.push_back(0.0);       gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.0225);  // gripper base
    gripper_x.push_back(0.01);     gripper_y.push_back(0.01);     gripper_z.push_back(0.03);     gripper_offset_x.push_back(0.0);       gripper_offset_y.push_back(0.05);    gripper_offset_z.push_back(0.09);    // right finger
    gripper_x.push_back(0.01);     gripper_y.push_back(0.01);     gripper_z.push_back(0.03);     gripper_offset_x.push_back(0.0);       gripper_offset_y.push_back(-0.05);   gripper_offset_z.push_back(0.09);    // left finger
    gripper_x.push_back(0.05);     gripper_y.push_back(0.04);     gripper_z.push_back(0.02);     gripper_offset_x.push_back(-0.01);     gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.02);   // connection
    gripper_x.push_back(0.015);    gripper_y.push_back(0.045);    gripper_z.push_back(0.015);    gripper_offset_x.push_back(-0.07);     gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(-0.03);   // realsense camera
  }
  dummy_cloud_xyzrgb->clear();
  for(unsigned int j=0; j<gripper_x.size(); j++){
    parameter_vector << gripper_x[j], gripper_y[j], gripper_z[j];
    offset_vector    << gripper_offset_x[j], gripper_offset_y[j], gripper_offset_z[j];
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
    *gripper_as_set_of_special_ellipsoids_in_gripper_frame += *dummy_cloud_xyzrgb;
  }
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_gripper_frame, *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, gripper_wrt_arm_hand_frame_transform);
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Fixing object plane transformation problem !
  // check if object point cloud collides with object plane special ellipsoid
  object_cloud_temp->clear();
  pcl::transformPointCloud(*object_cloud_downsampled_in_arm_hand_frame_xyzrgb, *object_cloud_temp, object_plane_transform_wrt_arm_hand_frame_inverse);
  for(unsigned int k=0; k<object_cloud_temp->size(); k++){
    special_ellipsoid_value =  pow(object_cloud_temp->points[k].x - object_plane_offset_x, 10)/pow(object_plane_x, 10) 
                             + pow(object_cloud_temp->points[k].y - object_plane_offset_y, 10)/pow(object_plane_y, 10) 
                             + pow(object_cloud_temp->points[k].z - object_plane_offset_z, 2) /pow(object_plane_z, 2);
    if( special_ellipsoid_value < 1 )
      collision_counter++;
  }
  if(collision_counter > object_cloud_temp->size()/2){
  	// re-define and re-draw object plane special ellipsoid
		object_plane_x = 0.35;       object_plane_y = 0.05;         object_plane_z = 0.35;
		object_plane_offset_x = 0.0; object_plane_offset_y = 0.05; object_plane_offset_z = 0.0;
		parameter_vector << object_plane_x, object_plane_y, object_plane_z;
		offset_vector    << object_plane_offset_x, object_plane_offset_y, object_plane_offset_z;
		construct_special_ellipsoid_point_cloud( object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
		pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, object_plane_transform_wrt_arm_hand_frame);
  }
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // load gripper workspace spheres and compute the centroid in the gripper frame
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
  
  end = clock();
	time_spent_for_initial_overhead = (double)( end - initial_overhead_begin )/ CLOCKS_PER_SEC;
	std::cout << "total initial overhead (to add to the execution time) = " << time_spent_for_initial_overhead << std::endl;
  
  end = clock();
	time_spent = (double)( end - time_to_load_clouds_begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent to load stuff = " << time_spent << std::endl;
  
  //char ch;
  //std::cout << "maximuize screen please ..." << std::endl;
  //std::cin >> ch;
  
  time_to_run_algorithm_begin = clock();
  begin2 = clock();
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // scanning for best grasp pose
  evaluate_grasp_pose_candidates();
  
  
  
  end = clock();
	time_spent = (double)( end - begin2 )/ CLOCKS_PER_SEC;
	std::cout << "time spent in checking gripper collision with table        = " << time_elapsed_checking_gripper_collision_with_table        << std::endl;
	std::cout << "time spent in checking gripper collision with object       = " << time_elapsed_checking_gripper_collision_with_object       << std::endl;
  std::cout << "time spent in checking object contact with gripper support = " << time_elapsed_checking_object_contact_with_gripper_support << std::endl;
  std::cout << "time spent in scanning for best gripper pose               = " << time_spent << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyzrgb->points.size() << " iterations, gripper collide with table  : " << gripper_collide_with_table   << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyzrgb->points.size() << " iterations, gripper collide with object : " << gripper_collide_with_object  << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyzrgb->points.size() << " iterations, gripper contacts with object: " << gripper_contacts_with_object << " times." << std::endl;
  
  
  end = clock();
	time_spent = (double)( end - time_to_run_algorithm_begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by the ALGORITHM = " << time_spent << std::endl;
  std::cout << "total spent (ALGORITHM + initial overhead) = " << time_spent+time_spent_for_initial_overhead << std::endl;
  
  
  
  
  
  
  
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
    scene_cloud_viewer->updatePointCloud(thumb_workspace_spheres_best, red_color, "thumb workspace spheres");
    pcl::transformPointCloud(*object_points_in_thumb_workspace_best, *object_points_in_thumb_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_thumb_workspace_best, red_color, "thumb workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "thumb workspace points");
    
    // index
    scene_cloud_viewer->addPointCloud(index_workspace_spheres_best, green_color, "index workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_index_workspace_best, green_color, "index workspace points");
    for(unsigned int j=0; j<index_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << index_workspace_active_spheres_parameter_best->points[j].x, index_workspace_active_spheres_parameter_best->points[j].y, index_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << index_workspace_active_spheres_offset_best->points[j].x, index_workspace_active_spheres_offset_best->points[j].y, index_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *index_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*index_workspace_spheres_best, *index_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(index_workspace_spheres_best, green_color, "index workspace spheres");
    pcl::transformPointCloud(*object_points_in_index_workspace_best, *object_points_in_index_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_index_workspace_best, green_color, "index workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "index workspace points");
    
    // middle
    scene_cloud_viewer->addPointCloud(middle_workspace_spheres_best, blue_color, "middle workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_middle_workspace_best, blue_color, "middle workspace points");
    for(unsigned int j=0; j<middle_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << middle_workspace_active_spheres_parameter_best->points[j].x, middle_workspace_active_spheres_parameter_best->points[j].y, middle_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << middle_workspace_active_spheres_offset_best->points[j].x, middle_workspace_active_spheres_offset_best->points[j].y, middle_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *middle_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*middle_workspace_spheres_best, *middle_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(middle_workspace_spheres_best, blue_color, "middle workspace spheres");
    pcl::transformPointCloud(*object_points_in_middle_workspace_best, *object_points_in_middle_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_middle_workspace_best, blue_color, "middle workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "middle workspace points");
    
    // pinky
    scene_cloud_viewer->addPointCloud(pinky_workspace_spheres_best, grey_color, "pinky workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_pinky_workspace_best, grey_color, "pinky workspace points");
    for(unsigned int j=0; j<pinky_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << pinky_workspace_active_spheres_parameter_best->points[j].x, pinky_workspace_active_spheres_parameter_best->points[j].y, pinky_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << pinky_workspace_active_spheres_offset_best->points[j].x, pinky_workspace_active_spheres_offset_best->points[j].y, pinky_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 2, 255, 0, 0 );
      *pinky_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*pinky_workspace_spheres_best, *pinky_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(pinky_workspace_spheres_best, grey_color, "pinky workspace spheres");
    pcl::transformPointCloud(*object_points_in_pinky_workspace_best, *object_points_in_pinky_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_pinky_workspace_best, grey_color, "pinky workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "pinky workspace points");
  }
  else if(gripper_model == "franka_gripper"){
    // right_finger
    scene_cloud_viewer->addPointCloud(right_finger_workspace_spheres_best, red_color_again, "right_finger workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_right_finger_workspace_best, red_color_again, "right_finger workspace points");
    for(unsigned int j=0; j<right_finger_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << right_finger_workspace_active_spheres_parameter_best->points[j].x, right_finger_workspace_active_spheres_parameter_best->points[j].y, right_finger_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << right_finger_workspace_active_spheres_offset_best->points[j].x, right_finger_workspace_active_spheres_offset_best->points[j].y, right_finger_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 50, 2, 255, 0, 0 );
      *right_finger_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*right_finger_workspace_spheres_best, *right_finger_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(right_finger_workspace_spheres_best, red_color_again, "right_finger workspace spheres");
    pcl::transformPointCloud(*object_points_in_right_finger_workspace_best, *object_points_in_right_finger_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_right_finger_workspace_best, red_color_again, "right_finger workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "right_finger workspace points");
    
    // left_finger
    scene_cloud_viewer->addPointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_left_finger_workspace_best, green_color_again, "left_finger workspace points");
    for(unsigned int j=0; j<left_finger_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << left_finger_workspace_active_spheres_parameter_best->points[j].x, left_finger_workspace_active_spheres_parameter_best->points[j].y, left_finger_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << left_finger_workspace_active_spheres_offset_best->points[j].x, left_finger_workspace_active_spheres_offset_best->points[j].y, left_finger_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 50, 2, 255, 0, 0 );
      *left_finger_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*left_finger_workspace_spheres_best, *left_finger_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
    pcl::transformPointCloud(*object_points_in_left_finger_workspace_best, *object_points_in_left_finger_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_left_finger_workspace_best, green_color_again, "left_finger workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "left_finger workspace points");
  }
  
  
  
  
  
  
  
  // object cloud
  scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,                   "object cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7,                   "object cloud");
  
  // object plane cloud
  scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,               "table cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,                   "table cloud");
  
  // object plane special ellipsoid
  scene_cloud_viewer->updatePointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, orange_color,         "table special ellipsoid");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,                   "table special ellipsoid");
  
  // object sampling cloud
  scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                         "object sampling cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "object sampling cloud");
  
  // gripper cloud transformed in arm hand frame
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, cyan_color,                     "gripper cloud transformed in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "gripper cloud transformed in arm hand frame");
  
  // gripper cloud in arm hand frame
  scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, cyan_color_again,               "gripper cloud in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "gripper cloud in arm hand frame");
  
  // gripper special ellipsoids transformed
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, *gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, black_color,    "gripper special ellipsoids transformed");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper special ellipsoids transformed");
  
  // gripper special ellipsoids
  scene_cloud_viewer->updatePointCloud(gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, black_color_again,          "gripper special ellipsoids");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper special ellipsoids");
  
  scene_cloud_viewer->spinOnce();
  
  
  
  // output
  std::cout << "best_gripper_transform = " << std::endl << best_gripper_transform << std::endl;
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by this program = " << time_spent << std::endl;
  
  //std::vector<pcl::visualization::Camera> cam;
  while ( !scene_cloud_viewer->wasStopped() ){
    scene_cloud_viewer->spinOnce();
    //if(gripper_model == "allegro_right_hand")
    //  save_file_name = "allegro_video/allegro_best.png";
    //else if(gripper_model == "franka_gripper")
    //  save_file_name = "franka_video/franka_best.png";
    //scene_cloud_viewer->saveScreenshot(save_file_name);
    //scene_cloud_viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  }
  return (0);
}

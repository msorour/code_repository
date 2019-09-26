
#ifndef __GRASPING_ALGORITHM_H__
#define __GRASPING_ALGORITHM_H__

#include <pcl/surface/concave_hull.h>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include "declarations.h"
#include <chrono>
#include <string>



#define MAXBUFSIZE  ((int) 1e6)
Eigen::MatrixXd readMatrix(const char *filename){
  int cols = 0, rows = 0;
  double buff[MAXBUFSIZE];
  // Read numbers from file into buffer.
  std::ifstream infile;
  infile.open(filename);
  while (! infile.eof()){
    std::string line;
    std::getline(infile, line);
    int temp_cols = 0;
    std::stringstream stream(line);
    while(! stream.eof())
      stream >> buff[cols*rows+temp_cols++];
    if (temp_cols == 0)
      continue;
    if (cols == 0)
      cols = temp_cols;
    rows++;
  }
  infile.close();
  rows--;
  // Populate matrix with numbers.
  Eigen::MatrixXd result(rows,cols);
  for (int i = 0; i < rows; i++)
  for (int j = 0; j < cols; j++)
  result(i,j) = buff[ cols*i+j ];
  return result;
};


void load_object_3_view_point_clouds_and_corresponding_transforms(void){
  tm = readMatrix(transformation_matrix_file_name.c_str());
  tm1 << tm(0,0),  tm(0,1),  tm(0,2),  tm(0,3),
         tm(1,0),  tm(1,1),  tm(1,2),  tm(1,3),
         tm(2,0),  tm(2,1),  tm(2,2),  tm(2,3),
         tm(3,0),  tm(3,1),  tm(3,2),  tm(3,3);
  tm2 << tm(4,0),  tm(4,1),  tm(4,2),  tm(4,3),
         tm(5,0),  tm(5,1),  tm(5,2),  tm(5,3),
         tm(6,0),  tm(6,1),  tm(6,2),  tm(6,3),
         tm(7,0),  tm(7,1),  tm(7,2),  tm(7,3);
  tm3 << tm(8,0),  tm(8,1),  tm(8,2),  tm(8,3),
         tm(9,0),  tm(9,1),  tm(9,2),  tm(9,3),
         tm(10,0), tm(10,1), tm(10,2), tm(10,3),
         tm(11,0), tm(11,1), tm(11,2), tm(11,3);
  
  // load the 3 view point clouds
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name1, *scene_cloud_xyz_1);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name2, *scene_cloud_xyz_2);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name3, *scene_cloud_xyz_3);
}


void load_transformations(void){
  // camera optical frame wrt arm hand frame
  camera_depth_optical_frame_wrt_arm_hand_frame_translation << -0.051, 0.023, -0.017; // from calibration
  camera_depth_optical_frame_wrt_arm_hand_frame_transform << Rotz_float(-M_PI/2), camera_depth_optical_frame_wrt_arm_hand_frame_translation, 0,0,0,1;
  transform.matrix() = camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  //scene_cloud_viewer->addCoordinateSystem(0.07, transform, "camera depth optical frame", 0);
  
  // gripper frame wrt arm hand frame
  if(gripper_model == "allegro_right_hand"){
    gripper_wrt_arm_hand_frame_translation << -0.0204, 0, 0.02;
    gripper_wrt_arm_hand_frame_transform << Rotz_float(M_PI), gripper_wrt_arm_hand_frame_translation, 0,0,0,1;}
  else if(gripper_model == "franka_gripper"){
    gripper_wrt_arm_hand_frame_translation << 0, 0, 0.005;
    gripper_wrt_arm_hand_frame_transform << Eigen::Matrix3f::Identity(), gripper_wrt_arm_hand_frame_translation, 0,0,0,1;}
  transform.matrix() = gripper_wrt_arm_hand_frame_transform;
  //scene_cloud_viewer->addCoordinateSystem(0.1, transform, "gripper frame", 0);
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
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyz,          *object_cloud_downsampled_in_arm_hand_frame_xyz,         camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  pcl::transformPointCloud(*object_cloud_in_camera_depth_optical_frame_xyz,                      *object_cloud_in_arm_hand_frame_xyz,                     camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz,    *object_plane_cloud_downsampled_in_arm_hand_frame_xyz,   camera_depth_optical_frame_wrt_arm_hand_frame_transform);
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_gripper_frame_xyz,                      *gripper_cloud_downsampled_in_arm_hand_frame_xyz,        gripper_wrt_arm_hand_frame_transform);
  
  // clouds in gripper frame
  pcl::transformPointCloud(*object_cloud_downsampled_in_camera_depth_optical_frame_xyz,          *object_cloud_downsampled_in_gripper_frame_xyz,          camera_depth_optical_frame_wrt_gripper_frame_transform);
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz,    *object_plane_cloud_downsampled_in_gripper_frame_xyz,    camera_depth_optical_frame_wrt_gripper_frame_transform);
}


void downsample_gripper_pointcloud(int desired_number_of_points){
	pcl::PointCloud<pcl::PointXYZ>::Ptr     allegro_hand_point_cloud                         (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr     franka_gripper_point_cloud                       (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr     allegro_hand_point_cloud_downsampled             (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr     franka_gripper_point_cloud_downsampled           (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::io::loadPCDFile<pcl::PointXYZ>("../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera.pcd", *allegro_hand_point_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>("../gripper_pcd_model/franka_gripper_model_cloud_plus_camera.pcd", *franka_gripper_point_cloud);
  copyPointCloud(*allegro_hand_point_cloud, *allegro_hand_point_cloud_downsampled);
  copyPointCloud(*franka_gripper_point_cloud, *franka_gripper_point_cloud_downsampled);
  
	double leaf_size = 0.01;
  double i;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PCDWriter writer;
	
  // downsampling gripper point cloud
  // Allegro hand
  i = 0.0;
  while(allegro_hand_point_cloud_downsampled->points.size() > desired_number_of_points){
    i += 0.001;
    vg.setInputCloud(allegro_hand_point_cloud);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*allegro_hand_point_cloud_downsampled);
  }
  std::cout << "allegro hand cloud after downsampling has: " << allegro_hand_point_cloud_downsampled->points.size() << " data points." << std::endl;
  // save downsampled cloud
  writer.write<pcl::PointXYZ>("../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled_"+std::to_string(desired_number_of_points)+".pcd", *allegro_hand_point_cloud_downsampled, false);
  
  // Franka gripper
  i = 0.0;
  while(franka_gripper_point_cloud_downsampled->points.size() > desired_number_of_points){
    i += 0.001;
    vg.setInputCloud(franka_gripper_point_cloud);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*franka_gripper_point_cloud_downsampled);
  }
  std::cout << "franka gripper cloud after downsampling has: " << franka_gripper_point_cloud_downsampled->points.size() << " data points." << std::endl;
  // save downsampled cloud
  writer.write<pcl::PointXYZ>("../gripper_pcd_model/franka_gripper_model_cloud_plus_camera_downsampled_"+std::to_string(desired_number_of_points)+".pcd", *franka_gripper_point_cloud_downsampled, false);
}


void object_pose_approximation( pcl::PointCloud<pcl::PointXYZ> object_cloud_in_global_frame, 
                                Eigen::Matrix4f& object_transform, 
                                Eigen::Vector4f& far_point_in_pos_direction_in_global_frame, 
                                Eigen::Vector4f& far_point_in_neg_direction_in_global_frame, 
                                Eigen::Vector3f& object_major_dimensions ){
  clock_t begin, end;
  begin = clock();
  double time_spent;
  
  //std::cout << "size of object point cloud: " << object_cloud_in_global_frame.size() << std::endl << std::endl;
  
  
  
  
  //
  // creating object coordinate frame
  //
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid;
  pcl::PointXYZ object_centroid_point;
  for(unsigned int i=0;i<object_cloud_in_global_frame.points.size();i++)
    object_centroid.add( object_cloud_in_global_frame.points[i] );
  object_centroid.get(object_centroid_point);
  
  // object major axis estimation (z-axis)
  // generating the object's principal axis (axis of symmetry) we should align the graspable volume around it!
  // get the longest distance accross object vertices in x, y, and z directions, to know along which axis exists the axis of symmetry
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = object_centroid_point;
  far_point_in_neg_direction = object_centroid_point;
  double largest_euclidean_distance_pos=0.0;
  double largest_euclidean_distance_neg=0.0;
  double euclidean_distance;
  
  // TRIAL#3 (Best till now, there is always small error but it is robust)
  // get the extreme points on object
  std::vector<double> eu_distance;
  int index_of_max_pos=0;
  int index_of_max_neg=0;
  
  // get farthest point from centroid
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++)
    eu_distance.push_back( pow(object_cloud_in_global_frame.points[i].x-object_centroid_point.x,2) + pow(object_cloud_in_global_frame.points[i].y-object_centroid_point.y,2) + pow(object_cloud_in_global_frame.points[i].z-object_centroid_point.z,2) );
  
  // fetch this point (name it: "far point in positive side")
  for(unsigned int i=0; i<eu_distance.size(); i++)
    if(eu_distance[i]>eu_distance[index_of_max_pos])
      index_of_max_pos = i;
  
  // get the farthest point from "far point in positive side"
  eu_distance.clear();
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++)
    eu_distance.push_back( pow(object_cloud_in_global_frame.points[i].x-object_cloud_in_global_frame.points[index_of_max_pos].x,2) + pow(object_cloud_in_global_frame.points[i].y-object_cloud_in_global_frame.points[index_of_max_pos].y,2) + pow(object_cloud_in_global_frame.points[i].z-object_cloud_in_global_frame.points[index_of_max_pos].z,2) );
  
  // fetch the point (name it: "far point in negative side")
  for(unsigned int i=0; i<eu_distance.size(); i++)
    if(eu_distance[i]>eu_distance[index_of_max_neg])
      index_of_max_neg = i;
  
  far_point_in_pos_direction.x = object_cloud_in_global_frame.points[index_of_max_pos].x;
  far_point_in_pos_direction.y = object_cloud_in_global_frame.points[index_of_max_pos].y;
  far_point_in_pos_direction.z = object_cloud_in_global_frame.points[index_of_max_pos].z;
  
  far_point_in_neg_direction.x = object_cloud_in_global_frame.points[index_of_max_neg].x;
  far_point_in_neg_direction.y = object_cloud_in_global_frame.points[index_of_max_neg].y;
  far_point_in_neg_direction.z = object_cloud_in_global_frame.points[index_of_max_neg].z;
  /*
  object_major_dimensions <<  far_point_in_pos_direction.x-far_point_in_neg_direction.x,
                              far_point_in_pos_direction.y-far_point_in_neg_direction.y,
                              far_point_in_pos_direction.z-far_point_in_neg_direction.z;
  
  std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  */
  // divide the object verices into two sets, one on the positive side of centroid and the other on the negative side
  pcl::PointCloud<pcl::PointXYZ> cloud_far_pos;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_neg;
  pcl::PointXYZ point;
  double distance_to_far_point_pos;
  double distance_to_far_point_neg;
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    distance_to_far_point_pos = (object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z);
    distance_to_far_point_neg = (object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z);
    if( distance_to_far_point_pos < distance_to_far_point_neg )
      cloud_far_pos.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg.points.push_back( object_cloud_in_global_frame.points[i] );
  }
  
  // getting the 2 far sides points (centroids)
  pcl::PointXYZ centroid_far_pos_point;
  pcl::PointXYZ centroid_far_neg_point;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg;
  
  for(unsigned int i=0;i<cloud_far_pos.points.size();i++)
    centroid_far_pos.add( cloud_far_pos.points[i] );
  centroid_far_pos.get(centroid_far_pos_point);
  
  for(unsigned int i=0;i<cloud_far_neg.points.size();i++)
    centroid_far_neg.add( cloud_far_neg.points[i] );
  centroid_far_neg.get(centroid_far_neg_point);
  
  
  // computing object's major axis (z-axis) orientation
  // major axis vector
  Eigen::Vector3d object_z_axis_vector;
  object_z_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  object_z_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  object_z_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  
  /*
  // normalize the major axis vector:
  Eigen::Vector3d object_z_axis_vector_normalized;
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  double alpha;   // angle about (+ve) x-axis
  double beta;    // angle about (+ve) y-axis
  double gamma;   // angle about (+ve) z-axis
  alpha = acos( object_z_axis_vector_normalized[0] );
  beta  = acos( object_z_axis_vector_normalized[1] );
  gamma = acos( object_z_axis_vector_normalized[2] );
  //std::cout << "alpha = " << alpha << std::endl;
  //std::cout << "beta  = " << beta << std::endl;
  //std::cout << "gamma = " << gamma << std::endl << std::endl;
  */
  
  // computing perpendicular axis to the major one (x-axis)
  // we have to take into account intersection with y-axis to make it more versatile and robust
  double slope = object_z_axis_vector[1]/object_z_axis_vector[0];
  double y_intercept = centroid_far_pos_point.y - slope*centroid_far_pos_point.x;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_pos_x_axis;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_neg_x_axis;
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    if( object_cloud_in_global_frame.points[i].y > (slope*object_cloud_in_global_frame.points[i].x + y_intercept) )
      cloud_far_pos_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
  }
  // getting the 2 far sides points (centroids)
  pcl::PointXYZ centroid_far_pos_point_x_axis;
  pcl::PointXYZ centroid_far_neg_point_x_axis;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos_x_axis;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg_x_axis;
  
  for(unsigned int i=0;i<cloud_far_pos_x_axis.points.size();i++)
    centroid_far_pos_x_axis.add( cloud_far_pos_x_axis.points[i] );
  centroid_far_pos_x_axis.get(centroid_far_pos_point_x_axis);
  
  for(unsigned int i=0;i<cloud_far_neg_x_axis.points.size();i++)
    centroid_far_neg_x_axis.add( cloud_far_neg_x_axis.points[i] );
  centroid_far_neg_x_axis.get(centroid_far_neg_point_x_axis);
  
  Eigen::Vector3d object_x_axis_vector;
  object_x_axis_vector[0] = centroid_far_pos_point_x_axis.x - centroid_far_neg_point_x_axis.x;
  object_x_axis_vector[1] = centroid_far_pos_point_x_axis.y - centroid_far_neg_point_x_axis.y;
  object_x_axis_vector[2] = centroid_far_pos_point_x_axis.z - centroid_far_neg_point_x_axis.z;
  
  // however this just computed x-axis vector might not be perfectly perpendicular to the z-axis previously computed
  // we shall project it onto a perpendicular axis to the z-axis
  // first get a unit vector perpendicular to z-axis
  object_x_axis_vector(1) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(1);
  //object_x_axis_vector(2) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(1)*object_z_axis_vector(1) )/object_z_axis_vector(2);
  //object_x_axis_vector(0) = -( object_x_axis_vector(1)*object_z_axis_vector(1) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(0);
  
  //std::cout<<"object_z_axis_vector.dot(object_x_axis_vector) = "<<object_z_axis_vector.dot(object_x_axis_vector)<<std::endl;
  
  //unit_vector_perpendicular_to_z_axis.normalize();
  // then project the computed x_axis onto it
  //Eigen::Matrix3d dummy_rotation;
  //dummy_rotation << object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x);
  
  // computing y-axis
  // generating an orthogornal frame out of the z-axis and the x-axis vectors
  Eigen::Vector3d object_x_axis_vector_normalized;
  Eigen::Vector3d object_y_axis_vector_normalized;
  Eigen::Vector3d object_z_axis_vector_normalized;
  Eigen::Vector3d unit_x, unit_y, unit_z;
  unit_x << 1,0,0;
  unit_y << 0,1,0;
  unit_z << 0,0,1;
  object_x_axis_vector_normalized = object_x_axis_vector.normalized();
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  //object_y_axis_vector_normalized = object_z_axis_vector_normalized.cross(object_x_axis_vector_normalized);
  object_y_axis_vector_normalized = object_z_axis_vector.cross(object_x_axis_vector);
  object_y_axis_vector_normalized.normalize();
  
  
  
  // compute object transform and show its coordinate system
  Eigen::Matrix3f object_rotation;
  Eigen::Vector3f object_translation;
  
  object_rotation <<  object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x),
                      object_x_axis_vector_normalized.dot(unit_y), object_y_axis_vector_normalized.dot(unit_y), object_z_axis_vector_normalized.dot(unit_y),
                      object_x_axis_vector_normalized.dot(unit_z), object_y_axis_vector_normalized.dot(unit_z), object_z_axis_vector_normalized.dot(unit_z);
  object_translation << object_centroid_point.x,object_centroid_point.y,object_centroid_point.z;
  object_transform << object_rotation, object_translation,
                      0,0,0,1;
  
  
  
  
  
  
  
  // compute the object far points in object frame then make correction
  Eigen::Vector4f far_point_in_pos_direction_in_object_frame;
  Eigen::Vector4f far_point_in_neg_direction_in_object_frame;
  Eigen::Matrix4f inverse_object_transform;
  inverse_object_transform << object_rotation.transpose(), -object_rotation.transpose()*object_translation,  // from khalil's book page 21
                              0, 0, 0, 1;
  
  far_point_in_pos_direction_in_object_frame << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_in_object_frame << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  far_point_in_pos_direction_in_object_frame = inverse_object_transform*far_point_in_pos_direction_in_object_frame;
  far_point_in_neg_direction_in_object_frame = inverse_object_transform*far_point_in_neg_direction_in_object_frame;
  
  /*
  object_major_dimensions <<  fabs(far_point_in_pos_direction_in_object_frame(0)-far_point_in_neg_direction_in_object_frame(0)),
                              fabs(far_point_in_pos_direction_in_object_frame(1)-far_point_in_neg_direction_in_object_frame(1)),
                              fabs(far_point_in_pos_direction_in_object_frame(2)-far_point_in_neg_direction_in_object_frame(2));
  std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  */
  //std::cout << "far_point_in_pos_direction_in_object_frame: " << std::endl << far_point_in_pos_direction_in_object_frame << std::endl;
  //std::cout << "far_point_in_neg_direction_in_object_frame: " << std::endl << far_point_in_neg_direction_in_object_frame << std::endl;
  
  // set x and y components to 0 and leave the z component
  far_point_in_pos_direction_in_object_frame(0) = 0;
  far_point_in_pos_direction_in_object_frame(1) = 0;
  far_point_in_neg_direction_in_object_frame(0) = 0;
  far_point_in_neg_direction_in_object_frame(1) = 0;
  
  // convert to global frame
  far_point_in_pos_direction_in_global_frame = object_transform*far_point_in_pos_direction_in_object_frame;
  far_point_in_neg_direction_in_global_frame = object_transform*far_point_in_neg_direction_in_object_frame;
  
  
  
  
  
  //
  //
  // use the updated points to compute a more accurate object frame
  far_point_in_pos_direction.x = far_point_in_pos_direction_in_global_frame(0);
  far_point_in_pos_direction.y = far_point_in_pos_direction_in_global_frame(1);
  far_point_in_pos_direction.z = far_point_in_pos_direction_in_global_frame(2);
  
  far_point_in_neg_direction.x = far_point_in_neg_direction_in_global_frame(0);
  far_point_in_neg_direction.y = far_point_in_neg_direction_in_global_frame(1);
  far_point_in_neg_direction.z = far_point_in_neg_direction_in_global_frame(2);
  
  // divide the object verices into two sets, one on the positive side of centroid and the other on the negative side
  cloud_far_pos.clear();
  cloud_far_neg.clear();
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    distance_to_far_point_pos = (object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z);
    distance_to_far_point_neg = (object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z);
    if( distance_to_far_point_pos < distance_to_far_point_neg )
      cloud_far_pos.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg.points.push_back( object_cloud_in_global_frame.points[i] );
  }
  
  // getting the 2 far sides points (centroids)
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos_again;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg_again;
  for(unsigned int i=0;i<cloud_far_pos.points.size();i++)
    centroid_far_pos_again.add( cloud_far_pos.points[i] );
  centroid_far_pos_again.get(centroid_far_pos_point);
  for(unsigned int i=0;i<cloud_far_neg.points.size();i++)
    centroid_far_neg.add( cloud_far_neg.points[i] );
  centroid_far_neg.get(centroid_far_neg_point);
  
  // computing object's major axis (z-axis) orientation
  // major axis vector
  object_z_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  object_z_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  object_z_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  
  // computing perpendicular axis to the major one (x-axis)
  // we have to take into account intersection with y-axis to make it more versatile and robust
  slope = object_z_axis_vector[1]/object_z_axis_vector[0];
  y_intercept = centroid_far_pos_point.y - slope*centroid_far_pos_point.x;
  cloud_far_pos_x_axis.clear();
  cloud_far_neg_x_axis.clear();
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    if( object_cloud_in_global_frame.points[i].y > (slope*object_cloud_in_global_frame.points[i].x + y_intercept) )
      cloud_far_pos_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
  }
  
  // getting the 2 far sides points (centroids)
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos_x_axis_again;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg_x_axis_again;
  for(unsigned int i=0;i<cloud_far_pos_x_axis.points.size();i++)
    centroid_far_pos_x_axis_again.add( cloud_far_pos_x_axis.points[i] );
  centroid_far_pos_x_axis_again.get(centroid_far_pos_point_x_axis);
  for(unsigned int i=0;i<cloud_far_neg_x_axis.points.size();i++)
    centroid_far_neg_x_axis_again.add( cloud_far_neg_x_axis.points[i] );
  centroid_far_neg_x_axis_again.get(centroid_far_neg_point_x_axis);
  
  object_x_axis_vector[0] = centroid_far_pos_point_x_axis.x - centroid_far_neg_point_x_axis.x;
  object_x_axis_vector[1] = centroid_far_pos_point_x_axis.y - centroid_far_neg_point_x_axis.y;
  object_x_axis_vector[2] = centroid_far_pos_point_x_axis.z - centroid_far_neg_point_x_axis.z;
  
  // however this just computed x-axis vector might not be perfectly perpendicular to the z-axis previously computed
  // we shall project it onto a perpendicular axis to the z-axis
  // first get a unit vector perpendicular to z-axis
  object_x_axis_vector(1) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(1);
  //object_x_axis_vector(2) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(1)*object_z_axis_vector(1) )/object_z_axis_vector(2);
  //object_x_axis_vector(0) = -( object_x_axis_vector(1)*object_z_axis_vector(1) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(0);
  
  // computing y-axis
  // generating an orthogornal frame out of the z-axis and the x-axis vectors
  unit_x << 1,0,0;
  unit_y << 0,1,0;
  unit_z << 0,0,1;
  object_x_axis_vector_normalized = object_x_axis_vector.normalized();
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  //object_y_axis_vector_normalized = object_z_axis_vector_normalized.cross(object_x_axis_vector_normalized);
  object_y_axis_vector_normalized = object_z_axis_vector.cross(object_x_axis_vector);
  object_y_axis_vector_normalized.normalize();
  
  // compute object transform and show its coordinate system
  object_rotation <<  object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x),
                      object_x_axis_vector_normalized.dot(unit_y), object_y_axis_vector_normalized.dot(unit_y), object_z_axis_vector_normalized.dot(unit_y),
                      object_x_axis_vector_normalized.dot(unit_z), object_y_axis_vector_normalized.dot(unit_z), object_z_axis_vector_normalized.dot(unit_z);
  object_translation << object_centroid_point.x,object_centroid_point.y,object_centroid_point.z;
  object_transform << object_rotation, object_translation,
                      0,0,0,1;
  
  
  /*
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.matrix() = object_transform;
  viewer->addCoordinateSystem(0.1, transform, "object coordinate frame", 0);
  */
  //std::cout << "object transformation matrix: " << std::endl << object_transform << std::endl << std::endl;
  
  
  
  
  
  
  // to compute object major dimensions properly
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_in_object_frame_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(object_cloud_in_global_frame, *object_cloud_in_object_frame_xyz, inverse_object_transform);
  
  far_point_in_pos_direction.x = 0.0;
  far_point_in_pos_direction.y = 0.0;
  far_point_in_pos_direction.z = 0.0;
  
  far_point_in_neg_direction.x = 0.0;
  far_point_in_neg_direction.y = 0.0;
  far_point_in_neg_direction.z = 0.0;
  
  for(unsigned int i=0;i<object_cloud_in_object_frame_xyz->size();i++){
    if( object_cloud_in_object_frame_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = object_cloud_in_object_frame_xyz->points[i].x;
    if( object_cloud_in_object_frame_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = object_cloud_in_object_frame_xyz->points[i].x;
    
    if( object_cloud_in_object_frame_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = object_cloud_in_object_frame_xyz->points[i].y;
    if( object_cloud_in_object_frame_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = object_cloud_in_object_frame_xyz->points[i].y;
    
    if( object_cloud_in_object_frame_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = object_cloud_in_object_frame_xyz->points[i].z;
    if( object_cloud_in_object_frame_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = object_cloud_in_object_frame_xyz->points[i].z;
  }
  
  object_major_dimensions <<  fabs(far_point_in_pos_direction.x-far_point_in_neg_direction.x),
                              fabs(far_point_in_pos_direction.y-far_point_in_neg_direction.y),
                              fabs(far_point_in_pos_direction.z-far_point_in_neg_direction.z);
  //std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  
  
  
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to estimate object pose = " << time_spent << std::endl;
}


void object_pose_approximation_and_sampling(){
  // compute object transformation matrix
  object_pose_approximation( *object_cloud_downsampled_in_arm_hand_frame_xyz, object_transform_wrt_arm_hand_frame, object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions );
  transform.matrix() = object_transform_wrt_arm_hand_frame;
  //scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
  object_rotation_wrt_arm_hand_frame    << object_transform_wrt_arm_hand_frame(0,0), object_transform_wrt_arm_hand_frame(0,1), object_transform_wrt_arm_hand_frame(0,2),
                                           object_transform_wrt_arm_hand_frame(1,0), object_transform_wrt_arm_hand_frame(1,1), object_transform_wrt_arm_hand_frame(1,2),
                                           object_transform_wrt_arm_hand_frame(2,0), object_transform_wrt_arm_hand_frame(2,1), object_transform_wrt_arm_hand_frame(2,2);
  object_translation_wrt_arm_hand_frame << object_transform_wrt_arm_hand_frame(0,3), object_transform_wrt_arm_hand_frame(1,3), object_transform_wrt_arm_hand_frame(2,3);
  
  // sampling the object around its z-axis for scanning
  object_sampling_in_object_frame_xyz->clear();
  object_major_dimensions(0) = 0.90*object_major_dimensions(0);
  object_major_dimensions(1) = 0.90*object_major_dimensions(1);
  object_major_dimensions(2) = 0.70*object_major_dimensions(2);
  for(unsigned int i=0; i<object_sampling_in_x_axis ;i++){
    for(unsigned int j=0; j<object_sampling_in_y_axis ;j++){
      for(unsigned int k=0; k<object_sampling_in_z_axis ;k++){
        point_xyz.x = -object_major_dimensions(0)/2 + object_major_dimensions(0)*i/object_sampling_in_x_axis;
        point_xyz.y = -object_major_dimensions(1)/2 + object_major_dimensions(1)*j/object_sampling_in_y_axis;
        point_xyz.z = -object_major_dimensions(2)/2 + object_major_dimensions(2)*k/object_sampling_in_z_axis;
        object_sampling_in_object_frame_xyz->points.push_back( point_xyz );
      }
    }
  }
  pcl::transformPointCloud(*object_sampling_in_object_frame_xyz, *object_sampling_in_arm_hand_frame_xyz, object_transform_wrt_arm_hand_frame);
  
  // object centroid location in arm hand frame
  for(unsigned int i=0;i<object_cloud_downsampled_in_arm_hand_frame_xyz->points.size();i++)
    object_centroid_in_arm_hand_frame.add( object_cloud_downsampled_in_arm_hand_frame_xyz->points[i] );
  object_centroid_in_arm_hand_frame.get(object_centroid_point_in_arm_hand_frame);
}


void object_plane_pose_approximation(void){
  // compute object plane transformation matrix
  object_pose_approximation( *object_plane_cloud_downsampled_in_arm_hand_frame_xyz, object_plane_transform_wrt_arm_hand_frame, object_plane_far_point_in_pos_direction_in_global_frame, object_plane_far_point_in_neg_direction_in_global_frame, object_plane_major_dimensions );
  
  transform.matrix() = object_plane_transform_wrt_arm_hand_frame;
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "object plane frame", 0);
  
  object_plane_rotation_wrt_arm_hand_frame    << object_plane_transform_wrt_arm_hand_frame(0,0), object_plane_transform_wrt_arm_hand_frame(0,1), object_plane_transform_wrt_arm_hand_frame(0,2),
                                                 object_plane_transform_wrt_arm_hand_frame(1,0), object_plane_transform_wrt_arm_hand_frame(1,1), object_plane_transform_wrt_arm_hand_frame(1,2),
                                                 object_plane_transform_wrt_arm_hand_frame(2,0), object_plane_transform_wrt_arm_hand_frame(2,1), object_plane_transform_wrt_arm_hand_frame(2,2);
  object_plane_translation_wrt_arm_hand_frame << object_plane_transform_wrt_arm_hand_frame(0,3), object_plane_transform_wrt_arm_hand_frame(1,3), object_plane_transform_wrt_arm_hand_frame(2,3);
  
  // object plane cloud in its own frame
  object_plane_transform_wrt_arm_hand_frame_inverse << object_plane_rotation_wrt_arm_hand_frame.transpose(), -object_plane_rotation_wrt_arm_hand_frame.transpose()*object_plane_translation_wrt_arm_hand_frame, 0,0,0,1; // from khalil's book page 21
  pcl::transformPointCloud(*object_plane_cloud_downsampled_in_arm_hand_frame_xyz, *object_plane_cloud_downsampled_in_object_plane_frame_xyz, object_plane_transform_wrt_arm_hand_frame_inverse);  
}


void construct_special_ellipsoid_point_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr& special_ellipsoid_point_cloud, 
                                              Eigen::Vector3f parameter,
                                              Eigen::Vector3f offset,
                                              int point_cloud_samples, 
                                              int power ){
  special_ellipsoid_point_cloud->clear();
  pcl::PointXYZ point_xyz;
  double value_x, value_y, value_z;
  for(unsigned int k=0; k<point_cloud_samples; k++){
    value_x = (-parameter(0) + offset(0)) + k*2*parameter(0)/point_cloud_samples;
    for(unsigned int l=0; l<point_cloud_samples; l++){
      value_y = (-parameter(1) + offset(1)) + l*2*parameter(1)/point_cloud_samples;
      value_z = offset(2) + parameter(2)*sqrt( 1 - pow(value_x-offset(0), power)/pow(parameter(0), power) - pow(value_y-offset(1), power)/pow(parameter(1), power) );
      
      if(!std::isnan(value_z)){
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        special_ellipsoid_point_cloud->points.push_back( point_xyz );
        
        value_z = offset(2) - parameter(2)*sqrt( 1 - pow(value_x-offset(0), power)/pow(parameter(0), power) - pow(value_y-offset(1), power)/pow(parameter(1), power) );
        if(!std::isnan(value_z)){
          point_xyz.x = value_x;
          point_xyz.y = value_y;
          point_xyz.z = value_z;
          special_ellipsoid_point_cloud->points.push_back( point_xyz );
        }
      }
    }
  }
}


void constructing_special_ellipsoids(void){
  // gripper support region special ellipsoid
  if(gripper_model == "allegro_right_hand"){
    gripper_support_x = 0.02; gripper_support_y = 0.047; gripper_support_z = 0.047;
    gripper_support_offset_x = gripper_support_x/2.0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.13;
		
		gripper_support_x_1 = 0.012; gripper_support_y_1 = 0.02; gripper_support_z_1 = 0.02;
    gripper_support_offset_x_1 = gripper_support_x_1/2.0; gripper_support_offset_y_1 = 0.03; gripper_support_offset_z_1 = 0.15;
		//gripper_support_x_1 = 0.02; gripper_support_y_1 = 0.04; gripper_support_z_1 = 0.06;
    //gripper_support_offset_x_1 = gripper_support_x_1; gripper_support_offset_y_1 = gripper_support_y_1; gripper_support_offset_z_1 = gripper_support_z_1;
		
		gripper_support_x_2 = 0.012; gripper_support_y_2 = 0.02; gripper_support_z_2 = 0.02;
    gripper_support_offset_x_2 = gripper_support_x_2/2.0; gripper_support_offset_y_2 = -0.03; gripper_support_offset_z_2 = 0.15;
		
		gripper_support_x_3 = 0.012; gripper_support_y_3 = 0.02; gripper_support_z_3 = 0.02;
    gripper_support_offset_x_3 = gripper_support_x_3/2.0; gripper_support_offset_y_3 = 0.03; gripper_support_offset_z_3 = 0.09;
		
		gripper_support_x_4 = 0.012; gripper_support_y_4 = 0.02; gripper_support_z_4 = 0.02;
    gripper_support_offset_x_4 = gripper_support_x_4/2.0; gripper_support_offset_y_4 = -0.03; gripper_support_offset_z_4 = 0.09;
		
		
		// 1st support area
		parameter_vector << gripper_support_x_1, gripper_support_y_1, gripper_support_z_1;
		offset_vector    << gripper_support_offset_x_1, gripper_support_offset_y_1, gripper_support_offset_z_1;
		construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame_1, parameter_vector, offset_vector, 100, 10 );
		gripper_support_offset_in_gripper_frame_1 << gripper_support_offset_x_1, gripper_support_offset_y_1, gripper_support_offset_z_1,1;
		gripper_support_offset_in_arm_hand_frame_1 = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame_1;
		
		gripper_support_in_gripper_frame_1 << gripper_support_x_1, gripper_support_y_1, gripper_support_z_1,1;
		gripper_support_in_arm_hand_frame_1 = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame_1;
		
		// 2nd support area
		parameter_vector << gripper_support_x_2, gripper_support_y_2, gripper_support_z_2;
		offset_vector    << gripper_support_offset_x_2, gripper_support_offset_y_2, gripper_support_offset_z_2;
		construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame_2, parameter_vector, offset_vector, 100, 10 );
		gripper_support_offset_in_gripper_frame_2 << gripper_support_offset_x_2, gripper_support_offset_y_2, gripper_support_offset_z_2,1;
		gripper_support_offset_in_arm_hand_frame_2 = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame_2;
		
		gripper_support_in_gripper_frame_2 << gripper_support_x_2, gripper_support_y_2, gripper_support_z_2,1;
		gripper_support_in_arm_hand_frame_2 = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame_2;
		
		// 3rd support area
		parameter_vector << gripper_support_x_3, gripper_support_y_3, gripper_support_z_3;
		offset_vector    << gripper_support_offset_x_3, gripper_support_offset_y_3, gripper_support_offset_z_3;
		construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame_3, parameter_vector, offset_vector, 100, 10 );
		gripper_support_offset_in_gripper_frame_3 << gripper_support_offset_x_3, gripper_support_offset_y_3, gripper_support_offset_z_3,1;
		gripper_support_offset_in_arm_hand_frame_3 = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame_3;
		
		gripper_support_in_gripper_frame_3 << gripper_support_x_3, gripper_support_y_3, gripper_support_z_3,1;
		gripper_support_in_arm_hand_frame_3 = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame_3;
		
		// 4th support area
		parameter_vector << gripper_support_x_4, gripper_support_y_4, gripper_support_z_4;
		offset_vector    << gripper_support_offset_x_4, gripper_support_offset_y_4, gripper_support_offset_z_4;
		construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame_4, parameter_vector, offset_vector, 100, 10 );
		gripper_support_offset_in_gripper_frame_4 << gripper_support_offset_x_4, gripper_support_offset_y_4, gripper_support_offset_z_4,1;
		gripper_support_offset_in_arm_hand_frame_4 = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame_4;
		
		gripper_support_in_gripper_frame_4 << gripper_support_x_4, gripper_support_y_4, gripper_support_z_4,1;
		gripper_support_in_arm_hand_frame_4 = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame_4;
  }
  else if(gripper_model == "franka_gripper"){
    gripper_support_x = 0.02; gripper_support_y = 0.06; gripper_support_z = 0.005;
    gripper_support_offset_x = 0; gripper_support_offset_y = 0.0; gripper_support_offset_z = 0.07;
		
		gripper_support_x_1 = 0.01; gripper_support_y_1 = 0.05; gripper_support_z_1 = 0.007;
    gripper_support_offset_x_1 = -0.011; gripper_support_offset_y_1 = 0; gripper_support_offset_z_1 = 0.07;
		
		gripper_support_x_2 = 0.01; gripper_support_y_2 = 0.05; gripper_support_z_2 = 0.007;
    gripper_support_offset_x_2 = 0.011; gripper_support_offset_y_2 = 0.0; gripper_support_offset_z_2 = 0.07;
		
		// 1st support area
		parameter_vector << gripper_support_x_1, gripper_support_y_1, gripper_support_z_1;
		offset_vector    << gripper_support_offset_x_1, gripper_support_offset_y_1, gripper_support_offset_z_1;
		construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame_1, parameter_vector, offset_vector, 100, 10 );
		gripper_support_offset_in_gripper_frame_1 << gripper_support_offset_x_1, gripper_support_offset_y_1, gripper_support_offset_z_1,1;
		gripper_support_offset_in_arm_hand_frame_1 = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame_1;
		
		gripper_support_in_gripper_frame_1 << gripper_support_x_1, gripper_support_y_1, gripper_support_z_1,1;
		gripper_support_in_arm_hand_frame_1 = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame_1;
		
		// 2nd support area
		parameter_vector << gripper_support_x_2, gripper_support_y_2, gripper_support_z_2;
		offset_vector    << gripper_support_offset_x_2, gripper_support_offset_y_2, gripper_support_offset_z_2;
		construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame_2, parameter_vector, offset_vector, 100, 10 );
		gripper_support_offset_in_gripper_frame_2 << gripper_support_offset_x_2, gripper_support_offset_y_2, gripper_support_offset_z_2,1;
		gripper_support_offset_in_arm_hand_frame_2 = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame_2;
		
		gripper_support_in_gripper_frame_2 << gripper_support_x_2, gripper_support_y_2, gripper_support_z_2,1;
		gripper_support_in_arm_hand_frame_2 = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame_2;
  }
  // draw
  parameter_vector << gripper_support_x, gripper_support_y, gripper_support_z;
  offset_vector    << gripper_support_offset_x, gripper_support_offset_y, gripper_support_offset_z;
  construct_special_ellipsoid_point_cloud( gripper_support_point_cloud_in_gripper_frame, parameter_vector, offset_vector, 100, 10 );
	gripper_support_offset_in_gripper_frame << gripper_support_offset_x, gripper_support_offset_y, gripper_support_offset_z,1;
	gripper_support_offset_in_arm_hand_frame = gripper_wrt_arm_hand_frame_transform*gripper_support_offset_in_gripper_frame;
  
	gripper_support_in_gripper_frame << gripper_support_x, gripper_support_y, gripper_support_z,1;
	gripper_support_in_arm_hand_frame = gripper_wrt_arm_hand_frame_transform*gripper_support_in_gripper_frame;
  
	
  
	
  // object plane special ellipsoid
  // define and draw object plane special ellipsoid
  object_plane_x = 0.35;       object_plane_y = 0.10;         object_plane_z = 0.35;
  object_plane_offset_x = 0.0; object_plane_offset_y = -0.10; object_plane_offset_z = 0.0;
  parameter_vector << object_plane_x, object_plane_y, object_plane_z;
  offset_vector    << object_plane_offset_x, object_plane_offset_y, object_plane_offset_z;
  construct_special_ellipsoid_point_cloud( object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, parameter_vector, offset_vector, 100, 10 );
  pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, object_plane_transform_wrt_arm_hand_frame);
  
  // gripper approximation as a set of special ellipsoids
  if(gripper_model == "allegro_right_hand"){
    gripper_x.push_back(0.018);    gripper_y.push_back(0.065);    gripper_z.push_back(0.059);    gripper_offset_x.push_back(-0.012);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.057);  // palm
    //gripper_x.push_back(0.017);    gripper_y.push_back(0.065);    gripper_z.push_back(0.059);    gripper_offset_x.push_back(-0.017);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.057);  // palm
    gripper_x.push_back(0.018);    gripper_y.push_back(0.020);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.012);    gripper_offset_y.push_back(0.05);    gripper_offset_z.push_back(0.17); // index
    gripper_x.push_back(0.018);    gripper_y.push_back(0.020);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.012);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17); // middle
    gripper_x.push_back(0.018);    gripper_y.push_back(0.020);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.012);    gripper_offset_y.push_back(-0.05);   gripper_offset_z.push_back(0.17); // pinky
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.070);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17);   // index, middle, pinky
    gripper_x.push_back(0.014);    gripper_y.push_back(0.08);     gripper_z.push_back(0.012);    gripper_offset_x.push_back(-0.022);    gripper_offset_y.push_back(0.11);    gripper_offset_z.push_back(0.022);  // thumb
    //gripper_x.push_back(0.028);    gripper_y.push_back(0.06);     gripper_z.push_back(0.045);    gripper_offset_x.push_back(0.03);    gripper_offset_y.push_back(0.07);    gripper_offset_z.push_back(0.04);  // thumb
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
  dummy_cloud_xyz->clear();
  for(unsigned int j=0; j<gripper_x.size(); j++){
    parameter_vector << gripper_x[j], gripper_y[j], gripper_z[j];
    offset_vector    << gripper_offset_x[j], gripper_offset_y[j], gripper_offset_z[j];
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 100, 10 );
    *gripper_as_set_of_special_ellipsoids_in_gripper_frame += *dummy_cloud_xyz;
  }
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_gripper_frame, *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, gripper_wrt_arm_hand_frame_transform);
	pcl::transformPointCloud(*gripper_support_point_cloud_in_gripper_frame, *gripper_support_point_cloud_in_arm_hand_frame, gripper_wrt_arm_hand_frame_transform);
	
	pcl::transformPointCloud(*gripper_support_point_cloud_in_gripper_frame_1, *gripper_support_point_cloud_in_arm_hand_frame_1, gripper_wrt_arm_hand_frame_transform);
	pcl::transformPointCloud(*gripper_support_point_cloud_in_gripper_frame_2, *gripper_support_point_cloud_in_arm_hand_frame_2, gripper_wrt_arm_hand_frame_transform);
	pcl::transformPointCloud(*gripper_support_point_cloud_in_gripper_frame_3, *gripper_support_point_cloud_in_arm_hand_frame_3, gripper_wrt_arm_hand_frame_transform);
	pcl::transformPointCloud(*gripper_support_point_cloud_in_gripper_frame_4, *gripper_support_point_cloud_in_arm_hand_frame_4, gripper_wrt_arm_hand_frame_transform);
}


void object_plane_pose_check(void){
  // Fixing object plane transformation problem !
  // check if object point cloud collides with object plane special ellipsoid
  object_cloud_temp->clear();
  pcl::transformPointCloud(*object_cloud_downsampled_in_arm_hand_frame_xyz, *object_cloud_temp, object_plane_transform_wrt_arm_hand_frame_inverse);
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
		construct_special_ellipsoid_point_cloud( object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, parameter_vector, offset_vector, 100, 10 );
		pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, object_plane_transform_wrt_arm_hand_frame);
  }
}


void load_allegro_right_hand_workspace_spheres( pcl::PointCloud<pcl::PointXYZ>::Ptr& gripper_augmented_workspace_xyz, 
                                                Eigen::Vector3f& gripper_workspace_centroid_point_in_gripper_frame,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& thumb_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& thumb_workspace_convex_offset, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& index_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& index_workspace_convex_offset, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& middle_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& middle_workspace_convex_offset, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& pinky_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& pinky_workspace_convex_offset ){
  gripper_augmented_workspace_xyz    ->clear();
  thumb_workspace_convex_parameter   ->clear();
  thumb_workspace_convex_offset      ->clear();
  index_workspace_convex_parameter   ->clear();
  index_workspace_convex_offset      ->clear();
  middle_workspace_convex_parameter  ->clear();
  middle_workspace_convex_offset     ->clear();
  pinky_workspace_convex_parameter   ->clear();
  pinky_workspace_convex_offset      ->clear();
  
  // declarations for allegro hand
  pcl::PointCloud<pcl::PointXYZ>::Ptr thumb_workspace_convex_xyz        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr index_workspace_convex_xyz        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr middle_workspace_convex_xyz       (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pinky_workspace_convex_xyz        (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr  dummy_cloud_xyz                  (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ> workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> workspace_convex_parameter;
  
  pcl::PointXYZ offset;
  pcl::PointXYZ parameter;
  std::vector<std::string> finger_list;
  
  Eigen::Vector3f parameter_vector, offset_vector;
  
  
  
  
  ifstream convex_workspace_file;
  finger_list.push_back("thumb");  finger_list.push_back("index");  finger_list.push_back("middle");  finger_list.push_back("pinky");
  for(int i=0; i<finger_list.size(); i++){
    std::vector<double> data;
    std::string line;
    //ifstream convex_workspace_file("../finger_workspace_spheres/"+finger_list[i]+"_workspace_spheres_3.txt");
    if(finger_list[i] == "thumb"){
      convex_workspace_file.open("../finger_workspace_spheres/"+finger_list[i]+"_workspace_spheres_15_1.7.txt");
			//convex_workspace_file.open("../finger_workspace_spheres/"+finger_list[i]+"_workspace_spheres_10_2.0.txt");
    }
		else{
      convex_workspace_file.open("../finger_workspace_spheres/"+finger_list[i]+"_workspace_spheres_15_1.txt");
			//convex_workspace_file.open("../finger_workspace_spheres/"+finger_list[i]+"_workspace_spheres_10_1.txt");
    }
		int line_count = 0;
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
        
        if(line_count==0){}
        else{
          parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
                  
          if(finger_list[i]=="thumb"){
            thumb_workspace_convex_offset->points.push_back( offset );
            thumb_workspace_convex_parameter->points.push_back( parameter );}
          else if(finger_list[i]=="index"){
            index_workspace_convex_offset->points.push_back( offset );
            index_workspace_convex_parameter->points.push_back( parameter );}
          else if(finger_list[i]=="middle"){
            middle_workspace_convex_offset->points.push_back( offset );
            middle_workspace_convex_parameter->points.push_back( parameter );}
          else if(finger_list[i]=="pinky"){
            pinky_workspace_convex_offset->points.push_back( offset );
            pinky_workspace_convex_parameter->points.push_back( parameter );}
        }
        line_count++;
      }
    }
    convex_workspace_file.close();
  }
      
  // generate/draw the point cloud of the workspace spheres
  for(int i=0; i<finger_list.size(); i++){
    for(unsigned int j=0; j<thumb_workspace_convex_offset->size(); j++){
      if(finger_list[i]=="thumb"){
        parameter_vector << thumb_workspace_convex_parameter->points[j].x, thumb_workspace_convex_parameter->points[j].y, thumb_workspace_convex_parameter->points[j].z;
        offset_vector    << thumb_workspace_convex_offset->points[j].x, thumb_workspace_convex_offset->points[j].y, thumb_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 30, 2 );
        *thumb_workspace_convex_xyz += *dummy_cloud_xyz;
      }
      else if(finger_list[i]=="index"){
        parameter_vector << index_workspace_convex_parameter->points[j].x, index_workspace_convex_parameter->points[j].y, index_workspace_convex_parameter->points[j].z;
        offset_vector    << index_workspace_convex_offset->points[j].x, index_workspace_convex_offset->points[j].y, index_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 30, 2 );
        *index_workspace_convex_xyz += *dummy_cloud_xyz;
      }
      else if(finger_list[i]=="middle"){
        parameter_vector << middle_workspace_convex_parameter->points[j].x, middle_workspace_convex_parameter->points[j].y, middle_workspace_convex_parameter->points[j].z;
        offset_vector    << middle_workspace_convex_offset->points[j].x, middle_workspace_convex_offset->points[j].y, middle_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 30, 2 );
        *middle_workspace_convex_xyz += *dummy_cloud_xyz;
      }
      else if(finger_list[i]=="pinky"){
        parameter_vector << pinky_workspace_convex_parameter->points[j].x, pinky_workspace_convex_parameter->points[j].y, pinky_workspace_convex_parameter->points[j].z;
        offset_vector    << pinky_workspace_convex_offset->points[j].x, pinky_workspace_convex_offset->points[j].y, pinky_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 30, 2 );
        *pinky_workspace_convex_xyz += *dummy_cloud_xyz;
      }
    }
  }
  *gripper_augmented_workspace_xyz = *thumb_workspace_convex_xyz;
  *gripper_augmented_workspace_xyz += *index_workspace_convex_xyz;
  *gripper_augmented_workspace_xyz += *middle_workspace_convex_xyz;
  *gripper_augmented_workspace_xyz += *pinky_workspace_convex_xyz;
  
  
  
  
  
  // gripper augmented workspace centroid location
  pcl::CentroidPoint<pcl::PointXYZ> gripper_workspace_centroid;
  pcl::PointXYZ gripper_workspace_centroid_point_in_gripper_frame_xyz;
  for(unsigned int i=0;i<gripper_augmented_workspace_xyz->points.size();i++)
    gripper_workspace_centroid.add( gripper_augmented_workspace_xyz->points[i] );
  gripper_workspace_centroid.get(gripper_workspace_centroid_point_in_gripper_frame_xyz);
  gripper_workspace_centroid_point_in_gripper_frame << gripper_workspace_centroid_point_in_gripper_frame_xyz.x, gripper_workspace_centroid_point_in_gripper_frame_xyz.y, gripper_workspace_centroid_point_in_gripper_frame_xyz.z;
  
  
  
  
  
  // filtering thumb workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<thumb_workspace_convex_offset->points.size();i++){
    if(thumb_workspace_convex_offset->points[i].z < gripper_workspace_centroid_point_in_gripper_frame_xyz.z){
      workspace_convex_offset.points.push_back( thumb_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( thumb_workspace_convex_parameter->points[i] );
    }
  }
  thumb_workspace_convex_offset->clear();
  thumb_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    thumb_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    thumb_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  // filtering index workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<index_workspace_convex_offset->points.size();i++){
    if(index_workspace_convex_offset->points[i].z > gripper_workspace_centroid_point_in_gripper_frame_xyz.z){
      workspace_convex_offset.points.push_back( index_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( index_workspace_convex_parameter->points[i] );
    }
  }
  index_workspace_convex_offset->clear();
  index_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    index_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    index_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  // filtering middle workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<middle_workspace_convex_offset->points.size();i++){
    if(middle_workspace_convex_offset->points[i].z > gripper_workspace_centroid_point_in_gripper_frame_xyz.z){
      workspace_convex_offset.points.push_back( middle_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( middle_workspace_convex_parameter->points[i] );
    }
  }
  middle_workspace_convex_offset->clear();
  middle_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    middle_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    middle_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  // filtering pinky workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<pinky_workspace_convex_offset->points.size();i++){
    if(pinky_workspace_convex_offset->points[i].z > gripper_workspace_centroid_point_in_gripper_frame_xyz.z){
      workspace_convex_offset.points.push_back( pinky_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( pinky_workspace_convex_parameter->points[i] );
    }
  }
  pinky_workspace_convex_offset->clear();
  pinky_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    pinky_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    pinky_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  
  
}


void load_franka_gripper_workspace_spheres( pcl::PointCloud<pcl::PointXYZ>::Ptr& gripper_augmented_workspace_xyz, 
                                            Eigen::Vector3f& gripper_workspace_centroid_point_in_gripper_frame,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& right_finger_workspace_convex_parameter,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& right_finger_workspace_convex_offset, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& left_finger_workspace_convex_parameter,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& left_finger_workspace_convex_offset ){
  gripper_augmented_workspace_xyz         ->clear();
  right_finger_workspace_convex_parameter ->clear();
  right_finger_workspace_convex_offset    ->clear();
  left_finger_workspace_convex_parameter  ->clear();
  left_finger_workspace_convex_offset     ->clear();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr right_finger_workspace_convex_xyz     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr left_finger_workspace_convex_xyz      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_cloud_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointXYZ offset;
  pcl::PointXYZ parameter;
  std::vector<std::string> finger_list;
  Eigen::Vector3f parameter_vector, offset_vector;
  
  
  finger_list.push_back("franka_right_finger");    finger_list.push_back("franka_left_finger");
    
    for(int i=0; i<finger_list.size(); i++){
      std::vector<double> data;
      std::string line;
      ifstream convex_workspace_file("../finger_workspace_spheres/"+finger_list[i]+"_workspace_spheres.txt");
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
          parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
                  
          if(finger_list[i]=="franka_right_finger"){
            right_finger_workspace_convex_offset->points.push_back( offset );
            right_finger_workspace_convex_parameter->points.push_back( parameter );}
          else if(finger_list[i]=="franka_left_finger"){
            left_finger_workspace_convex_offset->points.push_back( offset );
            left_finger_workspace_convex_parameter->points.push_back( parameter );}
        }
      }
      convex_workspace_file.close();
    }
    
    // generate/draw the point cloud of the workspace spheres
    for(int i=0; i<finger_list.size(); i++){
      for(unsigned int j=0; j<right_finger_workspace_convex_offset->size(); j++){
        if(finger_list[i]=="franka_right_finger"){
          parameter_vector << right_finger_workspace_convex_parameter->points[j].x, right_finger_workspace_convex_parameter->points[j].y, right_finger_workspace_convex_parameter->points[j].z;
          offset_vector    << right_finger_workspace_convex_offset->points[j].x, right_finger_workspace_convex_offset->points[j].y, right_finger_workspace_convex_offset->points[j].z;
          construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 30, 2 );
          *right_finger_workspace_convex_xyz += *dummy_cloud_xyz;
        }
        else if(finger_list[i]=="franka_left_finger"){
          parameter_vector << left_finger_workspace_convex_parameter->points[j].x, left_finger_workspace_convex_parameter->points[j].y, left_finger_workspace_convex_parameter->points[j].z;
          offset_vector    << left_finger_workspace_convex_offset->points[j].x, left_finger_workspace_convex_offset->points[j].y, left_finger_workspace_convex_offset->points[j].z;
          construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 30, 2 );
          *left_finger_workspace_convex_xyz += *dummy_cloud_xyz;
        }
      }
    }
    *gripper_augmented_workspace_xyz = *right_finger_workspace_convex_xyz;
    *gripper_augmented_workspace_xyz += *left_finger_workspace_convex_xyz;
  
  
  
  
  
  // gripper augmented workspace centroid location
  pcl::CentroidPoint<pcl::PointXYZ> gripper_workspace_centroid;
  pcl::PointXYZ gripper_workspace_centroid_point_in_gripper_frame_xyz;
  for(unsigned int i=0;i<gripper_augmented_workspace_xyz->points.size();i++)
    gripper_workspace_centroid.add( gripper_augmented_workspace_xyz->points[i] );
  gripper_workspace_centroid.get(gripper_workspace_centroid_point_in_gripper_frame_xyz);
  gripper_workspace_centroid_point_in_gripper_frame << gripper_workspace_centroid_point_in_gripper_frame_xyz.x, gripper_workspace_centroid_point_in_gripper_frame_xyz.y, gripper_workspace_centroid_point_in_gripper_frame_xyz.z;
  
}


void load_gripper_workspace_spheres_and_compute_centroid(void){
  // load gripper workspace spheres and compute the centroid in the gripper frame
  if(gripper_model == "allegro_right_hand"){
    load_allegro_right_hand_workspace_spheres( gripper_augmented_workspace_xyz,     gripper_workspace_centroid_point_in_gripper_frame,
                                               thumb_workspace_convex_parameter,    thumb_workspace_convex_offset, 
                                               index_workspace_convex_parameter,    index_workspace_convex_offset, 
                                               middle_workspace_convex_parameter,   middle_workspace_convex_offset, 
                                               pinky_workspace_convex_parameter,    pinky_workspace_convex_offset );
  }
  else if(gripper_model == "franka_gripper"){
    load_franka_gripper_workspace_spheres( gripper_augmented_workspace_xyz,         gripper_workspace_centroid_point_in_gripper_frame,
                                           right_finger_workspace_convex_parameter, right_finger_workspace_convex_offset, 
                                           left_finger_workspace_convex_parameter,  left_finger_workspace_convex_offset );
  }
}


void metric_1_number_of_active_workspace_spheres_and_corresponding_object_points( pcl::PointCloud<pcl::PointXYZ>::Ptr&     object_cloud_transformed_in_gripper_frame_xyz, 
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_convex_offset,
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_convex_parameter, 
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     object_points_in_finger_workspace,
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_active_spheres_offset, 
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_active_spheres_parameter ){
  int counter;
  double sphere_value;
  // Evaluation Metric#1 : number of object points && number of workspace spheres
  object_points_in_finger_workspace->clear();
  finger_workspace_active_spheres_offset->clear();
  finger_workspace_active_spheres_parameter->clear();
  // for each thumb workspace sphere
  for(unsigned int l=0; l<finger_workspace_convex_offset->size(); l++){
    counter = 0;
    // for each point in object downsampled cloud
    for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyz->size(); k++){
      sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - finger_workspace_convex_offset->points[l].x, 2)
                     + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - finger_workspace_convex_offset->points[l].y, 2) 
                     + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - finger_workspace_convex_offset->points[l].z, 2) )/pow(finger_workspace_convex_parameter->points[l].x, 2);
      // add all object points inside this workspace sphere
      if( sphere_value < 1 ){
        object_points_in_finger_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyz->points[k] );
        counter++;
      }
    } // end of iteration through all object points
    if(counter > 0){
      //add this sphere to set of active spheres
      finger_workspace_active_spheres_offset->points.push_back   ( finger_workspace_convex_offset    ->points[l] );
      finger_workspace_active_spheres_parameter->points.push_back( finger_workspace_convex_parameter ->points[l] );
    }    
  } // end of iteration through all finger workspace spheres
  
}


void point_cloud_as_set_of_spheres_fixed_radius( pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_cloud_xyz,            // input
                                                 double sphere_radius,                                                        // input
                                                 double overlap_distance,                                                     // input
                                                 int iterations,                                                              // input
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_spheres_filtered_xyz, // output
                                                 Eigen::Vector4f& far_point_in_pos_direction_4d,                              // output
                                                 Eigen::Vector4f& far_point_in_neg_direction_4d ){                            // output
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       finger_sphere_offset_cloud_xyz                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       dummy_cloud_xyz                               (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       poly_mesh_vertices                            (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PolygonMesh mesh_out;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  
  double value_x, value_y, value_z;
  pcl::PointXYZ point_xyz;
  double sphere_value;
  
  // centroid of workspace point cloud
  // object plane centroid location
  pcl::CentroidPoint<pcl::PointXYZ> finger_workspace_centroid;
  pcl::PointXYZ finger_workspace_centroid_point;
  for(int i=0; i<finger_workspace_cloud_xyz->points.size(); i++)
    finger_workspace_centroid.add( finger_workspace_cloud_xyz->points[i] );
  finger_workspace_centroid.get(finger_workspace_centroid_point);
  
  // get the far point in negative and positive directions
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = finger_workspace_centroid_point;
  far_point_in_neg_direction = finger_workspace_centroid_point;
  for(unsigned int i=0;i<finger_workspace_cloud_xyz->size();i++){
    if( finger_workspace_cloud_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = finger_workspace_cloud_xyz->points[i].x;
    if( finger_workspace_cloud_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = finger_workspace_cloud_xyz->points[i].x;
    
    if( finger_workspace_cloud_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = finger_workspace_cloud_xyz->points[i].y;
    if( finger_workspace_cloud_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = finger_workspace_cloud_xyz->points[i].y;
    
    if( finger_workspace_cloud_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = finger_workspace_cloud_xyz->points[i].z;
    if( finger_workspace_cloud_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = finger_workspace_cloud_xyz->points[i].z;
  }
  far_point_in_pos_direction_4d << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_4d << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  pcl::PointXYZ finger_workspace_lattice_dimensions;
  finger_workspace_lattice_dimensions.x = (far_point_in_pos_direction.x - far_point_in_neg_direction.x);
  finger_workspace_lattice_dimensions.y = (far_point_in_pos_direction.y - far_point_in_neg_direction.y);
  finger_workspace_lattice_dimensions.z = (far_point_in_pos_direction.z - far_point_in_neg_direction.z);
  
  // generating the concave hull
  concave_hull.setInputCloud( finger_workspace_cloud_xyz );
  concave_hull.setAlpha( 0.01f );
  concave_hull.reconstruct( mesh_out );
  
  // getting the vertices of the polygon mesh generated
  pcl::fromPCLPointCloud2(mesh_out.cloud, *poly_mesh_vertices);
  
  // shifting in x, y, and z-axes to find the biggest fitting spheres
  for(int l=0; l<iterations; l++){
    for(int m=0; m<iterations; m++){
      for(int n=0; n<iterations; n++){
        // construct the sphere lattice
        finger_sphere_offset_cloud_xyz->clear();
        int number_of_spheres_in_x = finger_workspace_lattice_dimensions.x/(2*sphere_radius) +1;
        int number_of_spheres_in_y = finger_workspace_lattice_dimensions.y/(2*sphere_radius) +1;
        int number_of_spheres_in_z = finger_workspace_lattice_dimensions.z/(2*sphere_radius) +1;
        for(int i=0; i<number_of_spheres_in_x; i++){
          for(int j=0; j<number_of_spheres_in_y; j++){
            for(int k=0; k<number_of_spheres_in_z; k++){
              point_xyz.x = far_point_in_neg_direction.x+sphere_radius*(l+1)/iterations + i*2*sphere_radius;
              point_xyz.y = far_point_in_neg_direction.y+sphere_radius*(m+1)/iterations + j*2*sphere_radius;
              point_xyz.z = far_point_in_neg_direction.z+sphere_radius*(n+1)/iterations + k*2*sphere_radius;
              finger_sphere_offset_cloud_xyz->points.push_back( point_xyz );
            }
          }
        }
        
        // filtering the sphere lattice (#1)
        finger_workspace_spheres_filtered_xyz->clear();
        for(int j=0; j<finger_sphere_offset_cloud_xyz->size(); j++){
          for(int k=0; k<finger_workspace_cloud_xyz->size(); k++){
            sphere_value = ( pow(finger_workspace_cloud_xyz->points[k].x - finger_sphere_offset_cloud_xyz->points[j].x, 2)
                           + pow(finger_workspace_cloud_xyz->points[k].y - finger_sphere_offset_cloud_xyz->points[j].y, 2) 
                           + pow(finger_workspace_cloud_xyz->points[k].z - finger_sphere_offset_cloud_xyz->points[j].z, 2) )/pow(sphere_radius, 2);
            if(sphere_value <= 1){
              finger_workspace_spheres_filtered_xyz->points.push_back( finger_sphere_offset_cloud_xyz->points[j] );
              break;
            }
          }
        }
        
        // filtering the sphere lattice (#2)
        //dummy_cloud_xyz->clear();
        int number_of_mesh_points_inside_this_sphere;
        for(int j=0; j<finger_workspace_spheres_filtered_xyz->size(); j++){
          number_of_mesh_points_inside_this_sphere = 0;
          for(int k=0; k<poly_mesh_vertices->size(); k++){
            sphere_value = ( pow(poly_mesh_vertices->points[k].x - finger_workspace_spheres_filtered_xyz->points[j].x, 2)
                           + pow(poly_mesh_vertices->points[k].y - finger_workspace_spheres_filtered_xyz->points[j].y, 2) 
                           + pow(poly_mesh_vertices->points[k].z - finger_workspace_spheres_filtered_xyz->points[j].z, 2) )/pow(sphere_radius, 2);
            if(sphere_value < 1)
              number_of_mesh_points_inside_this_sphere++;
          }
          if(number_of_mesh_points_inside_this_sphere == 0)
            dummy_cloud_xyz->points.push_back( finger_workspace_spheres_filtered_xyz->points[j] );
        }
        
      }
    }
  }
  *finger_workspace_spheres_filtered_xyz = *dummy_cloud_xyz;
  
  // filtering (#3)
  // having all these spheres, we check the center of each sphere if it exists in another fellow sphere
  // if it does we remove it
  std::vector<double> sphere_offset_vector_x, sphere_offset_vector_y, sphere_offset_vector_z;
  for(int i=0; i<finger_workspace_spheres_filtered_xyz->size(); i++){
    sphere_offset_vector_x.push_back( finger_workspace_spheres_filtered_xyz->points[i].x );
    sphere_offset_vector_y.push_back( finger_workspace_spheres_filtered_xyz->points[i].y );
    sphere_offset_vector_z.push_back( finger_workspace_spheres_filtered_xyz->points[i].z );
  }
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    for(int j=sphere_offset_vector_x.size()-1; j>-1; j--){
      sphere_value = ( pow(sphere_offset_vector_x[j] - sphere_offset_vector_x[i], 2)
                     + pow(sphere_offset_vector_y[j] - sphere_offset_vector_y[i], 2) 
                     + pow(sphere_offset_vector_z[j] - sphere_offset_vector_z[i], 2) )/pow(sphere_radius, 2);
      // the center of this sphere is on the border of another sphere
      if(sphere_value < overlap_distance and i!=j){
        sphere_offset_vector_x.erase(sphere_offset_vector_x.begin()+j);
        sphere_offset_vector_y.erase(sphere_offset_vector_y.begin()+j);
        sphere_offset_vector_z.erase(sphere_offset_vector_z.begin()+j);
      }
    }
  }
  
  finger_workspace_spheres_filtered_xyz->clear();
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    point_xyz.x = sphere_offset_vector_x[i];
    point_xyz.y = sphere_offset_vector_y[i];
    point_xyz.z = sphere_offset_vector_z[i];
    finger_workspace_spheres_filtered_xyz->points.push_back( point_xyz );
  }
  
  /*
  // get the index with highest number of spheres
  std::vector<int> number_of_filling_spheres;
  number_of_filling_spheres.push_back( finger_workspace_spheres_filtered_xyz->size() );
  std::vector<int>::iterator it = std::find(number_of_filling_spheres.begin(), number_of_filling_spheres.end(), *std::max_element(number_of_filling_spheres.begin(), number_of_filling_spheres.end()) );
  int index_of_greatest = std::distance(number_of_filling_spheres.begin(), it);
  //std::cout <<  "index = " << index_of_greatest << std::endl;
  //std::cout <<  "value = " << number_of_filling_spheres[index_of_greatest] << std::endl;
  */
  
  
}


void point_cloud_as_set_of_spheres( int desired_number_of_spheres,                                                       // input
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_cloud_xyz,                    // input
                                    double radius_of_smallest_sphere,                                                    // input
                                    double radius_of_largest_sphere,                                                     // input
                                    double sphere_radius_increment,                                                      // input
                                    double overlap_distance,                                                             // input
                                    int iterations,                                                                      // input
                                    int sphere_point_cloud_samples,                                                      // input
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_spheres_filtered_xyz,         // output
                                    std::vector<double>& sphere_radius_filtered,                                         // output
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_spheres_visualization_xyz,    // output
                                    Eigen::Vector4f& far_point_in_pos_direction_4d,                                      // output
                                    Eigen::Vector4f& far_point_in_neg_direction_4d ){                                    // output
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       dummy_cloud_xyz         (new pcl::PointCloud<pcl::PointXYZ>);
  double value_x, value_y, value_z;
  pcl::PointXYZ point_xyz;
  double sphere_value;
  
  // centroid of workspace point cloud
  // object plane centroid location
  pcl::CentroidPoint<pcl::PointXYZ> finger_workspace_centroid;
  pcl::PointXYZ finger_workspace_centroid_point;
  for(int i=0; i<finger_workspace_cloud_xyz->points.size(); i++)
    finger_workspace_centroid.add( finger_workspace_cloud_xyz->points[i] );
  finger_workspace_centroid.get(finger_workspace_centroid_point);
  
  // get the far point in negative and positive directions
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = finger_workspace_centroid_point;
  far_point_in_neg_direction = finger_workspace_centroid_point;
  for(unsigned int i=0;i<finger_workspace_cloud_xyz->size();i++){
    if( finger_workspace_cloud_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = finger_workspace_cloud_xyz->points[i].x;
    if( finger_workspace_cloud_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = finger_workspace_cloud_xyz->points[i].x;
    
    if( finger_workspace_cloud_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = finger_workspace_cloud_xyz->points[i].y;
    if( finger_workspace_cloud_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = finger_workspace_cloud_xyz->points[i].y;
    
    if( finger_workspace_cloud_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = finger_workspace_cloud_xyz->points[i].z;
    if( finger_workspace_cloud_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = finger_workspace_cloud_xyz->points[i].z;
  }
  far_point_in_pos_direction_4d << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_4d << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  pcl::PointXYZ finger_workspace_lattice_dimensions;
  finger_workspace_lattice_dimensions.x = (far_point_in_pos_direction.x - far_point_in_neg_direction.x);
  finger_workspace_lattice_dimensions.y = (far_point_in_pos_direction.y - far_point_in_neg_direction.y);
  finger_workspace_lattice_dimensions.z = (far_point_in_pos_direction.z - far_point_in_neg_direction.z);
  
  
  //
  std::vector<double> sphere_radius;
  std::vector<double> sphere_radius_dummy;
  //std::vector<double> sphere_radius_filtered;
  int number_of_increments = ceil((radius_of_largest_sphere - radius_of_smallest_sphere)/sphere_radius_increment);
  for(int i=0; i<number_of_increments; i++)
    sphere_radius.push_back( radius_of_largest_sphere-i*sphere_radius_increment );
  
  for(int i=0; i<sphere_radius.size(); i++){
    std::cout << "sphere_radius vector: " << sphere_radius[i] << std::endl;
    point_cloud_as_set_of_spheres_fixed_radius( finger_workspace_cloud_xyz, sphere_radius[i], overlap_distance, iterations, dummy_cloud_xyz, far_point_in_pos_direction_4d, far_point_in_neg_direction_4d );
    std::cout << "number of spheres for this radius value: " << dummy_cloud_xyz->size() << std::endl;
    *finger_workspace_spheres_filtered_xyz += *dummy_cloud_xyz;
    for(int j=0; j<dummy_cloud_xyz->size(); j++)
      sphere_radius_dummy.push_back(sphere_radius[i]);
  }
  
  // filtering
  // having all these spheres, we check the center of each sphere if it exists in another fellow sphere
  // if it does we remove it, we give priority to bigger spheres
  std::vector<double> sphere_offset_vector_x, sphere_offset_vector_y, sphere_offset_vector_z;
  for(int i=0; i<finger_workspace_spheres_filtered_xyz->size(); i++){
    sphere_offset_vector_x.push_back( finger_workspace_spheres_filtered_xyz->points[i].x );
    sphere_offset_vector_y.push_back( finger_workspace_spheres_filtered_xyz->points[i].y );
    sphere_offset_vector_z.push_back( finger_workspace_spheres_filtered_xyz->points[i].z );
  }
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    for(int j=sphere_offset_vector_x.size()-1; j>-1; j--){
      sphere_value = ( pow(sphere_offset_vector_x[j] - sphere_offset_vector_x[i], 2)
                     + pow(sphere_offset_vector_y[j] - sphere_offset_vector_y[i], 2) 
                     + pow(sphere_offset_vector_z[j] - sphere_offset_vector_z[i], 2) )/pow(sphere_radius_dummy[i], 2);
      // the center of this sphere is on the border of another sphere
      if(sphere_value < overlap_distance and i!=j){
        sphere_offset_vector_x.erase(sphere_offset_vector_x.begin()+j);
        sphere_offset_vector_y.erase(sphere_offset_vector_y.begin()+j);
        sphere_offset_vector_z.erase(sphere_offset_vector_z.begin()+j);
        sphere_radius_dummy.erase(sphere_radius_dummy.begin()+j);
      }
    }
  }
  
  //
  finger_workspace_spheres_filtered_xyz->clear();
  if( desired_number_of_spheres < sphere_offset_vector_x.size() )
    for(int i=0; i<desired_number_of_spheres; i++){
      point_xyz.x = sphere_offset_vector_x[i];
      point_xyz.y = sphere_offset_vector_y[i];
      point_xyz.z = sphere_offset_vector_z[i];
      finger_workspace_spheres_filtered_xyz->points.push_back( point_xyz );
      sphere_radius_filtered.push_back( sphere_radius_dummy[i] );
    }
  else{
    for(int i=0; i<sphere_offset_vector_x.size(); i++){
      point_xyz.x = sphere_offset_vector_x[i];
      point_xyz.y = sphere_offset_vector_y[i];
      point_xyz.z = sphere_offset_vector_z[i];
      finger_workspace_spheres_filtered_xyz->points.push_back( point_xyz );
      sphere_radius_filtered.push_back( sphere_radius_dummy[i] );
    }
  }
  
  // generate and view the point cloud of the sphere
  for(int j=0; j<finger_workspace_spheres_filtered_xyz->size(); j++){
    for(int k=0; k<sphere_point_cloud_samples; k++){
      value_x = (-sphere_radius_filtered[j]+finger_workspace_spheres_filtered_xyz->points[j].x) + k*2*sphere_radius_filtered[j]/sphere_point_cloud_samples;
      for(int l=0; l<sphere_point_cloud_samples; l++){
        value_y = (-sphere_radius_filtered[j]+finger_workspace_spheres_filtered_xyz->points[j].y) + l*2*sphere_radius_filtered[j]/sphere_point_cloud_samples;
        value_z = finger_workspace_spheres_filtered_xyz->points[j].z + sphere_radius_filtered[j]*sqrt( 1 - pow(value_x-finger_workspace_spheres_filtered_xyz->points[j].x, 2)/pow(sphere_radius_filtered[j], 2) - pow(value_y-finger_workspace_spheres_filtered_xyz->points[j].y, 2)/pow(sphere_radius_filtered[j], 2) );
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        finger_workspace_spheres_visualization_xyz->points.push_back( point_xyz );
        
        value_z = finger_workspace_spheres_filtered_xyz->points[j].z - sphere_radius_filtered[j]*sqrt( 1 - pow(value_x-finger_workspace_spheres_filtered_xyz->points[j].x, 2)/pow(sphere_radius_filtered[j], 2) - pow(value_y-finger_workspace_spheres_filtered_xyz->points[j].y, 2)/pow(sphere_radius_filtered[j], 2) );
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        
        finger_workspace_spheres_visualization_xyz->points.push_back( point_xyz );          
      }
    }
  }
  
  
  
}


void point_cloud_as_set_of_spheres_fixed_radius_paper_photos(  pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_cloud_xyz,            // input
                                                       double sphere_radius,                                                                // input
                                                       double overlap_distance,                                                             //input
                                                       int iterations,                                                                      // input
                                                       int sphere_point_cloud_samples,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_spheres_filtered_xyz,         // output
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr&  finger_workspace_spheres_visualization_xyz,    // output
                                                       Eigen::Vector4f& far_point_in_pos_direction_4d,                                      // output
                                                       Eigen::Vector4f& far_point_in_neg_direction_4d ){                                    // output
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       finger_sphere_offset_cloud_xyz                (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       dummy_cloud_xyz                               (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       poly_mesh_vertices                            (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PolygonMesh mesh_out;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  
  double value_x, value_y, value_z;
  pcl::PointXYZ point_xyz;
  double sphere_value;
  
  // centroid of workspace point cloud
  // object plane centroid location
  pcl::CentroidPoint<pcl::PointXYZ> finger_workspace_centroid;
  pcl::PointXYZ finger_workspace_centroid_point;
  for(int i=0; i<finger_workspace_cloud_xyz->points.size(); i++)
    finger_workspace_centroid.add( finger_workspace_cloud_xyz->points[i] );
  finger_workspace_centroid.get(finger_workspace_centroid_point);
  
  // get the far point in negative and positive directions
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = finger_workspace_centroid_point;
  far_point_in_neg_direction = finger_workspace_centroid_point;
  for(unsigned int i=0;i<finger_workspace_cloud_xyz->size();i++){
    if( finger_workspace_cloud_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = finger_workspace_cloud_xyz->points[i].x;
    if( finger_workspace_cloud_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = finger_workspace_cloud_xyz->points[i].x;
    
    if( finger_workspace_cloud_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = finger_workspace_cloud_xyz->points[i].y;
    if( finger_workspace_cloud_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = finger_workspace_cloud_xyz->points[i].y;
    
    if( finger_workspace_cloud_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = finger_workspace_cloud_xyz->points[i].z;
    if( finger_workspace_cloud_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = finger_workspace_cloud_xyz->points[i].z;
  }
  far_point_in_pos_direction_4d << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_4d << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  pcl::PointXYZ finger_workspace_lattice_dimensions;
  finger_workspace_lattice_dimensions.x = (far_point_in_pos_direction.x - far_point_in_neg_direction.x);
  finger_workspace_lattice_dimensions.y = (far_point_in_pos_direction.y - far_point_in_neg_direction.y);
  finger_workspace_lattice_dimensions.z = (far_point_in_pos_direction.z - far_point_in_neg_direction.z);
  
  // generating the concave hull
  concave_hull.setInputCloud( finger_workspace_cloud_xyz );
  concave_hull.setAlpha( 0.01f );
  concave_hull.reconstruct( mesh_out );
  
  // getting the vertices of the polygon mesh generated
  pcl::fromPCLPointCloud2(mesh_out.cloud, *poly_mesh_vertices);
  
  // shifting in x, y, and z-axes to find the biggest fitting spheres
  for(int l=0; l<iterations; l++){
    for(int m=0; m<iterations; m++){
      for(int n=0; n<iterations; n++){
        // construct the sphere lattice
        finger_sphere_offset_cloud_xyz->clear();
        int number_of_spheres_in_x = finger_workspace_lattice_dimensions.x/(2*sphere_radius) +1;
        int number_of_spheres_in_y = finger_workspace_lattice_dimensions.y/(2*sphere_radius) +1;
        int number_of_spheres_in_z = finger_workspace_lattice_dimensions.z/(2*sphere_radius) +1;
        for(int i=0; i<number_of_spheres_in_x; i++){
          for(int j=0; j<number_of_spheres_in_y; j++){
            for(int k=0; k<number_of_spheres_in_z; k++){
              point_xyz.x = far_point_in_neg_direction.x+sphere_radius*(l+1)/iterations + i*2*sphere_radius;
              point_xyz.y = far_point_in_neg_direction.y+sphere_radius*(m+1)/iterations + j*2*sphere_radius;
              point_xyz.z = far_point_in_neg_direction.z+sphere_radius*(n+1)/iterations + k*2*sphere_radius;
              finger_sphere_offset_cloud_xyz->points.push_back( point_xyz );
            }
          }
        }
        
        // filtering the sphere lattice (#1)
        finger_workspace_spheres_filtered_xyz->clear();
        for(int j=0; j<finger_sphere_offset_cloud_xyz->size(); j++){
          for(int k=0; k<finger_workspace_cloud_xyz->size(); k++){
            sphere_value = ( pow(finger_workspace_cloud_xyz->points[k].x - finger_sphere_offset_cloud_xyz->points[j].x, 2)
                           + pow(finger_workspace_cloud_xyz->points[k].y - finger_sphere_offset_cloud_xyz->points[j].y, 2) 
                           + pow(finger_workspace_cloud_xyz->points[k].z - finger_sphere_offset_cloud_xyz->points[j].z, 2) )/pow(sphere_radius, 2);
            if(sphere_value <= 1){
              finger_workspace_spheres_filtered_xyz->points.push_back( finger_sphere_offset_cloud_xyz->points[j] );
              break;
            }
          }
        }
        
        // filtering the sphere lattice (#2)
        //dummy_cloud_xyz->clear();
        int number_of_mesh_points_inside_this_sphere;
        for(int j=0; j<finger_workspace_spheres_filtered_xyz->size(); j++){
          number_of_mesh_points_inside_this_sphere = 0;
          for(int k=0; k<poly_mesh_vertices->size(); k++){
            sphere_value = ( pow(poly_mesh_vertices->points[k].x - finger_workspace_spheres_filtered_xyz->points[j].x, 2)
                           + pow(poly_mesh_vertices->points[k].y - finger_workspace_spheres_filtered_xyz->points[j].y, 2) 
                           + pow(poly_mesh_vertices->points[k].z - finger_workspace_spheres_filtered_xyz->points[j].z, 2) )/pow(sphere_radius, 2);
            if(sphere_value < 1)
              number_of_mesh_points_inside_this_sphere++;
          }
          if(number_of_mesh_points_inside_this_sphere == 0)
            dummy_cloud_xyz->points.push_back( finger_workspace_spheres_filtered_xyz->points[j] );
        }
        
      }
    }
  }
  *finger_workspace_spheres_filtered_xyz = *dummy_cloud_xyz;
  //*finger_workspace_spheres_filtered_xyz = *finger_sphere_offset_cloud_xyz;
  
  
  // filtering (#3)
  // having all these spheres, we check the center of each sphere if it exists in another fellow sphere
  // if it does we remove it
  std::vector<double> sphere_offset_vector_x, sphere_offset_vector_y, sphere_offset_vector_z;
  for(int i=0; i<finger_workspace_spheres_filtered_xyz->size(); i++){
    sphere_offset_vector_x.push_back( finger_workspace_spheres_filtered_xyz->points[i].x );
    sphere_offset_vector_y.push_back( finger_workspace_spheres_filtered_xyz->points[i].y );
    sphere_offset_vector_z.push_back( finger_workspace_spheres_filtered_xyz->points[i].z );
  }
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    for(int j=sphere_offset_vector_x.size()-1; j>-1; j--){
      sphere_value = ( pow(sphere_offset_vector_x[j] - sphere_offset_vector_x[i], 2)
                     + pow(sphere_offset_vector_y[j] - sphere_offset_vector_y[i], 2) 
                     + pow(sphere_offset_vector_z[j] - sphere_offset_vector_z[i], 2) )/pow(sphere_radius, 2);
      // the center of this sphere is on the border of another sphere
      if(sphere_value < overlap_distance and i!=j){
        sphere_offset_vector_x.erase(sphere_offset_vector_x.begin()+j);
        sphere_offset_vector_y.erase(sphere_offset_vector_y.begin()+j);
        sphere_offset_vector_z.erase(sphere_offset_vector_z.begin()+j);
      }
    }
  }
  
  finger_workspace_spheres_filtered_xyz->clear();
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    point_xyz.x = sphere_offset_vector_x[i];
    point_xyz.y = sphere_offset_vector_y[i];
    point_xyz.z = sphere_offset_vector_z[i];
    finger_workspace_spheres_filtered_xyz->points.push_back( point_xyz );
  }
  
  
  
  
  
  
  
  // generate and view the point cloud of the sphere
  for(int j=0; j<finger_workspace_spheres_filtered_xyz->size(); j++){
    for(int k=0; k<sphere_point_cloud_samples; k++){
      value_x = (-sphere_radius+finger_workspace_spheres_filtered_xyz->points[j].x) + k*2*sphere_radius/sphere_point_cloud_samples;
      for(int l=0; l<sphere_point_cloud_samples; l++){
        value_y = (-sphere_radius+finger_workspace_spheres_filtered_xyz->points[j].y) + l*2*sphere_radius/sphere_point_cloud_samples;
        value_z = finger_workspace_spheres_filtered_xyz->points[j].z + sphere_radius*sqrt( 1 - pow(value_x-finger_workspace_spheres_filtered_xyz->points[j].x, 2)/pow(sphere_radius, 2) - pow(value_y-finger_workspace_spheres_filtered_xyz->points[j].y, 2)/pow(sphere_radius, 2) );
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        finger_workspace_spheres_visualization_xyz->points.push_back( point_xyz );
        
        value_z = finger_workspace_spheres_filtered_xyz->points[j].z - sphere_radius*sqrt( 1 - pow(value_x-finger_workspace_spheres_filtered_xyz->points[j].x, 2)/pow(sphere_radius, 2) - pow(value_y-finger_workspace_spheres_filtered_xyz->points[j].y, 2)/pow(sphere_radius, 2) );
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        
        finger_workspace_spheres_visualization_xyz->points.push_back( point_xyz );          
      }
    }
  }
  
  
  
  
  
  
  
  
  /*
  // get the index with highest number of spheres
  std::vector<int> number_of_filling_spheres;
  number_of_filling_spheres.push_back( finger_workspace_spheres_filtered_xyz->size() );
  std::vector<int>::iterator it = std::find(number_of_filling_spheres.begin(), number_of_filling_spheres.end(), *std::max_element(number_of_filling_spheres.begin(), number_of_filling_spheres.end()) );
  int index_of_greatest = std::distance(number_of_filling_spheres.begin(), it);
  //std::cout <<  "index = " << index_of_greatest << std::endl;
  //std::cout <<  "value = " << number_of_filling_spheres[index_of_greatest] << std::endl;
  */
  
  
}


void registering_downsampling_segmenting_3_view_point_clouds( pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_1,                             // input
                                                              Eigen::Matrix4f tm1,                                                                 // input
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_2,                             // input
                                                              Eigen::Matrix4f tm2,                                                                 // input
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_3,                             // input
                                                              Eigen::Matrix4f tm3,                                                                 // input
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  table_cloud_xyz_downsampled,                   // output
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  object_cloud_xyz_downsampled ){                // output






  double leaf_size = 0.01;
  double distance_threshold = 0.01;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1_transformed_downsampled     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2_transformed_downsampled     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3_transformed_downsampled     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_transformed_downsampled       (new pcl::PointCloud<pcl::PointXYZ>);
  
  double i;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PCDWriter writer;
  
  //std::cout << "scene cloud 1 size [input] : " << scene_cloud_xyz_1->size() << std::endl;
  //std::cout << "scene cloud 2 size [input] : " << scene_cloud_xyz_2->size() << std::endl;
  //std::cout << "scene cloud 3 size [input] : " << scene_cloud_xyz_3->size() << std::endl;
  
  pcl::transformPointCloud(*scene_cloud_xyz_1, *scene_cloud_xyz_1_transformed, tm1);
  pcl::transformPointCloud(*scene_cloud_xyz_2, *scene_cloud_xyz_2_transformed, tm2);
  pcl::transformPointCloud(*scene_cloud_xyz_3, *scene_cloud_xyz_3_transformed, tm3);
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // downsampling
  vg.setInputCloud (scene_cloud_xyz_1_transformed);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*scene_cloud_xyz_1_transformed_downsampled);
  //std::cout << "PointCloud#1 after downsampling (1st pass) has: " << scene_cloud_xyz_1_transformed_downsampled->points.size() << " data points." << std::endl;
  
  vg.setInputCloud (scene_cloud_xyz_2_transformed);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*scene_cloud_xyz_2_transformed_downsampled);
  //std::cout << "PointCloud#2 after downsampling (1st pass) has: " << scene_cloud_xyz_2_transformed_downsampled->points.size() << " data points." << std::endl;
  
  vg.setInputCloud (scene_cloud_xyz_3_transformed);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*scene_cloud_xyz_3_transformed_downsampled);
  //std::cout << "PointCloud#3 after downsampling (1st pass) has: " << scene_cloud_xyz_3_transformed_downsampled->points.size() << " data points." << std::endl;
  
  *scene_cloud_xyz_transformed_downsampled =  *scene_cloud_xyz_1_transformed_downsampled;
  *scene_cloud_xyz_transformed_downsampled += *scene_cloud_xyz_2_transformed_downsampled;
  *scene_cloud_xyz_transformed_downsampled += *scene_cloud_xyz_3_transformed_downsampled;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // segmentation/clustering
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold(distance_threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  int nr_points = (int) scene_cloud_xyz_transformed_downsampled->points.size();
  while(scene_cloud_xyz_transformed_downsampled->points.size() > 0.3*nr_points){
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(scene_cloud_xyz_transformed_downsampled);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0){
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (scene_cloud_xyz_transformed_downsampled);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *scene_cloud_xyz_transformed_downsampled = *cloud_f;
  }
  // downsampling object cloud to ensure below 500 points
  i = 0.0;
  while(cloud_plane->points.size() > 500){
    i += 0.001;
    vg.setInputCloud(cloud_plane);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*cloud_plane);
  }
  //std::cout << "plane cloud segmented/downsampled has [output]: " << cloud_plane->points.size() << " data points." << std::endl;
  *table_cloud_xyz_downsampled = *cloud_plane;
  // save plane cloud
  //writer.write<pcl::PointXYZ>("table.pcd", *table_cloud_xyz_downsampled, false);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (scene_cloud_xyz_transformed_downsampled);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (scene_cloud_xyz_transformed_downsampled);
  ec.extract (cluster_indices);

  int j = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back (scene_cloud_xyz_transformed_downsampled->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    if(j==0)
      break;
    j++;
  }
  
  // downsampling object cloud to ensure below arbitrary number of points
  i = 0.0;
  while(cloud_cluster->points.size() > desired_number_of_object_cloud_points){
    i += 0.001;
    vg.setInputCloud(cloud_cluster);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*cloud_cluster);
  }
  *object_cloud_xyz_downsampled = *cloud_cluster;
  
  //std::cout << "object point cloud segmented/downsampled has [output]: " << object_cloud_xyz_downsampled->points.size() << " data points." << std::endl;
  //std::stringstream ss;
  //ss << "object_cloud.pcd";
  //writer.write<pcl::PointXYZ>(ss.str(), *object_cloud_xyz_downsampled, false);
  
  

}


void registering_downsampling_segmenting_3_view_point_clouds( pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_1,                             // input
                                                              Eigen::Matrix4f tm1,                                                                 // input
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_2,                             // input
                                                              Eigen::Matrix4f tm2,                                                                 // input
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_3,                             // input
                                                              Eigen::Matrix4f tm3,                                                                 // input
                                                              
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_1_transformed,                 // output
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_2_transformed,                 // output
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_3_transformed,                 // output
                                                              
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_1_transformed_downsampled,     // output
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_2_transformed_downsampled,     // output
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  scene_cloud_xyz_3_transformed_downsampled,     // output
                                                              
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  table_cloud_xyz_downsampled,                   // output
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr&  object_cloud_xyz_downsampled ){                // output






  double leaf_size = 0.01;
  double distance_threshold = 0.01;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_transformed_downsampled       (new pcl::PointCloud<pcl::PointXYZ>);
  
  double i;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PCDWriter writer;
  
  //std::cout << "scene cloud 1 size [input] : " << scene_cloud_xyz_1->size() << std::endl;
  //std::cout << "scene cloud 2 size [input] : " << scene_cloud_xyz_2->size() << std::endl;
  //std::cout << "scene cloud 3 size [input] : " << scene_cloud_xyz_3->size() << std::endl;
  
  pcl::transformPointCloud(*scene_cloud_xyz_1, *scene_cloud_xyz_1_transformed, tm1);
  pcl::transformPointCloud(*scene_cloud_xyz_2, *scene_cloud_xyz_2_transformed, tm2);
  pcl::transformPointCloud(*scene_cloud_xyz_3, *scene_cloud_xyz_3_transformed, tm3);
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // downsampling
  vg.setInputCloud (scene_cloud_xyz_1_transformed);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*scene_cloud_xyz_1_transformed_downsampled);
  //std::cout << "PointCloud#1 after downsampling (1st pass) has: " << scene_cloud_xyz_1_transformed_downsampled->points.size() << " data points." << std::endl;
  
  vg.setInputCloud (scene_cloud_xyz_2_transformed);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*scene_cloud_xyz_2_transformed_downsampled);
  //std::cout << "PointCloud#2 after downsampling (1st pass) has: " << scene_cloud_xyz_2_transformed_downsampled->points.size() << " data points." << std::endl;
  
  vg.setInputCloud (scene_cloud_xyz_3_transformed);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*scene_cloud_xyz_3_transformed_downsampled);
  //std::cout << "PointCloud#3 after downsampling (1st pass) has: " << scene_cloud_xyz_3_transformed_downsampled->points.size() << " data points." << std::endl;
  
  *scene_cloud_xyz_transformed_downsampled =  *scene_cloud_xyz_1_transformed_downsampled;
  *scene_cloud_xyz_transformed_downsampled += *scene_cloud_xyz_2_transformed_downsampled;
  *scene_cloud_xyz_transformed_downsampled += *scene_cloud_xyz_3_transformed_downsampled;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // segmentation/clustering
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold(distance_threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  int nr_points = (int) scene_cloud_xyz_transformed_downsampled->points.size();
  while(scene_cloud_xyz_transformed_downsampled->points.size() > 0.3*nr_points){
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(scene_cloud_xyz_transformed_downsampled);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0){
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (scene_cloud_xyz_transformed_downsampled);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *scene_cloud_xyz_transformed_downsampled = *cloud_f;
  }
  // downsampling object cloud to ensure below 500 points
  i = 0.0;
  while(cloud_plane->points.size() > 500){
    i += 0.001;
    vg.setInputCloud(cloud_plane);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*cloud_plane);
  }
  //std::cout << "plane cloud segmented/downsampled has [output]: " << cloud_plane->points.size() << " data points." << std::endl;
  *table_cloud_xyz_downsampled = *cloud_plane;
  // save plane cloud
  //writer.write<pcl::PointXYZ>("table.pcd", *table_cloud_xyz_downsampled, false);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (scene_cloud_xyz_transformed_downsampled);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (scene_cloud_xyz_transformed_downsampled);
  ec.extract (cluster_indices);

  int j = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back (scene_cloud_xyz_transformed_downsampled->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    if(j==0)
      break;
    j++;
  }
  
  // downsampling object cloud to ensure below 500 points
  i = 0.0;
  while(cloud_cluster->points.size() > desired_number_of_object_cloud_points){
    i += 0.001;
    vg.setInputCloud(cloud_cluster);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*cloud_cluster);
  }
  *object_cloud_xyz_downsampled = *cloud_cluster;
  
  //std::cout << "object point cloud segmented/downsampled has [output]: " << object_cloud_xyz_downsampled->points.size() << " data points." << std::endl;
  //std::stringstream ss;
  //ss << "object_cloud.pcd";
  //writer.write<pcl::PointXYZ>(ss.str(), *object_cloud_xyz_downsampled, false);
  
  

}


void evaluate_grasp_pose_candidates(void){
  
  workspace_centroid_wrt_gripper_frame_translation        << gripper_workspace_centroid_point_in_gripper_frame(0), gripper_workspace_centroid_point_in_gripper_frame(1), gripper_workspace_centroid_point_in_gripper_frame(2);
  workspace_centroid_wrt_gripper_frame_transform          << Eigen::Matrix3f::Identity(), workspace_centroid_wrt_gripper_frame_translation, 0,0,0,1;
  workspace_centroid_wrt_gripper_frame_transform_inverse  << Eigen::Matrix3f::Identity(), -Eigen::Matrix3f::Identity()*workspace_centroid_wrt_gripper_frame_translation, 0,0,0,1;
  
  // we will do translation to each object sample point in the gripper centroid frame
  object_frame_wrt_gripper_centroid_frame_transform = workspace_centroid_wrt_gripper_frame_transform_inverse*gripper_wrt_arm_hand_frame_inverse_transform*object_transform_wrt_arm_hand_frame;
    
  //int number_of_iterations = 0;
  
  // GRASPING CODE
  // iterate through all points in the "object sampling cloud"
	double total_iteration_duration = 0.0, average_iteration_duration = 0.0;
  for(unsigned int i=0; i<object_sampling_in_arm_hand_frame_xyz->points.size(); i++){
  //for(unsigned int i=0; i<object_cloud_downsampled_in_arm_hand_frame_xyz->points.size(); i++){
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
    gripper_centroid_translation_in_gripper_centroid_frame << object_sampling_in_arm_hand_frame_xyz->points[i].x,   object_sampling_in_arm_hand_frame_xyz->points[i].y,   object_sampling_in_arm_hand_frame_xyz->points[i].z;
    //gripper_centroid_translation_in_gripper_centroid_frame << object_cloud_downsampled_in_arm_hand_frame_xyz->points[i].x,   object_cloud_downsampled_in_arm_hand_frame_xyz->points[i].y,   object_cloud_downsampled_in_arm_hand_frame_xyz->points[i].z;
    gripper_centroid_transform_in_gripper_centroid_frame   << gripper_centroid_rotation_in_gripper_centroid_frame, gripper_centroid_translation_in_gripper_centroid_frame,
                                                              0,0,0,1;
    
    
    if(metric_8_score_best>=20000 and (total_score_best>40000) ){break;}
    //if(i>=100 and (total_score_best>40000) ){break;}
    
    // STEP : OK
    // iterate through the range of possible orientations at each sample point
    gripper_centroid_transform_before_orientation_loop = gripper_centroid_transform_in_gripper_centroid_frame;
    dummy_translation << 0,0,0;
    for(unsigned int j=0; j<orientation_samples_in_x; j++){				// sampling orientations about one axis (say x-axis)
			if(metric_8_score_best>=20000 and (total_score_best>40000) ){break;}
			//if(i>=100 and (total_score_best>40000) ){break;}
			
			for(unsigned int m=0; m<orientation_samples_in_y; m++){			// sampling orientations about one axis (say y-axis)
				if(metric_8_score_best>=20000 and (total_score_best>40000) ){break;}
				//if(i>=100 and (total_score_best>40000) ){break;}
				
				for(unsigned int n=0; n<orientation_samples_in_z; n++){		// sampling orientations about one axis (say z-axis)
					
					sampling_iteration++;
					auto start_iteration = std::chrono::high_resolution_clock::now();
					
					dummy_rotation = Rotx_float(initial_orientation+j*orientation_range/orientation_samples_in_x)
													*Roty_float(initial_orientation+m*orientation_range/orientation_samples_in_y)
													*Rotz_float(initial_orientation+n*orientation_range/orientation_samples_in_z);
					//dummy_rotation = Rotx_float(initial_orientation+j*orientation_range/orientation_samples)*Roty_float(initial_orientation+m*orientation_range/orientation_samples)*Rotz_float(initial_orientation+n*orientation_range/orientation_samples);
					dummy_transform << dummy_rotation, dummy_translation, 0,0,0,1;
					gripper_centroid_transform_in_gripper_centroid_frame = gripper_centroid_transform_before_orientation_loop*dummy_transform;
					
					// then transform this motion to the arm hand frame
					gripper_transform = gripper_centroid_transform_in_gripper_centroid_frame*workspace_centroid_wrt_gripper_frame_transform_inverse*gripper_wrt_arm_hand_frame_inverse_transform;
					pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyz, *gripper_cloud_transformed_in_arm_hand_frame_xyz, gripper_transform);
					
					gripper_rotation    << gripper_transform(0,0), gripper_transform(0,1), gripper_transform(0,2),
																 gripper_transform(1,0), gripper_transform(1,1), gripper_transform(1,2),
																 gripper_transform(2,0), gripper_transform(2,1), gripper_transform(2,2);
					gripper_translation << gripper_transform(0,3), gripper_transform(1,3), gripper_transform(2,3);
					
					
					
					
					
					// STEP : OK
					// check if the current gripper pose collides with table (object_plane)
					// transform the gripper point cloud to object_plane frame to be able to use the special ellipsoid
					auto start_to_check_collision_with_table = std::chrono::high_resolution_clock::now();
					pcl::transformPointCloud(*gripper_cloud_transformed_in_arm_hand_frame_xyz, *gripper_cloud_transformed_in_object_plane_frame_xyz, object_plane_transform_wrt_arm_hand_frame_inverse);
					gripper_collides_with_object_plane = false;
					for(unsigned int k=0; k<gripper_cloud_transformed_in_object_plane_frame_xyz->size(); k++){
						special_ellipsoid_value =  pow(gripper_cloud_transformed_in_object_plane_frame_xyz->points[k].x - object_plane_offset_x, 10)/pow(object_plane_x, 10) 
																		 + pow(gripper_cloud_transformed_in_object_plane_frame_xyz->points[k].y - object_plane_offset_y, 10)/pow(object_plane_y, 10) 
																		 + pow(gripper_cloud_transformed_in_object_plane_frame_xyz->points[k].z - object_plane_offset_z, 2) /pow(object_plane_z, 2);
						if( special_ellipsoid_value < 1 ){
							gripper_collides_with_object_plane = true;
							gripper_collide_with_table++;
							break;
						}
					}
					std::chrono::duration<double> time_to_check_collision_with_table = std::chrono::high_resolution_clock::now() - start_to_check_collision_with_table;
					time_elapsed_checking_gripper_collision_with_table += time_to_check_collision_with_table.count();
					average_time_to_check_gripper_collision_with_table = time_elapsed_checking_gripper_collision_with_table/sampling_iteration;
					
					
					
					if(!gripper_collides_with_object_plane){
						
						// STEP : OK
						// if the gripper doesn't collide with the table (object plane)
						// check that the gripper doesn't collide with object
						// this is done by checking gripper special ellipsoids
						// first apply the inverse gripper transform on the object cloud
						// to make it light : we use downsampled object cloud
						auto start_to_check_collision_with_object = std::chrono::high_resolution_clock::now();					
						inverse_gripper_transform << gripper_rotation.transpose(), -gripper_rotation.transpose()*gripper_translation, 0, 0, 0, 1;  // from khalil's book page 21
						pcl::transformPointCloud(*object_cloud_downsampled_in_arm_hand_frame_xyz, *object_cloud_transformed_in_arm_hand_frame_xyz, inverse_gripper_transform);
						pcl::transformPointCloud(*object_cloud_transformed_in_arm_hand_frame_xyz, *object_cloud_transformed_in_gripper_frame_xyz,  gripper_wrt_arm_hand_frame_inverse_transform);
						gripper_collides_with_object = false;
						for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyz->size(); k++){
							// for each point check it is not inside any of the gripper special ellipsoids
							for(unsigned int l=0; l<gripper_x.size(); l++){
								special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_offset_x[l], 10)/pow(gripper_x[l], 10) 
																				 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_offset_y[l], 10)/pow(gripper_y[l], 10) 
																				 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_offset_z[l], 2) /pow(gripper_z[l], 2);
								if( special_ellipsoid_value < 1 ){
									gripper_collides_with_object = true;
									gripper_collide_with_object++;
									break;
								}
							}
							if(gripper_collides_with_object)
								break;
						}					
						
						std::chrono::duration<double> time_to_check_collision_with_object = std::chrono::high_resolution_clock::now() - start_to_check_collision_with_object;
						time_elapsed_checking_gripper_collision_with_object += time_to_check_collision_with_object.count();
						average_time_to_check_gripper_collision_with_object = time_elapsed_checking_gripper_collision_with_object/sampling_iteration;
						
						
						
						if(!gripper_collides_with_object){
							// STEP
							// if gripper doesn't collide with object
							// and doesn't collide with object plane (table)
							
							if(gripper_model == "allegro_right_hand"){
								
								
								
								// METRIC#1
								// Number of workspace spheres that can access the object && Number of object points lying within
								// thumb
								metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyz,                                                                          // input : object related
																																															thumb_workspace_convex_offset, thumb_workspace_convex_parameter,                                                        // input : gripper finger related
																																															object_points_in_thumb_workspace, thumb_workspace_active_spheres_offset,   thumb_workspace_active_spheres_parameter );  // output
								// index
								metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyz,                                                                          // input : object related
																																															index_workspace_convex_offset, index_workspace_convex_parameter,                                                        // input : gripper finger related
																																															object_points_in_index_workspace, index_workspace_active_spheres_offset,   index_workspace_active_spheres_parameter );  // output
								// middle
								metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyz,                                                                          // input : object related
																																															middle_workspace_convex_offset, middle_workspace_convex_parameter,                                                      // input : gripper finger related
																																															object_points_in_middle_workspace, middle_workspace_active_spheres_offset, middle_workspace_active_spheres_parameter ); // output
								// pinky
								metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyz,                                                                          // input : object related
																																															pinky_workspace_convex_offset, pinky_workspace_convex_parameter,                                                        // input : gripper finger related
																																															object_points_in_pinky_workspace, pinky_workspace_active_spheres_offset,   pinky_workspace_active_spheres_parameter );  // output
								
								metric_1_score =  object_points_in_thumb_workspace->size()*thumb_workspace_active_spheres_offset->size()
								                 +object_points_in_index_workspace->size()*index_workspace_active_spheres_offset->size()
												         +object_points_in_middle_workspace->size()*middle_workspace_active_spheres_offset->size()
												         +object_points_in_pinky_workspace->size()*pinky_workspace_active_spheres_offset->size();
								
								
									
									
								// METRIC#2
								// How close the object centroid to gripper support offset (special ellipsoid)
								object_centroid_point_transformed << object_centroid_point_in_arm_hand_frame.x, object_centroid_point_in_arm_hand_frame.y, object_centroid_point_in_arm_hand_frame.z, 1;	// object centroid point in the old arm hand frame
								object_centroid_point_transformed = inverse_gripper_transform*object_centroid_point_transformed;  // now we have the object centroid point in the new(tansformed) arm hand frame
								object_centroid_point_transformed = gripper_wrt_arm_hand_frame_inverse_transform*object_centroid_point_transformed;  // now we have the object centroid point transformed in gripper frame
								distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_in_gripper_frame(0)-object_centroid_point_transformed(0)),2)+
																																						pow(fabs(gripper_support_offset_in_gripper_frame(1)-object_centroid_point_transformed(1)),2)+
																																						pow(fabs(gripper_support_offset_in_gripper_frame(2)-object_centroid_point_transformed(2)),2));
								
								metric_2_score = 1000.0/distance_between_gripper_support_and_object_centroid;
								
								
								
								
								// METRIC#3
								// Encapsulating the object centroid point by the finger's workspaces
								// object_points_in_thumb_workspace_centroid_point
								pcl::CentroidPoint<pcl::PointXYZ> object_points_in_thumb_workspace_centroid;
								pcl::PointXYZ object_points_in_thumb_workspace_centroid_point;
								for(unsigned int i=0;i<object_points_in_thumb_workspace->points.size();i++)
									object_points_in_thumb_workspace_centroid.add( object_points_in_thumb_workspace->points[i] );
								object_points_in_thumb_workspace_centroid.get(object_points_in_thumb_workspace_centroid_point);
								
								// object_points_in_index_workspace_centroid_point
								pcl::CentroidPoint<pcl::PointXYZ> object_points_in_index_workspace_centroid;
								pcl::PointXYZ object_points_in_index_workspace_centroid_point;
								for(unsigned int i=0;i<object_points_in_index_workspace->points.size();i++)
									object_points_in_index_workspace_centroid.add( object_points_in_index_workspace->points[i] );
								object_points_in_index_workspace_centroid.get(object_points_in_index_workspace_centroid_point);
								
								// object_points_in_middle_workspace_centroid_point
								pcl::CentroidPoint<pcl::PointXYZ> object_points_in_middle_workspace_centroid;
								pcl::PointXYZ object_points_in_middle_workspace_centroid_point;
								for(unsigned int i=0;i<object_points_in_middle_workspace->points.size();i++)
									object_points_in_middle_workspace_centroid.add( object_points_in_middle_workspace->points[i] );
								object_points_in_middle_workspace_centroid.get(object_points_in_middle_workspace_centroid_point);
								
								// object_points_in_pinky_workspace_centroid_point
								pcl::CentroidPoint<pcl::PointXYZ> object_points_in_pinky_workspace_centroid;
								pcl::PointXYZ object_points_in_pinky_workspace_centroid_point;
								for(unsigned int i=0;i<object_points_in_pinky_workspace->points.size();i++)
									object_points_in_pinky_workspace_centroid.add( object_points_in_pinky_workspace->points[i] );
								object_points_in_pinky_workspace_centroid.get(object_points_in_pinky_workspace_centroid_point);
								
								if( object_centroid_point_transformed(0) <= object_points_in_thumb_workspace_centroid_point.x and
								    object_centroid_point_transformed(0) <= object_points_in_index_workspace_centroid_point.x and
										object_centroid_point_transformed(0) <= object_points_in_middle_workspace_centroid_point.x and
										object_centroid_point_transformed(0) <= object_points_in_pinky_workspace_centroid_point.x and
										object_centroid_point_transformed(2) >  object_points_in_thumb_workspace_centroid_point.z and
										object_centroid_point_transformed(2) <  object_points_in_index_workspace_centroid_point.z and
										object_centroid_point_transformed(2) <  object_points_in_middle_workspace_centroid_point.z and
										object_centroid_point_transformed(2) <  object_points_in_pinky_workspace_centroid_point.z ){
									metric_3_score = 2000.0;
								}
								
								
								
								// METRIC#4~7
								// Number of object points in gripper support ellipsoids (4 regions)
								metric_4_score = 0.0;
								metric_5_score = 0.0;
								metric_6_score = 0.0;
								metric_7_score = 0.0;
								for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyz->size(); k++){
									special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_support_offset_x_1, 10)/pow(gripper_support_x_1, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_support_offset_y_1, 10)/pow(gripper_support_y_1, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_support_offset_z_1, 2) /pow(gripper_support_z_1, 2);
									if( special_ellipsoid_value < 1 )
										metric_4_score += 1000;
									
									special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_support_offset_x_2, 10)/pow(gripper_support_x_2, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_support_offset_y_2, 10)/pow(gripper_support_y_2, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_support_offset_z_2, 2) /pow(gripper_support_z_2, 2);
									if( special_ellipsoid_value < 1 )
										metric_5_score += 1000;
									
									special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_support_offset_x_3, 10)/pow(gripper_support_x_3, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_support_offset_y_3, 10)/pow(gripper_support_y_3, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_support_offset_z_3, 2) /pow(gripper_support_z_3, 2);
									if( special_ellipsoid_value < 1 )
										metric_6_score += 1000;
									
									special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_support_offset_x_4, 10)/pow(gripper_support_x_4, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_support_offset_y_4, 10)/pow(gripper_support_y_4, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_support_offset_z_4, 2) /pow(gripper_support_z_4, 2);
									if( special_ellipsoid_value < 1 )
										metric_7_score += 1000;
								}
								
								
								
								// METRIC#8
								// How many gripper support regions are active
								metric_8_score = 0;
								if( (metric_4_score and metric_5_score) or 
									  (metric_4_score and metric_6_score) or 
									  (metric_4_score and metric_7_score) or 
									  (metric_5_score and metric_6_score) or 
									  (metric_5_score and metric_7_score) or 
									  (metric_6_score and metric_7_score))
									metric_8_score = 5000;
								if( (metric_4_score and metric_5_score and metric_6_score) or
								    (metric_4_score and metric_5_score and metric_7_score) or
										(metric_4_score and metric_6_score and metric_7_score) or
										(metric_5_score and metric_6_score and metric_7_score) )
									metric_8_score = 10000;
								if(metric_4_score and metric_5_score and metric_6_score and metric_7_score)
									metric_8_score = 20000;
								
								
								//total_score = metric_1_score + metric_2_score + metric_3_score + metric_4_score + metric_5_score;
								total_score = metric_1_score + metric_2_score + metric_3_score + metric_4_score + metric_5_score + metric_6_score + metric_7_score + metric_8_score;
								
								
								//
								// Evaluating the best score
								if( total_score > total_score_best ){
									total_score_best = total_score;
									metric_1_score_best = metric_1_score;
									metric_2_score_best = metric_2_score;
									metric_3_score_best = metric_3_score;
									metric_4_score_best = metric_4_score;
									metric_5_score_best = metric_5_score;
									metric_6_score_best = metric_6_score;
									metric_7_score_best = metric_7_score;
									metric_8_score_best = metric_8_score;
									
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
									
									best_gripper_transform = gripper_transform;
								}
								
								
							}
							else if(gripper_model == "franka_gripper"){
							
								
								// METRIC#1
								// Number of workspace spheres that can access the object && Number of object points lying within
								// right finger
								metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyz,                                                                                            // input : object related
																																															right_finger_workspace_convex_offset, right_finger_workspace_convex_parameter,                                                            // input : gripper finger related
																																															object_points_in_right_finger_workspace, right_finger_workspace_active_spheres_offset, right_finger_workspace_active_spheres_parameter ); // output
								// left_finger
								metric_1_number_of_active_workspace_spheres_and_corresponding_object_points(  object_cloud_transformed_in_gripper_frame_xyz,                                                                                            // input : object related
																																															left_finger_workspace_convex_offset, left_finger_workspace_convex_parameter,                                                              // input : gripper finger related
																																															object_points_in_left_finger_workspace, left_finger_workspace_active_spheres_offset, left_finger_workspace_active_spheres_parameter );    // output
								
								metric_1_score = 20*( object_points_in_right_finger_workspace->size()*right_finger_workspace_active_spheres_offset->size()
								                     +object_points_in_left_finger_workspace->size()*left_finger_workspace_active_spheres_offset->size() );
								
								
								// METRIC#2
								// How close the object centroid to gripper support offset (special ellipsoid)
								object_centroid_point_transformed << object_centroid_point_in_arm_hand_frame.x, object_centroid_point_in_arm_hand_frame.y, object_centroid_point_in_arm_hand_frame.z, 1;	// object centroid point in the old arm hand frame
								object_centroid_point_transformed = inverse_gripper_transform*object_centroid_point_transformed;  // now we have the object centroid point in the new(tansformed) arm hand frame
								object_centroid_point_transformed = gripper_wrt_arm_hand_frame_inverse_transform*object_centroid_point_transformed;  // now we have the object centroid point transformed in gripper frame
								distance_between_gripper_support_and_object_centroid = sqrt(pow(fabs(gripper_support_offset_in_gripper_frame(0)-object_centroid_point_transformed(0)),2)+
																																						pow(fabs(gripper_support_offset_in_gripper_frame(1)-object_centroid_point_transformed(1)),2)+
																																						pow(fabs(gripper_support_offset_in_gripper_frame(2)-object_centroid_point_transformed(2)),2));
								
								metric_2_score = 1000.0/distance_between_gripper_support_and_object_centroid;
								
								
								// METRIC#3
								// Encapsulating the object centroid point by the finger's workspaces
								// object_points_in_right_finger_workspace_centroid_point
								pcl::CentroidPoint<pcl::PointXYZ> object_points_in_right_finger_workspace_centroid;
								pcl::PointXYZ object_points_in_right_finger_workspace_centroid_point;
								for(unsigned int i=0;i<object_points_in_right_finger_workspace->points.size();i++)
									object_points_in_right_finger_workspace_centroid.add( object_points_in_right_finger_workspace->points[i] );
								object_points_in_right_finger_workspace_centroid.get(object_points_in_right_finger_workspace_centroid_point);
								
								// object_points_in_left_finger_workspace_centroid_point
								pcl::CentroidPoint<pcl::PointXYZ> object_points_in_left_finger_workspace_centroid;
								pcl::PointXYZ object_points_in_left_finger_workspace_centroid_point;
								for(unsigned int i=0;i<object_points_in_left_finger_workspace->points.size();i++)
									object_points_in_left_finger_workspace_centroid.add( object_points_in_left_finger_workspace->points[i] );
								object_points_in_left_finger_workspace_centroid.get(object_points_in_left_finger_workspace_centroid_point);
								
								if( object_centroid_point_transformed(0) <= object_points_in_right_finger_workspace_centroid_point.x and
								    object_centroid_point_transformed(0) <= object_points_in_left_finger_workspace_centroid_point.x and
										object_centroid_point_transformed(2) >  object_points_in_right_finger_workspace_centroid_point.z and
										object_centroid_point_transformed(2) <  object_points_in_left_finger_workspace_centroid_point.z ){
									metric_3_score = 2000.0;
								}
								
								
								// METRIC#4~7
								// Number of object points in gripper support ellipsoids (1 region)
								metric_4_score = 0.0;
								metric_5_score = 0.0;
								metric_6_score = 0.0;
								metric_7_score = 0.0;
								for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyz->size(); k++){
									special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_support_offset_x_1, 10)/pow(gripper_support_x_1, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_support_offset_y_1, 10)/pow(gripper_support_y_1, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_support_offset_z_1, 2) /pow(gripper_support_z_1, 2);
									if( special_ellipsoid_value < 1 )
										metric_4_score += 1000;
									
									special_ellipsoid_value =  pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].x - gripper_support_offset_x_2, 10)/pow(gripper_support_x_2, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].y - gripper_support_offset_y_2, 10)/pow(gripper_support_y_2, 10) 
																					 + pow(object_cloud_transformed_in_gripper_frame_xyz->points[k].z - gripper_support_offset_z_2, 2) /pow(gripper_support_z_2, 2);
									if( special_ellipsoid_value < 1 )
										metric_5_score += 1000;
								}
								
								
								// METRIC#8
								// How many gripper support regions are active
								metric_8_score = 0;
								if( (metric_4_score and metric_5_score) or 
									  (metric_4_score and metric_6_score) or 
									  (metric_4_score and metric_7_score) or 
									  (metric_5_score and metric_6_score) or 
									  (metric_5_score and metric_7_score) or 
									  (metric_6_score and metric_7_score))
									metric_8_score = 5000;
								if( (metric_4_score and metric_5_score and metric_6_score) or
								    (metric_4_score and metric_5_score and metric_7_score) or
										(metric_4_score and metric_6_score and metric_7_score) or
										(metric_5_score and metric_6_score and metric_7_score) )
									metric_8_score = 10000;
								if(metric_4_score and metric_5_score and metric_6_score and metric_7_score)
									metric_8_score = 20000;
								
								
								total_score = metric_1_score + metric_2_score + metric_3_score + metric_4_score + metric_5_score + metric_6_score + metric_7_score + metric_8_score;
								
								
								//
								// Evaluating the best score
								if( total_score > total_score_best ){
									total_score_best = total_score;
									metric_1_score_best = metric_1_score;
									metric_2_score_best = metric_2_score;
									metric_3_score_best = metric_3_score;
									metric_4_score_best = metric_4_score;
									metric_5_score_best = metric_5_score;
									metric_6_score_best = metric_6_score;
									metric_7_score_best = metric_7_score;
									metric_8_score_best = metric_8_score;
									
									// saving best object points
									*object_points_in_right_finger_workspace_best = *object_points_in_right_finger_workspace;
									*object_points_in_left_finger_workspace_best  = *object_points_in_left_finger_workspace;
									
									// saving the best workspace spheres
									*right_finger_workspace_active_spheres_offset_best     = *right_finger_workspace_active_spheres_offset;
									*right_finger_workspace_active_spheres_parameter_best  = *right_finger_workspace_active_spheres_parameter;
									*left_finger_workspace_active_spheres_offset_best     = *left_finger_workspace_active_spheres_offset;
									*left_finger_workspace_active_spheres_parameter_best  = *left_finger_workspace_active_spheres_parameter;
									
									best_gripper_transform = gripper_transform;
								}
								
								
							}
							
							
							
							
							
						}
					}
					
					std::chrono::duration<double> iteration_duration = std::chrono::high_resolution_clock::now() - start_iteration;
					total_iteration_duration += iteration_duration.count();
					average_iteration_duration = total_iteration_duration/sampling_iteration;
					
					/*
					cout << "iteration: " << i << ", " << j << ", " << m << ", " << n
					     << ", time to check collision(table): "  << average_time_to_check_gripper_collision_with_table
							 << ", time to check collision(object): " << average_time_to_check_gripper_collision_with_object
							 << ", iteration time: " << average_iteration_duration 
							 << ", metric#1= " << metric_1_score_best 
							 << ", metric#2= " << metric_2_score_best 
							 << ", metric#3= " << metric_3_score_best 
							 << ", metric#4= " << metric_4_score_best
							 << ", metric#5= " << metric_5_score_best 
							 << ", metric#6= " << metric_6_score_best 
							 << ", metric#7= " << metric_7_score_best
							 << ", metric#8= " << metric_8_score_best 
							 << ", total_score= " << total_score_best << endl;
					*/
					if(metric_8_score_best>=20000 and (total_score_best>40000) ){break;}
					//if(i>=100 and (total_score_best>40000) ){break;}
				}
			}
    }
  }
}


void visualize_workspace_spheres_and_object_points_for_best_gripper_pose(void){
  // Draw the workspace spheres of the best gripper pose
  if(gripper_model == "allegro_right_hand"){
    // thumb
    scene_cloud_viewer->addPointCloud(thumb_workspace_spheres_best, red_color, "thumb workspace spheres");
    scene_cloud_viewer->addPointCloud(object_points_in_thumb_workspace_best, red_color, "thumb workspace points");
    for(unsigned int j=0; j<thumb_workspace_active_spheres_offset_best->size(); j++){
      parameter_vector << thumb_workspace_active_spheres_parameter_best->points[j].x, thumb_workspace_active_spheres_parameter_best->points[j].y, thumb_workspace_active_spheres_parameter_best->points[j].z;
      offset_vector    << thumb_workspace_active_spheres_offset_best->points[j].x, thumb_workspace_active_spheres_offset_best->points[j].y, thumb_workspace_active_spheres_offset_best->points[j].z;
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 100, 2 );
      *thumb_workspace_spheres_best += *dummy_cloud_xyz;}
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 100, 2 );
      *index_workspace_spheres_best += *dummy_cloud_xyz;}
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 100, 2 );
      *middle_workspace_spheres_best += *dummy_cloud_xyz;}
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 100, 2 );
      *pinky_workspace_spheres_best += *dummy_cloud_xyz;}
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 50, 2 );
      *right_finger_workspace_spheres_best += *dummy_cloud_xyz;}
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyz, parameter_vector, offset_vector, 50, 2 );
      *left_finger_workspace_spheres_best += *dummy_cloud_xyz;}
    pcl::transformPointCloud(*left_finger_workspace_spheres_best, *left_finger_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
    pcl::transformPointCloud(*object_points_in_left_finger_workspace_best, *object_points_in_left_finger_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_left_finger_workspace_best, green_color_again, "left_finger workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "left_finger workspace points");
  }
}



#endif

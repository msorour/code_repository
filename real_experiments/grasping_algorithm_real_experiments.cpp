
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch

then run using:
reset && cmake .. && make && ./grasping_algorithm_real_experiments storage_bin2 0.01 0.007 ../include/gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera.pcd
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int8.h>

#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include "include/Eigen/Dense"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <unistd.h>

#include "include/useful_implementations.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "../pcl_codes/grasping2/include/grasping_algorithm.h"

//boost::shared_ptr<pcl::visualization::PCLVisualizer> realsense_viewer  (new pcl::visualization::PCLVisualizer("realsense cloud"));

pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud                            (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_xyz                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_filtered_xyz               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_xyzrgb                 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_transformed_xyzrgb     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    current_scene_cloud_transformed_xyz        (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointXYZ  point_xyz;

//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(augmented_cloud);

std::string file_name;
double leaf_size, distance_threshold;

void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *current_scene_cloud_xyzrgb = *temp_cloud;
  std::cout << "point cloud arrived ... " <<std::endl;
}


int main(int argc, char **argv){
  std::string point_cloud_name, point_cloud_id;
  point_cloud_name   = argv[1];
  leaf_size          = std::stof( argv[2] );
  distance_threshold = std::stof( argv[3] );
  
  // ROS
  ros::init(argc, argv, "grasping_algorithm_real_experiments");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);  // once per 10 seconds
  
  //realsense_viewer->addPointCloud(current_scene_cloud_xyzrgb, rgb,"realsense cloud");
  //realsense_viewer->addCoordinateSystem(0.2);
  
  ros::Publisher pose_pub           = n.advertise<geometry_msgs::PoseStamped> ("/panda_arm/command/pose", 1);
  ros::Publisher joint_position_pub = n.advertise<std_msgs::Int8>             ("/panda_arm/command/joint_position", 10);
  
  ros::Subscriber realsense_sub     = n.subscribe<sensor_msgs::PointCloud2>   ("/camera/depth/color/points", 1, view_point_cloud);
  
  
  
  tf::TransformListener listener;
  tf::StampedTransform stamped_transform;
  
  //realsense_viewer->setBackgroundColor(255,255,255);
  


  
  // arm pose
  geometry_msgs::PoseStamped poseCommand;
  geometry_msgs::PoseStamped poseResponse;
  poseCommand.header.frame_id="/panda_link0";
  
  Eigen::Vector3f camera_depth_frame_new_wrt_link0_translation;
  Eigen::Matrix3f camera_depth_frame_new_wrt_link0_rotation;
  Eigen::Matrix4f camera_depth_frame_new_wrt_link0_transform;
  
  Eigen::Vector3f camera_depth_frame_old_wrt_link0_translation;
  Eigen::Matrix3f camera_depth_frame_old_wrt_link0_rotation;
  Eigen::Matrix4f camera_depth_frame_old_wrt_link0_transform;
  
  Eigen::Matrix4f camera_depth_frame_new_wrt_camera_depth_frame_old_transform;
  Eigen::Quaternionf q;
  
  
  // This transformation matrix will be used later on to go to desired gripper pose
	// get current panda_hand with respect to panda_link0 transformation matrix
	//Eigen::Matrix3f arm_hand_wrt_arm_link0_frame_rotation = Eigen::Quaternionf{Eigen::AngleAxisf{0.3659975, Eigen::Vector3f{-0.01241378, 0.01459942, -0.9304185}}}.toRotationMatrix();
	Eigen::Vector3f arm_hand_wrt_arm_link0_frame_translation_old;
	Eigen::Matrix3f arm_hand_wrt_arm_link0_frame_rotation_old;
	Eigen::Matrix4f arm_hand_wrt_arm_link0_frame_transform_old;
  
  
  std_msgs::Int8 config_index;
  config_index.data = 0;
  
  clock_t begin7, end;
	double time_spent;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // STEP#1 : Getting the object + table point clouds
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if ( ros::ok() ){
  	// get the current camera_depth_frame transform with respect to link0
		ros::Duration(1).sleep();
		try{listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), stamped_transform);}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
		poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
		poseResponse.pose.orientation.x = stamped_transform.getRotation().x();
	
		camera_depth_frame_old_wrt_link0_translation << stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z();
		q.x() = stamped_transform.getRotation().x();
		q.y() = stamped_transform.getRotation().y();
		q.z() = stamped_transform.getRotation().z();
		q.w() = stamped_transform.getRotation().w();
		camera_depth_frame_old_wrt_link0_rotation = q.normalized().toRotationMatrix();
		camera_depth_frame_old_wrt_link0_transform << camera_depth_frame_old_wrt_link0_rotation, camera_depth_frame_old_wrt_link0_translation,
					                          0,0,0,1;
		
		std::cout << "camera_depth_frame_old_wrt_link0_transform = " <<std::endl << camera_depth_frame_old_wrt_link0_transform <<std::endl;
  	
  	
  	// This transformation matrix will be used later on to go to desired gripper pose
  	// get current panda_hand with respect to panda_link0 transformation matrix
		try{listener.lookupTransform("/panda_link0", "/panda_hand", ros::Time(0), stamped_transform);}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
		poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
		poseResponse.pose.orientation.x = stamped_transform.getRotation().x();

		arm_hand_wrt_arm_link0_frame_translation_old << stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z();
		q.x() = stamped_transform.getRotation().x();
		q.y() = stamped_transform.getRotation().y();
		q.z() = stamped_transform.getRotation().z();
		q.w() = stamped_transform.getRotation().w();
		arm_hand_wrt_arm_link0_frame_rotation_old = q.normalized().toRotationMatrix();
		arm_hand_wrt_arm_link0_frame_transform_old << arm_hand_wrt_arm_link0_frame_rotation_old, arm_hand_wrt_arm_link0_frame_translation_old,
						                                      0,0,0,1;
  	
  	
  	
  	// declarations for segmentation
  	// Create the filtering object: downsample the dataset using a leaf size
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PCDWriter writer;
  	
  	pcl::ExtractIndices<pcl::PointXYZ> extract;
  	
  	// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		
  	
  	// taking 3 point clouds
    for(int i=0;i<3;i++){
      
      config_index.data = i;
      joint_position_pub.publish( config_index );
			ros::Duration(8).sleep();
		  ros::spinOnce();
			
			
      // get the current camera_depth_frame transform with respect to link0
			try{listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), stamped_transform);}
			catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
			poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
			poseResponse.pose.orientation.x = stamped_transform.getRotation().x();
			
			camera_depth_frame_new_wrt_link0_translation << stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z();
			q.x() = stamped_transform.getRotation().x();
			q.y() = stamped_transform.getRotation().y();
			q.z() = stamped_transform.getRotation().z();
			q.w() = stamped_transform.getRotation().w();
			camera_depth_frame_new_wrt_link0_rotation = q.normalized().toRotationMatrix();
			camera_depth_frame_new_wrt_link0_transform << camera_depth_frame_new_wrt_link0_rotation, camera_depth_frame_new_wrt_link0_translation,
			                                0,0,0,1;
			
      camera_depth_frame_new_wrt_camera_depth_frame_old_transform = camera_depth_frame_old_wrt_link0_transform.inverse()*camera_depth_frame_new_wrt_link0_transform;
      pcl::transformPointCloud (*current_scene_cloud_xyzrgb, *current_scene_cloud_transformed_xyzrgb, camera_depth_frame_new_wrt_camera_depth_frame_old_transform);
      
      current_scene_cloud_transformed_xyz->clear();
      for(unsigned int j=0; j<current_scene_cloud_transformed_xyzrgb->size(); j++){
	      point_xyz.x = current_scene_cloud_transformed_xyzrgb->points[j].x;
      	point_xyz.y = current_scene_cloud_transformed_xyzrgb->points[j].y;
      	point_xyz.z = current_scene_cloud_transformed_xyzrgb->points[j].z;
      	current_scene_cloud_transformed_xyz->points.push_back( point_xyz );
      }
      
      
      // segmentation
      begin7 = clock();
	
			// Read in the cloud data
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
			std::cout << "PointCloud before filtering has: " << current_scene_cloud_transformed_xyz->points.size () << " data points." << std::endl; //*

			vg.setInputCloud (current_scene_cloud_transformed_xyz);
			vg.setLeafSize (leaf_size, leaf_size, leaf_size);
			vg.filter (*cloud_filtered);
			std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

			
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold(distance_threshold);

			int nr_points = (int) cloud_filtered->points.size ();
			int l = 0;
			while (cloud_filtered->points.size () > 0.3 * nr_points){
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud (cloud_filtered);
				seg.segment (*inliers, *coefficients);
				if (inliers->indices.size () == 0){
				  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				  break;
				}

				// Extract the planar inliers from the input cloud
				extract.setInputCloud (cloud_filtered);
				extract.setIndices (inliers);
				extract.setNegative (false);

				// Get the points associated with the planar surface
				extract.filter (*cloud_plane);
				
				// Remove the planar inliers, extract the rest
				extract.setNegative (true);
				extract.filter (*cloud_f);
				*cloud_filtered = *cloud_f;
				l++;
			}
			writer.write<pcl::PointXYZ>( "pcd_files/" + point_cloud_name + "_table.pcd", *cloud_plane, false);
			tree->setInputCloud (cloud_filtered);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (0.02); // 2cm
			ec.setMinClusterSize (100);
			ec.setMaxClusterSize (25000);
			ec.setSearchMethod (tree);
			ec.setInputCloud (cloud_filtered);
			ec.extract (cluster_indices);

			int k = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
				
				if(k==0)
					*augmented_cloud_xyz  += *cloud_cluster;
				k++;
			}
			
			end = clock();
			time_spent = (double)( end - begin7 )/ CLOCKS_PER_SEC;
			std::cout << "time spent in clustering/segmentation = " << time_spent << std::endl;
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  // remove far points then save
  augmented_cloud_filtered_xyz->clear();
	for(unsigned int j=0; j<augmented_cloud_xyz->size(); j++){
    point_xyz.x = augmented_cloud_xyz->points[j].x;
  	point_xyz.y = augmented_cloud_xyz->points[j].y;
  	point_xyz.z = augmented_cloud_xyz->points[j].z;
  	if(point_xyz.z < 0.7 and point_xyz.z > 0.27)
  		augmented_cloud_filtered_xyz->points.push_back( point_xyz );
  }
  std::cout << "segmented object cloud size : " << augmented_cloud_filtered_xyz->points.size ()  << " data points." << std::endl;
  augmented_cloud_filtered_xyz->width = 1;
	augmented_cloud_filtered_xyz->height = augmented_cloud_filtered_xyz->points.size();
  
  // save point cloud data
  file_name = "pcd_files/"+ point_cloud_name + ".pcd";
  //pcl::io::savePCDFileASCII(file_name, *augmented_cloud_xyz);
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud_filtered_xyz);
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // STEP#2 : running grasping algorithm and visualize output arm pose candidate
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  clock_t begin, begin2, begin3, begin4, begin5;
	begin = clock();
	
  std::string gripper_file_name      = argv[4];
  double gripper_leaf_size           = 0.01;
  double object_leaf_size            = 0.01;
  std::string object_file_name       = "pcd_files/"+ point_cloud_name + ".pcd";
  std::string object_plane_file_name = "pcd_files/"+ point_cloud_name + "_table.pcd";
  
  std::string gripper_model;
	if(gripper_file_name.find("allegro_right_hand" )!=std::string::npos){gripper_model = "allegro_right_hand";}
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){gripper_model = "franka_gripper";}
  
  // point clouds declarations
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_cloud_in_camera_depth_optical_frame_xyz                       (new pcl::PointCloud<pcl::PointXYZ>);
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_gripper_frame_xyz                  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_gripper_frame_xyzrgb               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_arm_hand_frame_xyz                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  object_plane_cloud_downsampled_in_object_plane_frame_xyzrgb          (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr     object_plane_cloud_downsampled_in_object_plane_frame_xyz             (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr     gripper_cloud_in_gripper_frame_xyz                                   (new pcl::PointCloud<pcl::PointXYZ>);
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
  scene_cloud_viewer->addCoordinateSystem(0.2);   // this is arm hand frame (the origin)
  scene_cloud_viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
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
  
  // downsampling object plane point cloud
  vg.setInputCloud(object_plane_cloud_in_camera_depth_optical_frame_xyz);
  vg.setLeafSize(object_leaf_size, object_leaf_size, object_leaf_size);
  vg.filter (*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz);
  std::cout << "Table point cloud before downsampling has:      " << object_plane_cloud_in_camera_depth_optical_frame_xyz->points.size()              << " data points." << std::endl;
  std::cout << "Table point cloud after downsampling has :      " << object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz, *object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  // downsampling gripper point cloud
  vg.setInputCloud(gripper_cloud_in_gripper_frame_xyz);
  vg.setLeafSize(gripper_leaf_size, gripper_leaf_size, gripper_leaf_size);
  vg.filter (*gripper_cloud_downsampled_in_gripper_frame_xyz);
  std::cout << "Gripper point cloud before downsampling has:     " << gripper_cloud_in_gripper_frame_xyz->points.size()              << " data points." << std::endl;
  std::cout << "Gripper point cloud after downsampling has :     " << gripper_cloud_downsampled_in_gripper_frame_xyz->points.size()  << " data points." << std::endl;
  copyPointCloud(*gripper_cloud_downsampled_in_gripper_frame_xyz, *gripper_cloud_downsampled_in_gripper_frame_xyzrgb);
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Defining transformations
  // camera optical frame wrt arm hand frame
  //Eigen::Vector3f camera_depth_optical_frame_wrt_arm_hand_frame_translation;   camera_depth_optical_frame_wrt_arm_hand_frame_translation << -0.067, 0.033, -0.027; // originally i use
  Eigen::Vector3f camera_depth_optical_frame_wrt_arm_hand_frame_translation;   camera_depth_optical_frame_wrt_arm_hand_frame_translation << -0.051, 0.023, -0.017; // from calibration
  //Eigen::Vector3f camera_depth_optical_frame_wrt_arm_hand_frame_translation;   camera_depth_optical_frame_wrt_arm_hand_frame_translation << -0.06, 0.027, -0.022;  // hard tuned
  Eigen::Matrix4f camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  camera_depth_optical_frame_wrt_arm_hand_frame_transform << Rotz_float(-M_PI/2), camera_depth_optical_frame_wrt_arm_hand_frame_translation,
                                                             0,0,0,1;
  transform.matrix() = camera_depth_optical_frame_wrt_arm_hand_frame_transform;
  scene_cloud_viewer->addCoordinateSystem(0.07, transform, "camera depth optical frame", 0);
  
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
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "gripper frame", 0);
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
  scene_cloud_viewer->addCoordinateSystem(0.2, transform, "object frame", 0);
  
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
  int object_sampling_in_x_axis = 7;   int object_sampling_in_y_axis = 7;   int object_sampling_in_z_axis = 3;
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
  scene_cloud_viewer->addCoordinateSystem(0.1, transform, "object plane frame", 0);
  
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
  object_plane_x = 0.35;       object_plane_y = 0.05;         object_plane_z = 0.35;
  object_plane_offset_x = 0.0; object_plane_offset_y = -0.05; object_plane_offset_z = 0.0;
  parameter_vector << object_plane_x, object_plane_y, object_plane_z;
  offset_vector    << object_plane_offset_x, object_plane_offset_y, object_plane_offset_z;
  construct_special_ellipsoid_point_cloud( object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
  pcl::transformPointCloud(*object_plane_special_ellipsoid_point_cloud_in_object_plane_frame, *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, object_plane_transform_wrt_arm_hand_frame);
  
  // gripper approximation as a set of special ellipsoids
  std::vector<double> gripper_x, gripper_y, gripper_z;
  std::vector<double> gripper_offset_x, gripper_offset_y, gripper_offset_z;
  if(gripper_model == "allegro_right_hand"){
    gripper_x.push_back(0.017);    gripper_y.push_back(0.059);    gripper_z.push_back(0.059);    gripper_offset_x.push_back(-0.017);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.057);  // palm
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.05);    gripper_offset_z.push_back(0.17); // index
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17); // middle
    //gripper_x.push_back(0.014);    gripper_y.push_back(0.016);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(-0.05);   gripper_offset_z.push_back(0.17); // pinky
    gripper_x.push_back(0.014);    gripper_y.push_back(0.070);    gripper_z.push_back(0.07);     gripper_offset_x.push_back(-0.014);    gripper_offset_y.push_back(0.0);     gripper_offset_z.push_back(0.17);   // index, middle, pinky
    gripper_x.push_back(0.014);    gripper_y.push_back(0.08);     gripper_z.push_back(0.012);    gripper_offset_x.push_back(-0.022);    gripper_offset_y.push_back(0.11);    gripper_offset_z.push_back(0.022);  // thumb
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_as_set_of_special_ellipsoids_in_gripper_frame    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  gripper_as_set_of_special_ellipsoids_in_arm_hand_frame   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  dummy_cloud_xyzrgb                                       (new pcl::PointCloud<pcl::PointXYZRGB>);
  for(unsigned int j=0; j<gripper_x.size(); j++){
    parameter_vector << gripper_x[j], gripper_y[j], gripper_z[j];
    offset_vector    << gripper_offset_x[j], gripper_offset_y[j], gripper_offset_z[j];
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 100, 10, 255, 0, 0 );
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
  
  int orientation_samples = 10;
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
  Eigen::Matrix3f best_gripper_rotation;
  Eigen::Vector3f best_gripper_translation;
  
  
  
  
  
  
  
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
      
      
      
      
      
      
      
      /*
      
      // object cloud
			scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,          "object cloud");
		
			// object plane cloud
			scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,      "table cloud");
		
			// object sampling cloud
			scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                "object sampling cloud");
		
			// gripper cloud
			//*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb += *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame;
			//scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, black_color,           "gripper cloud in arm hand frame");
			//scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color,            "gripper cloud in gripper frame");
			//scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
		
			//pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, gripper_transform);
			scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, black_color_again,     "gripper cloud transformed in arm hand frame");
      
      while ( !scene_cloud_viewer->wasStopped() ){scene_cloud_viewer->spinOnce();}
      
      */
      
      
      
      
      
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
            std::cout<<"metric#2 : "<<distance_between_gripper_support_and_object_centroid_best << ", at point: " << object_sampling_in_object_frame_xyzrgb->points[i] <<std::endl;
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
          std::cout<<"metric#2 : "<<distance_between_gripper_support_and_object_centroid_best << ", at point: " << object_sampling_in_object_frame_xyzrgb->points[i] <<std::endl;
          best_gripper_transform = gripper_transform;
          }
        }
      
      }
      }
      
      
      
      
      
      
      
      // object cloud
			scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,          "object cloud");
		
			// object plane cloud
			//*object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb += *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame;
			scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,      "table cloud");
		
			// object sampling cloud
			scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                "object sampling cloud");
		
			// gripper cloud
			//*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb += *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame;
			//scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, black_color,           "gripper cloud in arm hand frame");
			//scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color,            "gripper cloud in gripper frame");
			//scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
		
			pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, gripper_transform);
			scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, black_color_again,     "gripper cloud transformed in arm hand frame");
      
      
      
      
      
      
      
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
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
      construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
      *left_finger_workspace_spheres_best += *dummy_cloud_xyzrgb;}
    pcl::transformPointCloud(*left_finger_workspace_spheres_best, *left_finger_workspace_spheres_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(left_finger_workspace_spheres_best, green_color_again, "left_finger workspace spheres");
    pcl::transformPointCloud(*object_points_in_left_finger_workspace_best, *object_points_in_left_finger_workspace_best, best_gripper_transform*gripper_wrt_arm_hand_frame_transform);
    scene_cloud_viewer->updatePointCloud(object_points_in_left_finger_workspace_best, green_color_again, "left_finger workspace points");
    scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "left_finger workspace points");
  }
  
  
  
  
  
  
  // object cloud
  scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyzrgb, magenta_color,          "object cloud");
  
  // object plane cloud
  *object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb += *object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame;
  scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyzrgb, brown_color,      "table cloud");
  
  // object sampling cloud
  //scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyzrgb, blue_color_again,                "object sampling cloud");
  
  // gripper cloud
  *gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb += *gripper_as_set_of_special_ellipsoids_in_arm_hand_frame;
  scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, black_color,           "gripper cloud in arm hand frame");
  //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_gripper_frame_xyzrgb, black_color,            "gripper cloud in gripper frame");
  //scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_object_plane_frame_xyzrgb, black_color_again, "gripper cloud in object plane frame");
  
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyzrgb, *gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyzrgb, black_color_again,     "gripper cloud transformed in arm hand frame");
  
  //
  //scene_cloud_viewer->updatePointCloud(scene_cloud_xyzrgb, scene_cloud_rgb, "scene cloud viewer");
  
  
  
  // output
  best_gripper_rotation    << best_gripper_transform(0,0), best_gripper_transform(0,1), best_gripper_transform(0,2),
                              best_gripper_transform(1,0), best_gripper_transform(1,1), best_gripper_transform(1,2),
                              best_gripper_transform(2,0), best_gripper_transform(2,1), best_gripper_transform(2,2);
  best_gripper_translation << best_gripper_transform(0,3), best_gripper_transform(1,3), best_gripper_transform(2,3);
  Eigen::Quaternionf best_gripper_rotation_quat(best_gripper_rotation);
  //std::cout << "best_gripper_rotation    = " << best_gripper_rotation_quat.x() << ", " << best_gripper_rotation_quat.y() << ", " << best_gripper_rotation_quat.z() << ", " << best_gripper_rotation_quat.w() << std::endl;
  //std::cout << "best_gripper_translation = " << best_gripper_translation << std::endl;
  
  std::cout << "best_gripper_transform = " << std::endl << best_gripper_transform << std::endl;
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by this program = " << time_spent << std::endl;
  
  while ( !scene_cloud_viewer->wasStopped() ){scene_cloud_viewer->spinOnce();}
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  /*
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // STEP#3 : move the gripper to desired grasping pose
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
	// desired gripper transformation matrix with respect to panda_hand frame
	Eigen::Matrix4f desired_hand_transform_in_arm_hand_frame;
	desired_hand_transform_in_arm_hand_frame <<  
 	-0.093092,   -0.79676,  -0.597082, -0.0423373,
   0.342657,  -0.588692,    0.73214, -0.0668557,
  -0.934837,  -0.136438,   0.327818,   0.314664,
          0,          0,          0,          1;
	
	// desired hand transform in link0 frame
	Eigen::Matrix4f desired_hand_transform_in_arm_link0_frame;
	desired_hand_transform_in_arm_link0_frame = arm_hand_wrt_arm_link0_frame_transform_old*desired_hand_transform_in_arm_hand_frame;
	std::cout << "desired_hand_transform_in_arm_link0_frame = " <<std::endl << desired_hand_transform_in_arm_link0_frame <<std::endl;
  
	Eigen::Matrix3f desired_hand_rotation_in_arm_link0_frame;
	desired_hand_rotation_in_arm_link0_frame << desired_hand_transform_in_arm_link0_frame(0,0), desired_hand_transform_in_arm_link0_frame(0,1), desired_hand_transform_in_arm_link0_frame(0,2),
                                              desired_hand_transform_in_arm_link0_frame(1,0), desired_hand_transform_in_arm_link0_frame(1,1), desired_hand_transform_in_arm_link0_frame(1,2),
                                              desired_hand_transform_in_arm_link0_frame(2,0), desired_hand_transform_in_arm_link0_frame(2,1), desired_hand_transform_in_arm_link0_frame(2,2);
	
  Eigen::Quaternionf desired_hand_rotation_in_arm_link0_frame_quat(desired_hand_rotation_in_arm_link0_frame);
  std::cout << "best_hand_rotation = " << desired_hand_rotation_in_arm_link0_frame_quat.x() << ", " << desired_hand_rotation_in_arm_link0_frame_quat.y() << ", " << desired_hand_rotation_in_arm_link0_frame_quat.z() << ", " << desired_hand_rotation_in_arm_link0_frame_quat.w() << std::endl;
  
  Eigen::Vector3f desired_hand_translation_in_arm_link0_frame;
  desired_hand_translation_in_arm_link0_frame << desired_hand_transform_in_arm_link0_frame(0,3), desired_hand_transform_in_arm_link0_frame(1,3), desired_hand_transform_in_arm_link0_frame(2,3);
  
  poseCommand.pose.position.x = desired_hand_translation_in_arm_link0_frame(0);
  poseCommand.pose.position.y = desired_hand_translation_in_arm_link0_frame(1);
  poseCommand.pose.position.z = 0.17;
  poseCommand.pose.orientation.x = desired_hand_rotation_in_arm_link0_frame_quat.x();
  poseCommand.pose.orientation.y = desired_hand_rotation_in_arm_link0_frame_quat.y();
  poseCommand.pose.orientation.z = desired_hand_rotation_in_arm_link0_frame_quat.z();
  poseCommand.pose.orientation.w = desired_hand_rotation_in_arm_link0_frame_quat.w();
  
  for(int i=0;i<1;i++){
    
    // send pose command
		pose_pub.publish( poseCommand );
		
		// wait for 10 seconds for motion execution
		ros::Duration(10).sleep();
		ros::spinOnce();
    loop_rate.sleep();
  }
  
  
  
  */
  
  
  
  
  /*
  while(!realsense_viewer->wasStopped()){
  	realsense_viewer->spinOnce();
  }
  */
  
  
  
  
  
  return 0;
}













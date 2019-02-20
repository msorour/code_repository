
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch

then run using:
reset && cmake .. && make && ./grasping_algorithm_real_experiments mug 0.006 0.006
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
  point_cloud_name = argv[1];
  leaf_size = std::stof( argv[2] );
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
  tf::StampedTransform transform;
  
  //realsense_viewer->setBackgroundColor(255,255,255);
  


  
  // arm pose
  geometry_msgs::PoseStamped poseCommand;
  geometry_msgs::PoseStamped poseResponse;
  poseCommand.header.frame_id="/panda_link0";
  
  /*
  poseStamped.pose.position.x = 0.45;
  poseStamped.pose.position.y = 0.25;
  poseStamped.pose.position.z = 0.38;
  poseStamped.pose.orientation.x = 0.868;
  poseStamped.pose.orientation.y = -0.339;
  poseStamped.pose.orientation.z = 0.149;
  poseStamped.pose.orientation.w = 0.330;
  
  // send pose command
  pose_pub.publish( poseCommand );
  // wait for 7 seconds for motion execution
  ros::Duration(7).sleep();
  ros::spinOnce();
  */
  
  
  Eigen::Vector3f camera_depth_frame_new_wrt_link0_translation;
  Eigen::Matrix3f camera_depth_frame_new_wrt_link0_rotation;
  Eigen::Matrix4f camera_depth_frame_new_wrt_link0_transform;
  
  Eigen::Vector3f camera_depth_frame_old_wrt_link0_translation;
  Eigen::Matrix3f camera_depth_frame_old_wrt_link0_rotation;
  Eigen::Matrix4f camera_depth_frame_old_wrt_link0_transform;
  
  Eigen::Matrix4f camera_depth_frame_new_wrt_camera_depth_frame_old_transform;
  Eigen::Quaternionf q;
  
  
  std_msgs::Int8 config_index;
  config_index.data = 0;
  
  clock_t begin, end;
	double time_spent;
		
  
  if ( ros::ok() ){
  	// get the current camera_depth_frame transform with respect to link0
		ros::Duration(1).sleep();
		try{listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), transform);}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
		poseResponse.pose.position.x    = transform.getOrigin().x();
		poseResponse.pose.orientation.x = transform.getRotation().x();
	
		camera_depth_frame_old_wrt_link0_translation << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
		q.x() = transform.getRotation().x();
		q.y() = transform.getRotation().y();
		q.z() = transform.getRotation().z();
		q.w() = transform.getRotation().w();
		camera_depth_frame_old_wrt_link0_rotation = q.normalized().toRotationMatrix();
		camera_depth_frame_old_wrt_link0_transform << camera_depth_frame_old_wrt_link0_rotation, camera_depth_frame_old_wrt_link0_translation,
					                          0,0,0,1;
		
		std::cout << "camera_depth_frame_old_wrt_link0_transform = " <<std::endl << camera_depth_frame_old_wrt_link0_transform <<std::endl;
  	
  	
  	
  	
  	
  	
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
			try{listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), transform);}
			catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
			poseResponse.pose.position.x    = transform.getOrigin().x();
			poseResponse.pose.orientation.x = transform.getRotation().x();
			
			camera_depth_frame_new_wrt_link0_translation << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
			q.x() = transform.getRotation().x();
			q.y() = transform.getRotation().y();
			q.z() = transform.getRotation().z();
			q.w() = transform.getRotation().w();
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
      begin = clock();
	
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
			time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
			std::cout << "time spent in clustering/segmentation = " << time_spent << std::endl;
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      //*augmented_cloud  += *current_scene_cloud_transformed_xyzrgb;
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  //realsense_viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb, "realsense cloud");
  
  
  
  // remove far points then save
  augmented_cloud_filtered_xyz->clear();
	for(unsigned int j=0; j<augmented_cloud_xyz->size(); j++){
    point_xyz.x = augmented_cloud_xyz->points[j].x;
  	point_xyz.y = augmented_cloud_xyz->points[j].y;
  	point_xyz.z = augmented_cloud_xyz->points[j].z;
  	if(point_xyz.z < 0.7)
  		augmented_cloud_filtered_xyz->points.push_back( point_xyz );
  }
  std::cout << "segmented object cloud size : " << augmented_cloud_filtered_xyz->points.size ()  << " data points." << std::endl;
  augmented_cloud_filtered_xyz->width = 1;
	augmented_cloud_filtered_xyz->height = augmented_cloud_filtered_xyz->points.size();
  
  // save point cloud data
  file_name = "pcd_files/"+ point_cloud_name + ".pcd";
  //pcl::io::savePCDFileASCII(file_name, *augmented_cloud_xyz);
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud_filtered_xyz);
  
  
  
  
  
  
  
  
  
  /*
  while(!realsense_viewer->wasStopped()){
  	realsense_viewer->spinOnce();
  }
  */
  
  
  
  
  
  return 0;
}













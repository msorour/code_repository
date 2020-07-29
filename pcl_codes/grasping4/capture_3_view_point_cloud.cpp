
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch

then run using:
reset && cmake .. && make && ./capture_3_view_point_cloud storage_bin
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int8.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
//#include "../include/Eigen/Dense"

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

#include "include/grasping_algorithm.h"

//boost::shared_ptr<pcl::visualization::PCLVisualizer> realsense_viewer  (new pcl::visualization::PCLVisualizer("realsense cloud"));

pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud                            (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_xyzrgb                 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_transformed_xyzrgb     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy_xyzrgb                               (new pcl::PointCloud<pcl::PointXYZRGB>);

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
  
  // ROS
  ros::init(argc, argv, "capture_3_view_point_cloud");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);  // once per 10 seconds
  
  //realsense_viewer->addPointCloud(current_scene_cloud_xyzrgb, rgb,"realsense cloud");
  //realsense_viewer->addCoordinateSystem(0.2);
  
  ros::Publisher pose_pub           = n.advertise<geometry_msgs::PoseStamped> ("/panda_arm/command/pose", 1);
  ros::Publisher joint_position_pub = n.advertise<std_msgs::Int8>             ("/panda_arm/command/joint_position", 10);
  ros::Subscriber realsense_sub     = n.subscribe<sensor_msgs::PointCloud2>   ("/camera/depth/color/points", 1, view_point_cloud);
  
  tf::TransformListener listener;
  tf::StampedTransform stamped_transform;
  
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
  
  std_msgs::Int8 config_index;
  config_index.data = 0;
  
  clock_t begin7, end;
	double time_spent;
	
	ofstream transformation_matrix_file;
  transformation_matrix_file.open("../raw_object_pcd_files/"+ point_cloud_name + "_tf.txt");
  std::string tf_string;
  
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // STEP#1 : Getting the raw object point cloud
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
		
		// capturing 3 point clouds
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
			
			dummy_xyzrgb->clear();
			// remove far points
			for(int j=0; j<current_scene_cloud_xyzrgb->size(); j++){
				if(current_scene_cloud_xyzrgb->points[j].z < 1.0)
					dummy_xyzrgb->points.push_back(current_scene_cloud_xyzrgb->points[j]);
			}
			*current_scene_cloud_xyzrgb = *dummy_xyzrgb;
			current_scene_cloud_xyzrgb->width = current_scene_cloud_xyzrgb->points.size();
			current_scene_cloud_xyzrgb->height = 1;
			current_scene_cloud_xyzrgb->is_dense = true;
			
			// save individual point cloud
  		file_name = "../raw_object_pcd_files/"+ point_cloud_name + "_" + std::to_string(i) + ".pcd";
			pcl::io::savePCDFileASCII(file_name, *current_scene_cloud_xyzrgb);
			
      camera_depth_frame_new_wrt_camera_depth_frame_old_transform = camera_depth_frame_old_wrt_link0_transform.inverse()*camera_depth_frame_new_wrt_link0_transform;
      pcl::transformPointCloud (*current_scene_cloud_xyzrgb, *current_scene_cloud_transformed_xyzrgb, camera_depth_frame_new_wrt_camera_depth_frame_old_transform);
      *augmented_cloud += *current_scene_cloud_transformed_xyzrgb;
      
      std::stringstream ss;
		  ss << camera_depth_frame_new_wrt_camera_depth_frame_old_transform;
		  tf_string += ss.str() + "\n";
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  // save point cloud data
  file_name = "../raw_object_pcd_files/"+ point_cloud_name + ".pcd";
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
  
  transformation_matrix_file << tf_string;
  transformation_matrix_file.close();
  
  return 0;
}




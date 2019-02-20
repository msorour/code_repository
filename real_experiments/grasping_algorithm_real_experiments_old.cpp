
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud                     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_xyz                 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud2                    (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud                 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_transformed     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    current_scene_cloud_transformed_xyz (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointXYZ  point_xyz;

//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(augmented_cloud);

std::string file_name;
double leaf_size, distance_threshold;

void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *current_scene_cloud = *temp_cloud;
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
  
  //realsense_viewer->addPointCloud(current_scene_cloud, rgb,"realsense cloud");
  //realsense_viewer->addCoordinateSystem(0.2);
  
  ros::Publisher pose_pub           = n.advertise<geometry_msgs::PoseStamped> ("/panda_arm/command/pose", 1);
  ros::Publisher joint_position_pub = n.advertise<std_msgs::Int8>             ("/panda_arm/command/joint_position", 10);
  
  ros::Subscriber realsense_sub     = n.subscribe<sensor_msgs::PointCloud2>   ("/camera/depth/color/points", 1, view_point_cloud);
  
  
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  
  //realsense_viewer->setBackgroundColor(255,255,255);
  


  
  // the 3 arm poses
  std::vector<geometry_msgs::PoseStamped> poseVectorCommand;
  std::vector<geometry_msgs::PoseStamped> poseVectorResponse;
  geometry_msgs::PoseStamped poseStamped;
  geometry_msgs::PoseStamped poseResponse;
  poseStamped.header.frame_id="/panda_link0";
  std::vector<double> x_shift, y_shift, z_shift;
  /*
  // pose#1
  poseStamped.pose.position.x = 0.45;
  poseStamped.pose.position.y = 0.25;
  poseStamped.pose.position.z = 0.38;
  poseStamped.pose.orientation.x = 0.868;
  poseStamped.pose.orientation.y = -0.339;
  poseStamped.pose.orientation.z = 0.149;
  poseStamped.pose.orientation.w = 0.330;
  poseVectorCommand.push_back(poseStamped);
  x_shift.push_back(0.0);
  y_shift.push_back(0.0);
  z_shift.push_back(0.0);
  */
  /*
  // pose#2
  poseStamped.pose.position.x = 0.45;
  poseStamped.pose.position.y = 0.0;
  poseStamped.pose.position.z = 0.460;
  poseStamped.pose.orientation.x = 0.906;
  poseStamped.pose.orientation.y = -0.420;
  poseStamped.pose.orientation.z = 0.054;
  poseStamped.pose.orientation.w = -0.008;
  poseVectorCommand.push_back(poseStamped);
  //x_shift.push_back(0.02);
  //y_shift.push_back(0.0);
  //z_shift.push_back(0.01);
  x_shift.push_back(0.02);
  y_shift.push_back(0.0);
  z_shift.push_back(0.01);
  */
  /*
  // pose#3
  poseStamped.pose.position.x = 0.45;
  poseStamped.pose.position.y = -0.25;
  poseStamped.pose.position.z = 0.38;
  poseStamped.pose.orientation.x = 0.870;
  poseStamped.pose.orientation.y = -0.405;
  poseStamped.pose.orientation.z = -0.099;
  poseStamped.pose.orientation.w = -0.263;
  poseVectorCommand.push_back(poseStamped);
  x_shift.push_back(0.03);
  y_shift.push_back(0.01);
  z_shift.push_back(0.03);
  */
  
  
  Eigen::Vector3f dummy_translation;
  Eigen::Matrix3f dummy_rotation;
  Eigen::Matrix4f dummy_transform;
  
  Eigen::Vector3f link8new_wrt_link0_translation;
  Eigen::Matrix3f link8new_wrt_link0_rotation;
  Eigen::Matrix4f link8new_wrt_link0_transform;
  
  Eigen::Vector3f gripper_wrt_link8_translation;
  Eigen::Matrix3f gripper_wrt_link8_rotation;
  Eigen::Matrix4f gripper_wrt_link8_transform;
  gripper_wrt_link8_translation << 0.0204, 0, 0.02;
  gripper_wrt_link8_transform << Eigen::Matrix3f::Identity(), gripper_wrt_link8_translation,
                                 0,0,0,1;
  dummy_translation << 0,0,0;
  dummy_rotation = Rotz_float(-M_PI/4);
  dummy_transform << dummy_rotation, dummy_translation,
                     0,0,0,1;
  
  gripper_wrt_link8_transform = dummy_transform*gripper_wrt_link8_transform;
  
  Eigen::Vector3f link8old_wrt_link0_translation;
  Eigen::Matrix3f link8old_wrt_link0_rotation;
  Eigen::Matrix4f link8old_wrt_link0_transform;
  
  Eigen::Matrix4f link8new_wrt_link8old_transform;
  Eigen::Matrix4f camera_transform;
  Eigen::Quaternionf q;
  
  
  std_msgs::Int8 config_index;
  config_index.data = 0;
  
  clock_t begin, end;
	double time_spent;
		
  
  if ( ros::ok() ){
  	// get original transform before moving
		// get the current link8 pose transform with respect to link0
		
		ros::Duration(1).sleep();
		try{
			//listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), transform);
			//listener.lookupTransform("/panda_link0", "/camera_link", ros::Time(0), transform);
			//listener.lookupTransform("/panda_link0", "/camera_color_optical_frame", ros::Time(0), transform);
			listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
		 ROS_ERROR("%s",ex.what());
		 ros::Duration(1.0).sleep();
		}
		poseResponse.pose.position.x    = transform.getOrigin().x();
		poseResponse.pose.orientation.x = transform.getRotation().x();
	
		link8old_wrt_link0_translation << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
		q.x() = transform.getRotation().x();
		q.y() = transform.getRotation().y();
		q.z() = transform.getRotation().z();
		q.w() = transform.getRotation().w();
		link8old_wrt_link0_rotation = q.normalized().toRotationMatrix();
		link8old_wrt_link0_transform << link8old_wrt_link0_rotation, link8old_wrt_link0_translation,
					                          0,0,0,1;
		
		std::cout << "link8old_wrt_link0_transform = " <<std::endl << link8old_wrt_link0_transform <<std::endl;
  	
  	
  	
  	
  	
  	
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
		
  	
  	
    //for(int i=0;i<poseVectorCommand.size();i++){
    for(int i=0;i<3;i++){
      
      config_index.data = i;
      joint_position_pub.publish( config_index );
			ros::Duration(8).sleep();
		  ros::spinOnce();
			
			
      // send pose command
      //pose_pub.publish( poseVectorCommand[i] );
      
      // wait for 6 seconds for motion execution
      //ros::Duration(6).sleep();
      
      //ros::spinOnce();
      
      
      
      // get the current link8 pose transform with respect to link0
			try{
			//listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), transform);
			//listener.lookupTransform("/panda_link0", "/camera_link", ros::Time(0), transform);
			//listener.lookupTransform("/panda_link0", "/camera_color_optical_frame", ros::Time(0), transform);
			listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			 ROS_ERROR("%s",ex.what());
			 ros::Duration(1.0).sleep();
			}
			poseResponse.pose.position.x    = transform.getOrigin().x();
			poseResponse.pose.orientation.x = transform.getRotation().x();
			
			link8new_wrt_link0_translation << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
			q.x() = transform.getRotation().x();
			q.y() = transform.getRotation().y();
			q.z() = transform.getRotation().z();
			q.w() = transform.getRotation().w();
			link8new_wrt_link0_rotation = q.normalized().toRotationMatrix();
			link8new_wrt_link0_transform << link8new_wrt_link0_rotation, link8new_wrt_link0_translation,
			                                0,0,0,1;
			
      //std::cout << "link8new_wrt_link0_transform = " <<std::endl << link8new_wrt_link0_transform <<std::endl;
      link8new_wrt_link8old_transform = link8old_wrt_link0_transform.inverse()*link8new_wrt_link0_transform;
      camera_transform = link8new_wrt_link8old_transform;
      //std::cout << "relative transform = " <<std::endl << camera_transform <<std::endl;
      pcl::transformPointCloud (*current_scene_cloud, *current_scene_cloud_transformed, camera_transform);
      
      //file_name = point_cloud_name+"_" + std::to_string(i) + ".pcd";
      //pcl::io::savePCDFileASCII(file_name, *current_scene_cloud);
      
      // shifting the point cloud a bit -> hardcoded fix of mismatching problem
      

      current_scene_cloud_transformed_xyz->clear();
      for(unsigned int j=0; j<current_scene_cloud_transformed->size(); j++){
	      //current_scene_cloud_transformed->points[j].x += x_shift[i]; 
      	//current_scene_cloud_transformed->points[j].y += y_shift[i];
      	//current_scene_cloud_transformed->points[j].z -= z_shift[i];
      	//if(i>0)
      	//	current_scene_cloud_transformed->points[j].z -= 0.01;
      	
      	point_xyz.x = current_scene_cloud_transformed->points[j].x;
      	point_xyz.y = current_scene_cloud_transformed->points[j].y;
      	point_xyz.z = current_scene_cloud_transformed->points[j].z;
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
				//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
				
				// save plane cloud for the first point cloud only
				//if(i==0 and l==0)
			  //writer.write<pcl::PointXYZ>( "pcd_files/" + point_cloud_name + "_table.pcd", *cloud_plane, false);

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
				std::stringstream ss;
				ss << "cloud_cluster_" << i << "_" << k << ".pcd";
				writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
				
				if(k==0)
					*augmented_cloud_xyz  += *cloud_cluster;
				k++;
				
			}
		
			end = clock();
			time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
			std::cout << "time spent in clustering/segmentation = " << time_spent << std::endl;
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      //*augmented_cloud  += *current_scene_cloud_transformed;
      //*augmented_cloud  += *current_scene_cloud_transformed;
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  //realsense_viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb, "realsense cloud");
  
  // save point cloud data
  file_name = "pcd_files/"+ point_cloud_name + ".pcd";
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud_xyz);
  /*
  while(!realsense_viewer->wasStopped()){
  	realsense_viewer->spinOnce();
  }
  */
  return 0;
}










/*

link8new_wrt_link0_transform = 
 0.655757 -0.666768  0.354123 -0.353283
-0.747378 -0.639681  0.179539  0.371874
 0.106815 -0.382398 -0.917803  0.430491
        0         0         0         1

link8new_wrt_link0_transform = 
   0.67918  -0.727294 -0.0987814  -0.406854
  -0.67182  -0.561812  -0.482726   0.378553
  0.295587   0.394221  -0.870183   0.295187
         0          0          0          1
*/









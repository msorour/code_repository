
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch

then run using:
reset && cmake .. && make -j7 && ./grasping_algorithm_real_experiments cup_without_handle ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled_100.pcd

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

#include "include/declarations.h"
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
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_xyz                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_filtered_xyz               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_xyzrgb                 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    current_scene_cloud_xyz                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_transformed_xyzrgb     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    current_scene_cloud_transformed_xyz        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    dummy_xyz                                  (new pcl::PointCloud<pcl::PointXYZ>);

std::string file_name;

void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *current_scene_cloud_xyzrgb = *temp_cloud;
  std::cout << "point cloud arrived ... " <<std::endl;
}
/*
void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *current_scene_cloud_xyzrgb = *temp_cloud;
  std::cout << "point cloud arrived ... " <<std::endl;
}
*/

int main(int argc, char **argv){
  std::string point_cloud_name, point_cloud_id;
  point_cloud_name   = argv[1];
  
  // ROS
  ros::init(argc, argv, "grasping_algorithm_real_experiments");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);  // once per 10 seconds
  
  ros::Publisher pose_pub                 = n.advertise<geometry_msgs::PoseStamped> ("/panda_arm/command/pose", 1);
  ros::Publisher joint_position_pub       = n.advertise<std_msgs::Int8>             ("/panda_arm/command/joint_position", 10);
  
  ros::Subscriber realsense_sub           = n.subscribe<sensor_msgs::PointCloud2>   ("/camera/depth/color/points", 1, view_point_cloud);
  //ros::Subscriber realsense_image_sub     = n.subscribe<sensor_msgs::PointCloud2>   ("/camera/color/image_raw/compressed", 1, view_color_image);
  
  
  
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
	
	ofstream transformation_matrix_file;
  transformation_matrix_file.open("../raw_object_pcd_files/"+ point_cloud_name + "_tf.txt");
  std::string tf_string;
	
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
  	
  	// capturing the 3 view point clouds
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
			
			dummy_xyz->clear();
			// remove far and near points
			for(int j=0; j<current_scene_cloud_xyzrgb->size(); j++){
				if(current_scene_cloud_xyzrgb->points[j].z < 0.7 and current_scene_cloud_xyzrgb->points[j].z > 0.3){
					point_xyz.x = current_scene_cloud_xyzrgb->points[j].x;
					point_xyz.y = current_scene_cloud_xyzrgb->points[j].y;
					point_xyz.z = current_scene_cloud_xyzrgb->points[j].z;
					dummy_xyz->points.push_back(point_xyz);
				}
			}
			*current_scene_cloud_xyz = *dummy_xyz;
			current_scene_cloud_xyz->width = current_scene_cloud_xyz->points.size();
			current_scene_cloud_xyz->height = 1;
			current_scene_cloud_xyz->is_dense = true;
			
			
			// save individual point cloud
  		file_name = "../raw_object_pcd_files/"+ point_cloud_name + "_" + std::to_string(i) + ".pcd";
			pcl::io::savePCDFileASCII(file_name, *current_scene_cloud_xyz);
			
      camera_depth_frame_new_wrt_camera_depth_frame_old_transform = camera_depth_frame_old_wrt_link0_transform.inverse()*camera_depth_frame_new_wrt_link0_transform;
      pcl::transformPointCloud (*current_scene_cloud_xyzrgb, *current_scene_cloud_transformed_xyzrgb, camera_depth_frame_new_wrt_camera_depth_frame_old_transform);
      *augmented_cloud += *current_scene_cloud_transformed_xyzrgb;
      
      std::stringstream ss;
		  ss << camera_depth_frame_new_wrt_camera_depth_frame_old_transform;
		  tf_string += ss.str() + "\n";
      
      
      if(i==0){
        *scene_cloud_xyz_1 = *current_scene_cloud_xyz;
        tm1 = camera_depth_frame_new_wrt_camera_depth_frame_old_transform;}
      if(i==1){
        *scene_cloud_xyz_2 = *current_scene_cloud_xyz;
        tm2 = camera_depth_frame_new_wrt_camera_depth_frame_old_transform;}
      if(i==2){
        *scene_cloud_xyz_3 = *current_scene_cloud_xyz;
        tm3 = camera_depth_frame_new_wrt_camera_depth_frame_old_transform;}
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  
  // save point cloud data
  file_name = "../raw_object_pcd_files/"+ point_cloud_name + ".pcd";
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
  
  transformation_matrix_file << tf_string;
  transformation_matrix_file.close();
  
  
  
  
  // visualization of point cloud
  scene_cloud_viewer->addCoordinateSystem(0.2);   // this is arm hand frame (the origin)
  scene_cloud_viewer->setCameraPosition(-1.53884, 0.506528, -0.636167, -0.171077, 0.068023, 0.333948, 0.522106, -0.203709, -0.828196, 0);
  scene_cloud_viewer->setBackgroundColor(255,255,255);
  
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_arm_hand_frame_xyz, magenta_color,                    "object cloud");
  scene_cloud_viewer->addPointCloud(object_sampling_in_arm_hand_frame_xyz, blue_color_again,                          "object sampling cloud");
  
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyz, brown_color,                "table cloud");
  scene_cloud_viewer->addPointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, orange_color,       "table special ellipsoid");
  
  cout << "scene_cloud_xyz_1->points.size() = " << scene_cloud_xyz_1->points.size() << endl;
  cout << "scene_cloud_xyz_2->points.size() = " << scene_cloud_xyz_2->points.size() << endl;
  cout << "scene_cloud_xyz_3->points.size() = " << scene_cloud_xyz_3->points.size() << endl;
  
  cout << "tm1 = " << endl << tm1 << endl;
  cout << "tm2 = " << endl << tm2 << endl;
  cout << "tm3 = " << endl << tm3 << endl;
  /*
  //std::vector<pcl::visualization::Camera> cam;
  while ( !scene_cloud_viewer->wasStopped() ){
    scene_cloud_viewer->spinOnce();
  }
  */
  return 0;
}




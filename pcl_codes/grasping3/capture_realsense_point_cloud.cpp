
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"

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



boost::shared_ptr<pcl::visualization::PCLVisualizer> realsense_viewer (new pcl::visualization::PCLVisualizer("realsense cloud"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(augmented_cloud);
std::string file_name;

void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *current_scene_cloud = *temp_cloud;
  std::cout << "point cloud arrived ... " <<std::endl;
  
  // save point cloud data
  //pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
}


int main(int argc, char **argv){
  std::string point_cloud_name, point_cloud_id;
  point_cloud_name = argv[1];
  //point_cloud_id = argv[2];
  //file_name = point_cloud_name+"_"+point_cloud_id+".pcd";
  
  // ROS
  ros::init(argc, argv, "capture_realsense_point_cloud");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);  // once per 10 seconds
  
  realsense_viewer->addPointCloud(current_scene_cloud, rgb,"realsense cloud");
  realsense_viewer->addCoordinateSystem(0.2);
  
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/panda_arm/command/pose", 1);
  //ros::Publisher  index_joint_position_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/joint_state/index/position", 10);
  ros::Subscriber realsense_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, view_point_cloud);
  //ros::Subscriber tf_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, view_point_cloud);
  //ros::Subscriber arm_transformation_sub = n.subscribe<sensor_msgs::PointCloud2>("/franka_state_controller/joint_states", 1, get_arm_transformation_matrix);
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  
  realsense_viewer->setBackgroundColor(255,255,255);
  /*
  while ( ros::ok() and !realsense_viewer->wasStopped()){
    realsense_viewer->spinOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  
  



  
  // the 3 arm poses
  std::vector<geometry_msgs::PoseStamped> poseVectorCommand;
  std::vector<geometry_msgs::PoseStamped> poseVectorResponse;
  geometry_msgs::PoseStamped poseStamped;
  geometry_msgs::PoseStamped poseResponse;
  poseStamped.header.frame_id="/panda_link0";
  
	/*
  // pose#1
  poseStamped.pose.position.x = 0.560;
  poseStamped.pose.position.y = -0.047;
  poseStamped.pose.position.z = 0.491;
  poseStamped.pose.orientation.x = 0.917;
  poseStamped.pose.orientation.y = -0.395;
  poseStamped.pose.orientation.z = 0.033;
  poseStamped.pose.orientation.w = -0.036;
  poseVectorCommand.push_back(poseStamped);
  
  // pose#2
  poseStamped.pose.position.x = 0.560;
  poseStamped.pose.position.y = 0.047;
  poseStamped.pose.position.z = 0.491;
  poseStamped.pose.orientation.x = 0.917;
  poseStamped.pose.orientation.y = -0.395;
  poseStamped.pose.orientation.z = 0.033;
  poseStamped.pose.orientation.w = -0.036;
  poseVectorCommand.push_back(poseStamped);
  */
  
  
  
  // pose#1
  poseStamped.pose.position.x = -0.259;
  poseStamped.pose.position.y = 0.486;
  poseStamped.pose.position.z = 0.427;
  poseStamped.pose.orientation.x = 0.928;
  poseStamped.pose.orientation.y = -0.371;
  poseStamped.pose.orientation.z = 0.014;
  poseStamped.pose.orientation.w = 0.019;
  poseVectorCommand.push_back(poseStamped);
  
  // pose#2
  poseStamped.pose.position.x = -0.359;
  poseStamped.pose.position.y = 0.486;
  poseStamped.pose.position.z = 0.427;
  poseStamped.pose.orientation.x = 0.928;
  poseStamped.pose.orientation.y = -0.371;
  poseStamped.pose.orientation.z = 0.014;
  poseStamped.pose.orientation.w = 0.019;
  poseVectorCommand.push_back(poseStamped);
  
  
  
  
  
  /*
  // pose#2
  poseStamped.pose.position.x = 0.570;
  poseStamped.pose.position.y = 0.126;
  poseStamped.pose.position.z = 0.466;
  poseStamped.pose.orientation.x = 0.916;
  poseStamped.pose.orientation.y = -0.300;
  poseStamped.pose.orientation.z = 0.167;
  poseStamped.pose.orientation.w = 0.207;
  poseVectorCommand.push_back(poseStamped);
  */
  
  /*
  // pose#3
  poseStamped.pose.position.x = 0.557;
  poseStamped.pose.position.y = -0.250;
  poseStamped.pose.position.z = 0.448;
  poseStamped.pose.orientation.x = 0.844;
  poseStamped.pose.orientation.y = -0.486;
  poseStamped.pose.orientation.z = 0.029;
  poseStamped.pose.orientation.w = -0.225;
  poseVectorCommand.push_back(poseStamped);
  */
/*
  poseStamped.pose.position.x = 0.5;
  poseStamped.pose.position.y = 0.0;
  poseStamped.pose.position.z = 0.5;
  poseStamped.pose.orientation.x = 0.924;
  poseStamped.pose.orientation.y = -0.383;
  poseStamped.pose.orientation.z = 0.0;
  poseStamped.pose.orientation.w = 0.0;
  poseVectorCommand.push_back(poseStamped);
  poseVectorCommand.push_back(poseStamped);
  poseVectorCommand.push_back(poseStamped);
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
  Eigen::Matrix4f link8old_wrt_link0_transform_inverse;
  
  Eigen::Matrix4f link8new_wrt_link8old_transform;
  Eigen::Matrix4f camera_transform;
  Eigen::Quaternionf q;
  
  
  
  
  
  
  if ( ros::ok() ){
  	// get original transform before moving
		// get the current link8 pose transform with respect to link0
		ros::Duration(1).sleep();
		try{
		 listener.lookupTransform("/panda_link8", "/panda_link0", ros::Time(0), transform);
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
		//link8old_wrt_link0_transform = gripper_wrt_link8_transform*link8old_wrt_link0_transform;
		link8old_wrt_link0_transform = link8old_wrt_link0_transform;
		link8old_wrt_link0_transform_inverse << link8old_wrt_link0_rotation.transpose(), -link8old_wrt_link0_rotation.transpose()*link8old_wrt_link0_translation,
			                                      0,0,0,1;
	
		std::cout << "link8old_wrt_link0_transform = " <<std::endl << link8old_wrt_link0_transform <<std::endl;
  	
  	
    for(int i=0;i<poseVectorCommand.size();i++){
      //realsense_viewer->spinOnce();
      
      // send pose command
      pose_pub.publish( poseVectorCommand[i] );
      
      // wait for 7 seconds
      ros::Duration(7).sleep();
      
      ros::spinOnce();
      
      
      
      // get the current link8 pose transform with respect to link0
			try{
			 listener.lookupTransform("/panda_link8", "/panda_link0", ros::Time(0), transform);
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
			//link8new_wrt_link0_transform = gripper_wrt_link8_transform*link8new_wrt_link0_transform;
			link8new_wrt_link0_transform = link8new_wrt_link0_transform;
			
      std::cout << "link8new_wrt_link0_transform = " <<std::endl << link8new_wrt_link0_transform <<std::endl;
      
      link8new_wrt_link8old_transform = link8old_wrt_link0_transform_inverse*link8new_wrt_link0_transform;
      //std::cout << "link8new_wrt_link8old_transform = " <<std::endl << link8new_wrt_link8old_transform <<std::endl;
      
      //camera_transform = link8new_wrt_link8old_transform;
      //pcl::transformPointCloud (*current_scene_cloud, *current_scene_cloud, camera_transform);
      
      file_name = point_cloud_name+"_" + std::to_string(i) + ".pcd";
      pcl::io::savePCDFileASCII(file_name, *current_scene_cloud);
      
      *augmented_cloud += *current_scene_cloud;
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  realsense_viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb, "realsense cloud");
  
  // save point cloud data
  //file_name = point_cloud_name+"_" + std::to_string(i) + ".pcd";
  //file_name = point_cloud_name + ".pcd";
  //pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
  
  while(!realsense_viewer->wasStopped()){
  	realsense_viewer->spinOnce();
  }
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









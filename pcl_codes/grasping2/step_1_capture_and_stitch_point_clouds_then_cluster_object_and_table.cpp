
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch
*/

#include <ros/ros.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include "../../ros_packages/include/Eigen/Dense"

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

#include "geometry_msgs/PoseStamped.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> realsense_viewer (new pcl::visualization::PCLVisualizer("realsense cloud"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(augmented_cloud);
std::string file_name;

void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *augmented_cloud = *temp_cloud;
  realsense_viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb, "realsense cloud");
  
  // save point cloud data
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
}


int main(int argc, char **argv){
  std::string point_cloud_name, point_cloud_id;
  point_cloud_name = argv[1];
  point_cloud_id = argv[2];
  file_name = point_cloud_name+"_"+point_cloud_id+".pcd";
  
  // ROS
  ros::init(argc, argv, "capture_realsense_point_cloud");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.05);  // once per 20 seconds
  
  realsense_viewer->addPointCloud(augmented_cloud, rgb,"realsense cloud");
  realsense_viewer->addCoordinateSystem(0.2);
  
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/panda_arm/command/pose", 1);
  //ros::Publisher  index_joint_position_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/joint_state/index/position", 10);
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, view_point_cloud);
  
  realsense_viewer->setBackgroundColor(255,255,255);
  /*
  while ( ros::ok() and !realsense_viewer->wasStopped()){
    realsense_viewer->spinOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  
  



  
  // the 3 arm poses
  std::vector<geometry_msgs::PoseStamped> poseStampedVector;
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id="/panda_link0";
  // pose#1
  poseStamped.pose.position.x = 0.560;
  poseStamped.pose.position.y = -0.047;
  poseStamped.pose.position.z = 0.491;
  poseStamped.pose.orientation.x = 0.917;
  poseStamped.pose.orientation.y = -0.395;
  poseStamped.pose.orientation.z = 0.033;
  poseStamped.pose.orientation.w = -0.036;
  poseStampedVector.push_back(poseStamped);
  
  // pose#2
  poseStamped.pose.position.x = 0.570;
  poseStamped.pose.position.y = 0.126;
  poseStamped.pose.position.z = 0.466;
  poseStamped.pose.orientation.x = 0.916;
  poseStamped.pose.orientation.y = -0.300;
  poseStamped.pose.orientation.z = 0.167;
  poseStamped.pose.orientation.w = 0.207;
  poseStampedVector.push_back(poseStamped);
  
  // pose#3
  poseStamped.pose.position.x = 0.557;
  poseStamped.pose.position.y = -0.250;
  poseStamped.pose.position.z = 0.448;
  poseStamped.pose.orientation.x = 0.844;
  poseStamped.pose.orientation.y = -0.486;
  poseStamped.pose.orientation.z = 0.029;
  poseStamped.pose.orientation.w = -0.225;
  poseStampedVector.push_back(poseStamped);
  
  if ( ros::ok() ){
    for(int i=0;i<3;i++){
      realsense_viewer->spinOnce();
      
      // send pose command
      pose_pub.publish( poseStampedVector[i] );
      
      // wait for 10 seconds
      ros::Duration(10).sleep();
      
      // save point cloud data
      file_name = point_cloud_name+"_" + std::to_string(i) + ".pcd";
      //pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  
  
  
  
  
  return 0;
}

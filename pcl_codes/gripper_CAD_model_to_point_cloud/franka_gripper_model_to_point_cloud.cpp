#include <ros/ros.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include "../../ros_packages/include/Eigen/Dense"
#include "../../ros_packages/include/useful_implementations.h"

int main(){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_base_cloud   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_finger_cloud   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_finger_cloud    (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr connection_cloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud      (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr franka_gripper_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("franka gripper point cloud"));
  viewer->setBackgroundColor(255,255,255);
  viewer->addCoordinateSystem(0.05);
  
  // load .ply meshes, we first generate .ply file from the .dae file using meshlab software
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/franka_gripper_meshes/hand.ply"  , *gripper_base_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/franka_gripper_meshes/finger.ply", *right_finger_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/franka_gripper_meshes/finger.ply", *left_finger_cloud);
  
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/franka_gripper_meshes/franka_gripper_realsense_connection.ply", *connection_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/franka_gripper_meshes/realsense_d435.ply", *camera_cloud);
  
  
  //
  Eigen::Vector3f translation;
  translation << 0,0,0;
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f transform;
  transform << rotation, translation,
               0, 0, 0, 1;
  
  // gripper base link
  pcl::transformPointCloud(*gripper_base_cloud, *gripper_base_cloud, transform);
  
  // right finger
  translation << 0,-0.04,0.0584;
  rotation = Rotz_float( 3.14159 );
  transform << rotation, translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*right_finger_cloud, *right_finger_cloud, transform);
  
  // left finger
  translation << 0,0.04,0.0584;
  rotation = Eigen::Matrix3f::Identity();
  transform << rotation, translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*left_finger_cloud, *left_finger_cloud, transform);
  
  
  // realsense camera
  translation << -0.0673, 0, -0.02;
  rotation = Roty_float( -M_PI/2 );
  transform << rotation, translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*camera_cloud, *camera_cloud, transform);
  
  // camera connection
  translation << 0, 0, 0;
  rotation = Rotz_float( -M_PI/2 );
  transform << rotation, translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*connection_cloud, *connection_cloud, transform);
  
  translation << 0, 0, 0;
  rotation = Rotz_float( M_PI );
  transform << rotation, translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*connection_cloud, *connection_cloud, transform);
  
  
  // concatenate point clouds
  *franka_gripper_cloud = *gripper_base_cloud + *right_finger_cloud;
  *franka_gripper_cloud += *left_finger_cloud;
  *franka_gripper_cloud += *camera_cloud;
	*franka_gripper_cloud += *connection_cloud;
  
  /*
  // modify the hand cloud : shift the zero point to be identical with the kinematic model we use
  for(unsigned int i=0;i<franka_gripper_cloud->points.size();i++){
    franka_gripper_cloud->points[i].x -= 0.0098;
    franka_gripper_cloud->points[i].z += 0.095;
  }
  */
  // save the allegro hand cloud data
  pcl::io::savePCDFileASCII("../gripper_point_cloud/franka_gripper_model_cloud_plus_camera.pcd", *franka_gripper_cloud);
  
  viewer->addPointCloud( franka_gripper_cloud, "franka gripper point cloud" );
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "franka gripper point cloud");
  while (!viewer->wasStopped()){viewer->spinOnce();}
  return 0;
}

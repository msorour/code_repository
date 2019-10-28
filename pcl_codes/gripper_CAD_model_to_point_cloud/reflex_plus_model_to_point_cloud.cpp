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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_link_cloud           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pad_cloud                 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr swivel_1_cloud            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr swivel_2_cloud            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_1_cloud          (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_pad_1_cloud      (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_1_cloud            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_pad_1_cloud        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flex_block_cloud          (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr palm_cloud                (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger1_cloud             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger2_cloud             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger3_cloud             (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr connection_cloud          (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud              (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr reflex_plus_cloud         (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer      (new pcl::visualization::PCLVisualizer ("reflex plus point cloud"));
  viewer->setBackgroundColor(255,255,255);
  std::string id = "cloud";
  
  // load .ply meshes, we first generate .ply file from the .stl file using meshlab software
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/base_link.ply", *base_link_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/distal_1.ply", *distal_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/distal_pad_1.ply", *distal_pad_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/flex_block.ply", *flex_block_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/pad.ply", *pad_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/proximal_1.ply", *proximal_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/proximal_pad_1.ply", *proximal_pad_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/swivel_1.ply", *swivel_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../reflex_plus_meshes/swivel_2.ply", *swivel_2_cloud);
  
  //pcl::io::loadPLYFile<pcl::PointXYZRGB>("reflex_plus_meshes/reflex_plus_realsense_connection.ply", *connection_cloud);
  //pcl::io::loadPLYFile<pcl::PointXYZRGB>("reflex_plus_meshes/realsense_d435.ply", *camera_cloud);
  
  
  
  
  
  
  //
  // palm construction
  // link1
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0, 0;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_link_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*base_link_cloud, *base_link_transformed_cloud, transform);
  
  // link2
  Eigen::Affine3f relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pad_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*pad_cloud, *pad_transformed_cloud, transform);
  
  
  /*
  // link3
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0177;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_14_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_14_cloud, *link_14_transformed_cloud, transform);
  
  // link4
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0514;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_15_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_15_cloud, *link_15_transformed_cloud, transform);
  
  // link5 (tip)
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0543 - 0.0120;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_15_tip_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_15_tip_cloud, *link_15_tip_transformed_cloud, transform);
  */
  
  *palm_cloud = *base_link_transformed_cloud;
  //*palm_cloud = *base_link_transformed_cloud + *pad_transformed_cloud;
  
  /*
  *palm_cloud = *palm_cloud + *link_14_transformed_cloud;
  *palm_cloud = *palm_cloud + *link_15_transformed_cloud;
  *palm_cloud = *palm_cloud + *link_15_tip_transformed_cloud;
  */
  
  
  /*
  //
  // finger1 construction
  // link1
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -0.0182, 0.019333, -0.045987;
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(0.0), Eigen::Vector3f::UnitX() ) );
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(-95.0), Eigen::Vector3f::UnitY() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(-90.0), Eigen::Vector3f::UnitX() ) );
  
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_12_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_12_cloud, *link_12_transformed_cloud, transform);
  
  // link2
  Eigen::Affine3f relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.027, 0.005, 0.0399;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_13_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_13_cloud, *link_13_transformed_cloud, transform);
  
  // link3
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0177;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_14_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_14_cloud, *link_14_transformed_cloud, transform);
  
  // link4
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0514;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_15_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_15_cloud, *link_15_transformed_cloud, transform);
  
  // link5 (tip)
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0543 - 0.0120;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_15_tip_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_15_tip_cloud, *link_15_tip_transformed_cloud, transform);
  
  *finger1_cloud = *link_12_transformed_cloud + *link_13_transformed_cloud;
  *finger1_cloud = *finger1_cloud + *link_14_transformed_cloud;
  *finger1_cloud = *finger1_cloud + *link_15_transformed_cloud;
  *finger1_cloud = *finger1_cloud + *link_15_tip_transformed_cloud;
  
  
  
  //
  // index finger construction
  // link1
  transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0.0435, -0.001542;
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(-5.0), Eigen::Vector3f::UnitX() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(0.0), Eigen::Vector3f::UnitY() ) );
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(0.0), Eigen::Vector3f::UnitX() ) );

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_0_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_0_cloud, *link_0_transformed_cloud, transform);
  
  // link2
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0164;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_1_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_1_cloud, *link_1_transformed_cloud, transform);
  
  // link3
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0540;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_2_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_2_cloud, *link_2_transformed_cloud, transform);
  
  // link4
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0384;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_3_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_3_cloud, *link_3_transformed_cloud, transform);
  
  // link5 (tip)
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0387 - 0.0120;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_3_tip_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*link_3_tip_cloud, *link_3_tip_transformed_cloud, transform);
  
  *finger2_cloud = *link_0_transformed_cloud + *link_1_transformed_cloud;
  *finger2_cloud = *finger2_cloud + *link_2_transformed_cloud;
  *finger2_cloud = *finger2_cloud + *link_3_transformed_cloud;
  *finger2_cloud = *finger2_cloud + *link_3_tip_transformed_cloud;
  
  
  
  //
  // middle finger construction
  // link1
  transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0, 0.0007;
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(0.0), Eigen::Vector3f::UnitX() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(0.0), Eigen::Vector3f::UnitY() ) );
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(0.0), Eigen::Vector3f::UnitX() ) );

  // Executing the transformation
  pcl::transformPointCloud(*link_0_cloud, *link_0_transformed_cloud, transform);
  
  // link2
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0164;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::transformPointCloud(*link_1_cloud, *link_1_transformed_cloud, transform);
  
  // link3
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0540;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::transformPointCloud(*link_2_cloud, *link_2_transformed_cloud, transform);
  
  // link4
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0384;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::transformPointCloud(*link_3_cloud, *link_3_transformed_cloud, transform);
  
  // link5 (tip)
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0.0387 - 0.0120;
  relative_transform.prerotate( transform.rotation() );
  transform.translation() += relative_transform.translation();
  // Executing the transformation
  pcl::transformPointCloud(*link_3_tip_cloud, *link_3_tip_transformed_cloud, transform);
  
  *finger3_cloud = *link_0_transformed_cloud + *link_1_transformed_cloud;
  *finger3_cloud = *finger3_cloud + *link_2_transformed_cloud;
  *finger3_cloud = *finger3_cloud + *link_3_transformed_cloud;
  *finger3_cloud = *finger3_cloud + *link_3_tip_transformed_cloud;
  
  
  
  */
  
  
  
  //
  // concatenate point clouds
  *reflex_plus_cloud = *palm_cloud;
  
  /*
  *reflex_plus_cloud = *palm_cloud + *finger1_cloud;
  *reflex_plus_cloud = *reflex_plus_cloud + *finger2_cloud;
  *reflex_plus_cloud = *reflex_plus_cloud + *finger3_cloud;
  */
  
  /*
  Eigen::Vector3f hand_translation;
  hand_translation << -0.0091, 0, 0.095+0.02;
  Eigen::Matrix3f hand_rotation = Rotz_float(M_PI);
  Eigen::Matrix4f hand_transform;
  hand_transform << hand_rotation, hand_translation,
               0, 0, 0, 1;
  
  pcl::transformPointCloud(*reflex_plus_cloud, *reflex_plus_cloud, hand_transform);
  */
  
  /*
  // modify the hand cloud : shift the zero point to be identical with the kinematic model we use
  for(unsigned int i=0;i<reflex_plus_cloud->points.size();i++){
    reflex_plus_cloud->points[i].x -= 0.0098;
    reflex_plus_cloud->points[i].x -= 0.0091;
    
    reflex_plus_cloud->points[i].z += 0.095;
    reflex_plus_cloud->points[i].z += 0.02;
  }
  */
  
  /*
  Eigen::Vector3f camera_translation;
  camera_translation << -0.0673, 0, -0.02;
  Eigen::Matrix3f camera_rotation = Roty_float(-M_PI/2);
  Eigen::Matrix4f camera_transform;
  camera_transform << camera_rotation, camera_translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*camera_cloud, *camera_cloud, camera_transform);
  
  Eigen::Vector3f connection_translation;
  connection_translation << 0, 0, 0;
  Eigen::Matrix3f connection_rotation = Rotz_float(M_PI/2);
  Eigen::Matrix4f connection_transform;
  connection_transform << connection_rotation, connection_translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*connection_cloud, *connection_cloud, connection_transform);
  
  
  *reflex_plus_cloud += *connection_cloud;
  *reflex_plus_cloud += *camera_cloud;
  */
  
  
  /*
  Eigen::Matrix4f inverse_hand_transform;
  inverse_hand_transform << hand_rotation.transpose(), -hand_rotation.transpose()*hand_translation,  // from khalil's book page 21
                              0, 0, 0, 1;
  pcl::transformPointCloud(*reflex_plus_cloud, *reflex_plus_cloud, inverse_hand_transform);
  
  
  // modify the hand cloud : shift the zero point to be identical with the kinematic model we use
  for(unsigned int i=0;i<reflex_plus_cloud->points.size();i++){
    reflex_plus_cloud->points[i].x -= 0.0113;
    reflex_plus_cloud->points[i].z += 0.095;
  }
  */
  
  
  
  // save the reflex plus gripper cloud data
  pcl::io::savePCDFileASCII("reflex_plus_model_cloud_plus_camera.pcd", *reflex_plus_cloud);
  
  
  viewer->addPointCloud( reflex_plus_cloud, "reflex plus point cloud" );
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reflex plus point cloud");
  viewer->addCoordinateSystem(0.05);
  
  while (!viewer->wasStopped()){
    viewer->spinOnce();
  }
  
  return 0;
}

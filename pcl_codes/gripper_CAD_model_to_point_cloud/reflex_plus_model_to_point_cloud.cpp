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
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/base_link.ply", *base_link_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/distal_1.ply", *distal_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/distal_pad_1.ply", *distal_pad_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/flex_block.ply", *flex_block_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/pad.ply", *pad_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/proximal_1.ply", *proximal_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/proximal_pad_1.ply", *proximal_pad_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/swivel_1.ply", *swivel_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/swivel_2.ply", *swivel_2_cloud);
  
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/reflex_plus_realsense_connection.ply", *connection_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("../gripper_model/reflex_plus_meshes/realsense_d435.ply", *camera_cloud);
  
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f relative_transform = Eigen::Affine3f::Identity();
  
  Eigen::Affine3f swivel_1_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f proximal_1_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f proximal_pad_1_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f distal_1_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f distal_pad_1_transform = Eigen::Affine3f::Identity();
  
  Eigen::Affine3f swivel_2_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f proximal_2_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f proximal_pad_2_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f distal_2_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f distal_pad_2_transform = Eigen::Affine3f::Identity();
  
  Eigen::Affine3f proximal_3_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f proximal_pad_3_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f distal_3_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f distal_pad_3_transform = Eigen::Affine3f::Identity();
  
  
  
  
  //
  // palm construction
  // link1
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_link_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*base_link_cloud, *base_link_transformed_cloud, transform);
  
  // link2
  transform.translation() << 0.02, 0, 0.063;
  transform.rotate( Eigen::AngleAxisf( 1.5707963267949, Eigen::Vector3f::UnitX() ) );
  transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  transform.rotate( Eigen::AngleAxisf( -1.5707963267949, Eigen::Vector3f::UnitY() ) );
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pad_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*pad_cloud, *pad_transformed_cloud, transform);
  
  *palm_cloud = *base_link_transformed_cloud + *pad_transformed_cloud;
  
  
  
  
  // finger#1
  // swivel_1
  swivel_1_transform.translation() << 0.0503973683071414, -0.026, 0.063;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr swivel_1_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*swivel_1_cloud, *swivel_1_transformed_cloud, swivel_1_transform);
  
  // proximal_1
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0.01, 0, 0.0186;
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  proximal_1_transform = swivel_1_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.011, 0.007, 0.011;
  relative_transform.rotate( Eigen::AngleAxisf( 3.1459, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = proximal_1_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_1_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*proximal_1_cloud, *proximal_1_transformed_cloud, relative_transform);
  
  // proximal_pad_1
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitY() ) );
  proximal_pad_1_transform = proximal_1_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.008, 0.014, -0.002;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = proximal_pad_1_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_pad_1_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*proximal_pad_1_cloud, *proximal_pad_1_transformed_cloud, relative_transform);
  
  
  // distal_1
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0.076, 0, 0.0025;
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  distal_1_transform = proximal_1_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.077, 0.007, 0.007;
  relative_transform.rotate( Eigen::AngleAxisf( 3.1459, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = distal_1_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_1_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*distal_1_cloud, *distal_1_transformed_cloud, relative_transform);
  
  
  // distal_pad_1
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.01, 0, 0.001;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitY() ) );
  distal_pad_1_transform = distal_1_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.00825, 0.009, 0.057;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = distal_pad_1_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_pad_1_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*distal_pad_1_cloud, *distal_pad_1_transformed_cloud, relative_transform);
  
  *finger1_cloud = *swivel_1_transformed_cloud + *proximal_1_transformed_cloud;
  *finger1_cloud = *finger1_cloud + *proximal_pad_1_transformed_cloud;
  *finger1_cloud = *finger1_cloud + *distal_1_transformed_cloud;
  *finger1_cloud = *finger1_cloud + *distal_pad_1_transformed_cloud;
  
  
  
  
  // finger#2
  // swivel_2
  swivel_2_transform.translation() << 0.0503973683071414, 0.026, 0.063;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr swivel_2_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*swivel_1_cloud, *swivel_2_transformed_cloud, swivel_2_transform);
  
  // proximal_2
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0.01, 0, 0.0186;
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  proximal_2_transform = swivel_2_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.011, 0.007, 0.011;
  relative_transform.rotate( Eigen::AngleAxisf( 3.1459, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = proximal_2_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_2_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*proximal_1_cloud, *proximal_2_transformed_cloud, relative_transform);
  
  // proximal_pad_2
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitY() ) );
  proximal_pad_2_transform = proximal_2_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.008, 0.014, -0.002;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = proximal_pad_2_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_pad_2_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*proximal_pad_1_cloud, *proximal_pad_2_transformed_cloud, relative_transform);
  
  
  // distal_2
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0.076, 0, 0.0025;
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  distal_2_transform = proximal_2_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.077, 0.007, 0.007;
  relative_transform.rotate( Eigen::AngleAxisf( 3.1459, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = distal_2_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_2_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*distal_1_cloud, *distal_2_transformed_cloud, relative_transform);
  
  
  // distal_pad_2
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.01, 0, 0.001;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitY() ) );
  distal_pad_2_transform = distal_2_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.00825, 0.009, 0.057;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = distal_pad_2_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_pad_2_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*distal_pad_1_cloud, *distal_pad_2_transformed_cloud, relative_transform);
  
  *finger2_cloud = *swivel_2_transformed_cloud + *proximal_2_transformed_cloud;
  *finger2_cloud = *finger2_cloud + *proximal_pad_2_transformed_cloud;
  *finger2_cloud = *finger2_cloud + *distal_2_transformed_cloud;
  *finger2_cloud = *finger2_cloud + *distal_pad_2_transformed_cloud;
  
  
  
  // finger#3
  // proximal_3
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.03, 0, 0.08;
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 3.14159, Eigen::Vector3f::UnitY() ) );
  proximal_3_transform = relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.011, 0.007, 0.011;
  relative_transform.rotate( Eigen::AngleAxisf( 3.1459, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = proximal_3_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_3_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*proximal_1_cloud, *proximal_3_transformed_cloud, relative_transform);
  
  // proximal_pad_3
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0, 0, 0;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitY() ) );
  proximal_pad_3_transform = proximal_3_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.008, 0.014, -0.002;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitZ() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = proximal_pad_3_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proximal_pad_3_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*proximal_pad_1_cloud, *proximal_pad_3_transformed_cloud, relative_transform);
  
  
  // distal_3
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << 0.076, 0, 0.0025;
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  distal_3_transform = proximal_3_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.077, 0.007, 0.007;
  relative_transform.rotate( Eigen::AngleAxisf( 3.1459, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = distal_3_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_3_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*distal_1_cloud, *distal_3_transformed_cloud, relative_transform);
  
  
  // distal_pad_3
  // joint transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.01, 0, 0.001;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitY() ) );
  distal_pad_3_transform = distal_3_transform*relative_transform;
  // origin transform
  relative_transform = Eigen::Affine3f::Identity();
  relative_transform.translation() << -0.00825, 0.009, 0.057;
  relative_transform.rotate( Eigen::AngleAxisf( 1.5708, Eigen::Vector3f::UnitX() ) );
  relative_transform.rotate( Eigen::AngleAxisf( -1.5708, Eigen::Vector3f::UnitZ() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
  relative_transform.rotate( Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY() ) );
  relative_transform = distal_pad_3_transform*relative_transform;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr distal_pad_3_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*distal_pad_1_cloud, *distal_pad_3_transformed_cloud, relative_transform);
  
  *finger3_cloud = *proximal_3_transformed_cloud + *proximal_pad_3_transformed_cloud;
  *finger3_cloud = *finger3_cloud + *distal_3_transformed_cloud;
  *finger3_cloud = *finger3_cloud + *distal_pad_3_transformed_cloud;
  
  
  
  //
  // concatenate point clouds
  *reflex_plus_cloud = *palm_cloud + *finger1_cloud;
  *reflex_plus_cloud += *finger2_cloud;
  *reflex_plus_cloud += *finger3_cloud;
  
  
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
  
  
  Eigen::Vector3f camera_translation;
  camera_translation << 0, 0.0673, -0.02-0.005-0.012;
  Eigen::Matrix3f camera_rotation = Rotz_float(-M_PI/2)*Roty_float(-M_PI/2);
  Eigen::Matrix4f camera_transform;
  camera_transform << camera_rotation, camera_translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*camera_cloud, *camera_cloud, camera_transform);
  
  Eigen::Vector3f connection_translation;
  connection_translation << 0, 0, -0.005-0.012;
  Eigen::Matrix3f connection_rotation = Rotz_float(0);
  Eigen::Matrix4f connection_transform;
  connection_transform << connection_rotation, connection_translation,
               0, 0, 0, 1;
  pcl::transformPointCloud(*connection_cloud, *connection_cloud, connection_transform);
  //*reflex_plus_cloud += *connection_cloud;
  //*reflex_plus_cloud += *camera_cloud;
  
  
  
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
  pcl::io::savePCDFileASCII("../gripper_point_cloud/reflex_plus_model_cloud_plus_camera.pcd", *reflex_plus_cloud);
  
  
  viewer->addPointCloud( reflex_plus_cloud, "reflex plus point cloud" );
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reflex plus point cloud");
  viewer->addCoordinateSystem(0.02);
  
  while (!viewer->wasStopped()){
    viewer->spinOnce();
  }
  
  return 0;
}

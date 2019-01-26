#include <ros/ros.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>

#include "../../ros_packages/include/Eigen/Dense"

int main(){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_link_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_0_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_1_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_2_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_3_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_3_tip_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_12_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_13_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_14_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_15_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr link_15_tip_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr index_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pinky_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("allegro hand point cloud"));
  viewer->setBackgroundColor(255,255,255);
  std::string id = "cloud";
  
  // load .ply meshes, we first generate .ply file from the .stl file using meshlab software
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/base_link.ply", *base_link_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_0.0.ply", *link_0_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_1.0.ply", *link_1_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_2.0.ply", *link_2_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_3.0.ply", *link_3_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_3.0_tip.ply", *link_3_tip_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_12.0_right.ply", *link_12_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_13.0.ply", *link_13_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_14.0.ply", *link_14_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_15.0.ply", *link_15_cloud);
  pcl::io::loadPLYFile<pcl::PointXYZRGB>("allego_right_hand_meshes/link_15.0_tip.ply", *link_15_tip_cloud);
  
  
  
  // thumb finger construction
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
  
  *thumb_cloud = *link_12_transformed_cloud + *link_13_transformed_cloud;
  *thumb_cloud = *thumb_cloud + *link_14_transformed_cloud;
  *thumb_cloud = *thumb_cloud + *link_15_transformed_cloud;
  *thumb_cloud = *thumb_cloud + *link_15_tip_transformed_cloud;
  
  
  
  
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
  
  *index_cloud = *link_0_transformed_cloud + *link_1_transformed_cloud;
  *index_cloud = *index_cloud + *link_2_transformed_cloud;
  *index_cloud = *index_cloud + *link_3_transformed_cloud;
  *index_cloud = *index_cloud + *link_3_tip_transformed_cloud;
  
  
  
  
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
  
  *middle_cloud = *link_0_transformed_cloud + *link_1_transformed_cloud;
  *middle_cloud = *middle_cloud + *link_2_transformed_cloud;
  *middle_cloud = *middle_cloud + *link_3_transformed_cloud;
  *middle_cloud = *middle_cloud + *link_3_tip_transformed_cloud;
  
  
  
  
  // pinky finger construction
  // link1
  transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, -0.0435, -0.001542;
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(5.0), Eigen::Vector3f::UnitX() ) );   // roll-pitch-yaw used here is most probably is rotation about 1.x then 2.y then 3.x
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
  
  *pinky_cloud = *link_0_transformed_cloud + *link_1_transformed_cloud;
  *pinky_cloud = *pinky_cloud + *link_2_transformed_cloud;
  *pinky_cloud = *pinky_cloud + *link_3_transformed_cloud;
  *pinky_cloud = *pinky_cloud + *link_3_tip_transformed_cloud;
  
  
  
  
  
  
  
  // concatenate point clouds
  *allegro_hand_cloud = *base_link_cloud + *thumb_cloud;
  *allegro_hand_cloud = *allegro_hand_cloud + *index_cloud;
  *allegro_hand_cloud = *allegro_hand_cloud + *middle_cloud;
  *allegro_hand_cloud = *allegro_hand_cloud + *pinky_cloud;
  
  // modify the hand cloud : shift the zero point upwards by 95mm to be inline with the kinematic model we use
  // TODO : shift in +ve x-axis as well
  for(unsigned int i=0;i<allegro_hand_cloud->points.size();i++){
    allegro_hand_cloud->points[i].z += 0.095;
  }
  
  // save the allegro hand cloud data
  pcl::io::savePCDFileASCII("allegro_right_hand_model_cloud.pcd", *allegro_hand_cloud);
  
  
  viewer->addPointCloud( allegro_hand_cloud, "allegro hand point cloud" );
  viewer->addCoordinateSystem(0.05);
  
  while (!viewer->wasStopped()){
    viewer->spinOnce();
  }
  
  return 0;
}

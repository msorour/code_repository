#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h> // for sleep

#include <pcl/common/transforms.h>
#include "include/useful_implementations.h"

int main(int argc, char **argv){
  std::string cloud_file1_name = argv[1];
  std::string cloud_file2_name = argv[2];
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloud_file1_name, *cloud1);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloud_file2_name, *cloud2);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("cloud"));
  viewer->setBackgroundColor(255,255,255);
  viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  viewer->addPointCloud(augmented_cloud,rgb,"cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  
  Eigen::Vector3f transform1_trans;
  Eigen::Matrix3f transform1_rotation;
  Eigen::Matrix4f transform1_inv;
  Eigen::Matrix4f transform1;
  transform1 << 
   0.655757, -0.666768,  0.354123, -0.353283,
  -0.747378, -0.639681,  0.179539,  0.371874,
   0.106815, -0.382398, -0.917803,  0.430491,
          0,         0,         0,         1;
  transform1_trans <<-0.404678,0.334013,0.380559;
  transform1_rotation << 
   0.746606, -0.584638,  0.317456,
  -0.657026, -0.722867,  0.213965,
   0.104387, -0.368324, -0.923819;
  transform1_inv << transform1_rotation.transpose(), -transform1_rotation.transpose()*transform1_trans,
                    0,0,0,1;
  
  Eigen::Matrix4f transform2; 
  transform2 << 
   0.67918,  -0.727294, -0.0987814,  -0.406854,
  -0.67182,  -0.561812,  -0.482726,   0.378553,
  0.295587,   0.394221,  -0.870183,   0.295187,
         0,          0,          0,          1;
  
  Eigen::Vector3f translation;
  Eigen::Matrix3f rotation;
  Eigen::Matrix4f transform;
  
  Eigen::Vector3f dummy_translation;
  Eigen::Matrix3f dummy_rotation;
  Eigen::Matrix4f dummy_transform;
  
  
  Eigen::Vector3f camera_optical_frame_wrt_arm_link8_frame_translation;   camera_optical_frame_wrt_arm_link8_frame_translation << 0.03338934, -0.07033271, -0.02732752;
  Eigen::Matrix4f camera_optical_frame_wrt_arm_link8_frame_transform;
  camera_optical_frame_wrt_arm_link8_frame_transform << Eigen::Matrix3f::Identity(), camera_optical_frame_wrt_arm_link8_frame_translation,
                                                        0,0,0,1;
  dummy_translation << 0,0,0;
  dummy_rotation = Rotz_float(M_PI/4);
  dummy_transform << dummy_rotation, dummy_translation,
                     0,0,0,1;
  camera_optical_frame_wrt_arm_link8_frame_transform = camera_optical_frame_wrt_arm_link8_frame_transform*dummy_transform;
  //transform = camera_optical_frame_wrt_arm_link8_frame_transform.inverse()*transform1.inverse()*transform2*camera_optical_frame_wrt_arm_link8_frame_transform;
  
  
  transform = transform1.inverse()*transform2;
  std::cout << "relative transform = " << std::endl << transform << std::endl; 
  //pcl::transformPointCloud(*cloud2, *cloud2, transform);
  
  *augmented_cloud = *cloud1;
  *augmented_cloud += *cloud2;
  
  viewer->updatePointCloud(augmented_cloud,rgb,"cloud");
  /*
  Eigen::Quaternionf q;
  q.x() = ;
  q.y() = ;
  q.z() = ;
  q.w() = ;
  rotation = q.normalize().toRotationMatrix();
  */
  
  
  
  
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
  } 
  return 0;
}

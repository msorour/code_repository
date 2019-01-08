#include <ros/ros.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_lib_io.h>

#include <pcl/io/ply_io.h>

int main(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *mycloud);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  std::string id = "cloud";
  //viewer->addPointCloud(mycloud,id);
  
  
  // till now we can handle .ply files, we first generate .ply file from the .stl file using meshlab software
  // converting .ply mesh model into point cloud then visualize
  if(pcl::io::loadPLYFile<pcl::PointXYZ> ("cocacola_bottle.ply", *mycloud) == -1)
    std::cout<<"error reading .ply file"<<std::endl;
  
  viewer->addPointCloud(mycloud,id);
  
  // save the point cloud
  /*
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  point_cloud.height   = 1;
  point_cloud.width    = mycloud->size();
  point_cloud.is_dense = false;
	point_cloud.points.resize(point_cloud.width * point_cloud.height);
  */
  pcl::io::savePCDFileASCII("cocacola_bottle.pcd", *mycloud);
  
  while (!viewer->wasStopped()){
    viewer->spinOnce();
  }
  
  return 0;
}

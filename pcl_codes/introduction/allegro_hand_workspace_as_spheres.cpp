#include <iostream>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h>   // for sleep

int main(){
  std::string finger = "thumb";
  std::vector<std::string> finger_list;
  finger_list.push_back("thumb");
  finger_list.push_back("index");
  finger_list.push_back("middle");
  finger_list.push_back("pinky");
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_workspace_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_workspace_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_as_spheres_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_workspace_as_spheres_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // loading allegro hand model point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr allegro_hand_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/allegro_right_hand_model_cloud.pcd", *allegro_hand_cloud_xyz);
  copyPointCloud( *allegro_hand_cloud_xyz, *allegro_hand_cloud_xyzrgb);
  // modify the hand cloud : shift the zero point upwards by 95mm to be inline with the kinematic model we use
  for(unsigned int i=0;i<allegro_hand_cloud_xyz->points.size();i++){
    allegro_hand_cloud_xyzrgb->points[i].z += 0.095;
  }
  *augmented_cloud = *allegro_hand_cloud_xyzrgb;
  
  // to save hand + workspace spheres
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_workspace_spheres_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  *hand_workspace_spheres_xyzrgb = *allegro_hand_cloud_xyzrgb;
  
  // for visualization of point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("cloud viewer"));
  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  
  viewer->addPointCloud(augmented_cloud,"cloud");
  

  for(int j=0; j<4; j++){
    finger = finger_list[j];
    
    // loading finger workspace point cloud
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/"+finger+"_workspace.pcd", *finger_workspace_cloud_xyz);
    
    // loading finger workspace as spheres point cloud
    pcl::io::loadPCDFile<pcl::PointXYZ>(finger+"_workspace_as_spheres.pcd", *finger_workspace_as_spheres_cloud_xyz);
    
    // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
    copyPointCloud(*finger_workspace_cloud_xyz, *finger_workspace_cloud_xyzrgb);
    copyPointCloud(*finger_workspace_as_spheres_cloud_xyz, *finger_workspace_as_spheres_cloud_xyzrgb);
    
    //
    unsigned int r=0, g=0, b=0;
    if( finger == "thumb" )     { r=255;  g=0;    b=0;   }
    else if( finger == "index" ){ r=0;    g=255;  b=0;   }
    else if( finger == "middle"){ r=0;    g=0;    b=255; }
    else if( finger == "pinky" ){ r=100;  g=100;  b=100; }
    
    for(unsigned int i=0; i<finger_workspace_cloud_xyzrgb->size(); i++){
      finger_workspace_cloud_xyzrgb->points[i].r = r;
      finger_workspace_cloud_xyzrgb->points[i].g = g;
      finger_workspace_cloud_xyzrgb->points[i].b = b;
    }
    
    for(unsigned int i=0; i<finger_workspace_as_spheres_cloud_xyzrgb->size(); i++){
      finger_workspace_as_spheres_cloud_xyzrgb->points[i].r = r;
      finger_workspace_as_spheres_cloud_xyzrgb->points[i].g = g;
      finger_workspace_as_spheres_cloud_xyzrgb->points[i].b = b;
    }
    
    *hand_workspace_cloud_xyzrgb += *finger_workspace_cloud_xyzrgb;
    
    *augmented_cloud += *finger_workspace_cloud_xyzrgb;
    *augmented_cloud += *finger_workspace_as_spheres_cloud_xyzrgb;
    
    *hand_workspace_spheres_xyzrgb += *finger_workspace_as_spheres_cloud_xyzrgb;
    
    viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
    viewer->spinOnce();
  }
  
  // save hand workspace spheres point cloud
	//pcl::io::savePCDFileASCII("allegro_hand_workspace_as_spheres.pcd", *augmented_cloud);
	pcl::io::savePCDFileASCII("allegro_hand_workspace_as_spheres.pcd", *hand_workspace_spheres_xyzrgb);
	pcl::io::savePCDFileASCII("allegro_hand_workspace.pcd", *hand_workspace_cloud_xyzrgb);
	
	std::cout << "finished !" << std::endl;
	
  //while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    //Hullviewer->spinOnce(); 
  } 
  return 0;
}

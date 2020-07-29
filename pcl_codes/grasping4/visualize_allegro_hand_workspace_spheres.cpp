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
#include "include/grasping_algorithm.h"

int main(int argc, char **argv){
  
  // declarations for allegro hand
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  thumb_workspace_cloud               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  thumb_workspace_spheres             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  thumb_workspace_spheres_offset      (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  thumb_workspace_spheres_parameter   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  index_workspace_cloud               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  index_workspace_spheres             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  index_workspace_spheres_offset      (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  index_workspace_spheres_parameter   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  middle_workspace_cloud              (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  middle_workspace_spheres            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  middle_workspace_spheres_offset     (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  middle_workspace_spheres_parameter  (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pinky_workspace_cloud               (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pinky_workspace_spheres             (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pinky_workspace_spheres_offset      (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pinky_workspace_spheres_parameter   (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  dummy_cloud_xyzrgb                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  allegro_hand_workspace_cloud        (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  pcl::PointXYZRGB offset;
  pcl::PointXYZRGB parameter;
  Eigen::Vector3f parameter_vector, offset_vector;
  
  //pcl::io::loadPCDFile<pcl::PointXYZRGB>("../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd", *allegro_hand_workspace_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera.pcd", *allegro_hand_workspace_cloud);
  
  // finger workspace cloud
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../finger_workspace_pcd/thumb_workspace.pcd", *thumb_workspace_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../finger_workspace_pcd/index_workspace.pcd", *index_workspace_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../finger_workspace_pcd/middle_workspace.pcd", *middle_workspace_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("../finger_workspace_pcd/pinky_workspace.pcd", *pinky_workspace_cloud);
  
  
  // for visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("workspace spheres visulaizer"));
  viewer->setCameraPosition(0.639325, 0.420321, 0.468681, 0.0332625, 0.0307859, 0.101597, -0.356538, -0.284035, 0.890059, 0);
  viewer->addCoordinateSystem(0.1);
  viewer->setBackgroundColor(255,255,255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> black  (allegro_hand_workspace_cloud, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(allegro_hand_workspace_cloud, black, "hand cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "hand cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red  (thumb_workspace_spheres, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(thumb_workspace_spheres, red, "thumb spheres");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "thumb spheres");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red2  (thumb_workspace_cloud, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(thumb_workspace_cloud, red2, "thumb workspace");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "thumb workspace");
  
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green  (index_workspace_spheres, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(index_workspace_spheres, green, "index spheres");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "index spheres");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green2  (index_workspace_cloud, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(index_workspace_cloud, green2, "index workspace");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "index workspace");
  
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue  (middle_workspace_spheres, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB>(middle_workspace_spheres, blue, "middle spheres");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "middle spheres");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue2  (middle_workspace_cloud, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(middle_workspace_cloud, blue2, "middle workspace");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "middle workspace");
  
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey  (pinky_workspace_spheres, 100, 100, 100);
  viewer->addPointCloud<pcl::PointXYZRGB>(pinky_workspace_spheres, grey, "pinky spheres");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pinky spheres");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey2  (pinky_workspace_cloud, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(pinky_workspace_cloud, grey2, "pinky workspace");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pinky workspace");
  
  
  
  
  std::vector<double> data;
  std::string line;
  ifstream workspace_spheres_file_name;
  
  // thumb
  workspace_spheres_file_name.open("../finger_workspace_spheres/thumb_workspace_spheres_15_1.7.txt");
  int line_count = 0;
  if(workspace_spheres_file_name.is_open()){
    while(getline( workspace_spheres_file_name, line ) ){
      std::istringstream string_stream( line );
      std::string field;
      data.clear();
      while(string_stream){
        if(!getline( string_stream, field, ',' )) break;
        std::stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;
        data.push_back( f );
      }
      if(line_count==0){}
      else{
        parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
        thumb_workspace_spheres_offset->points.push_back( offset );
        thumb_workspace_spheres_parameter->points.push_back( parameter );
      }
      line_count++;
    }
  }
  workspace_spheres_file_name.close();
  // draw the point cloud of the workspace spheres
  for(unsigned int j=0; j<thumb_workspace_spheres_offset->size(); j++){
    parameter_vector << thumb_workspace_spheres_parameter->points[j].x, thumb_workspace_spheres_parameter->points[j].y, thumb_workspace_spheres_parameter->points[j].z;
    offset_vector    << thumb_workspace_spheres_offset->points[j].x, thumb_workspace_spheres_offset->points[j].y, thumb_workspace_spheres_offset->points[j].z;
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
    *thumb_workspace_spheres += *dummy_cloud_xyzrgb;
  }
  viewer->updatePointCloud<pcl::PointXYZRGB>(thumb_workspace_spheres, red, "thumb spheres");
  
  // index
  //workspace_spheres_file_name.open("../finger_workspace_spheres/index_workspace_spheres_10_1.txt");
  workspace_spheres_file_name.open("../finger_workspace_spheres/index_workspace_spheres_15_1.txt");
  line_count = 0;
  if(workspace_spheres_file_name.is_open()){
    while(getline( workspace_spheres_file_name, line ) ){
      std::istringstream string_stream( line );
      std::string field;
      data.clear();
      while(string_stream){
        if(!getline( string_stream, field, ',' )) break;
        std::stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;
        data.push_back( f );
      }
      if(line_count==0){}
      else{
        parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
        index_workspace_spheres_offset->points.push_back( offset );
        index_workspace_spheres_parameter->points.push_back( parameter );
      }
      line_count++;
    }
  }
  workspace_spheres_file_name.close();
  // draw the point cloud of the workspace spheres
  for(unsigned int j=0; j<index_workspace_spheres_offset->size(); j++){
    parameter_vector << index_workspace_spheres_parameter->points[j].x, index_workspace_spheres_parameter->points[j].y, index_workspace_spheres_parameter->points[j].z;
    offset_vector    << index_workspace_spheres_offset->points[j].x, index_workspace_spheres_offset->points[j].y, index_workspace_spheres_offset->points[j].z;
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
    *index_workspace_spheres += *dummy_cloud_xyzrgb;
  }
  viewer->updatePointCloud<pcl::PointXYZRGB>(index_workspace_spheres, green, "index spheres");
  
  // middle
  //workspace_spheres_file_name.open("../finger_workspace_spheres/middle_workspace_spheres_10_1.txt");
  workspace_spheres_file_name.open("../finger_workspace_spheres/middle_workspace_spheres_15_1.txt");
  line_count = 0;
  if(workspace_spheres_file_name.is_open()){
    while(getline( workspace_spheres_file_name, line ) ){
      std::istringstream string_stream( line );
      std::string field;
      data.clear();
      while(string_stream){
        if(!getline( string_stream, field, ',' )) break;
        std::stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;
        data.push_back( f );
      }
      if(line_count==0){}
      else{
        parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
        middle_workspace_spheres_offset->points.push_back( offset );
        middle_workspace_spheres_parameter->points.push_back( parameter );
      }
      line_count++;
    }
  }
  workspace_spheres_file_name.close();
  // draw the point cloud of the workspace spheres
  for(unsigned int j=0; j<middle_workspace_spheres_offset->size(); j++){
    parameter_vector << middle_workspace_spheres_parameter->points[j].x, middle_workspace_spheres_parameter->points[j].y, middle_workspace_spheres_parameter->points[j].z;
    offset_vector    << middle_workspace_spheres_offset->points[j].x, middle_workspace_spheres_offset->points[j].y, middle_workspace_spheres_offset->points[j].z;
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
    *middle_workspace_spheres += *dummy_cloud_xyzrgb;
  }
  viewer->updatePointCloud<pcl::PointXYZRGB>(middle_workspace_spheres, blue, "middle spheres");
  
  // pinky
  //workspace_spheres_file_name.open("../finger_workspace_spheres/pinky_workspace_spheres_10_1.txt");
  workspace_spheres_file_name.open("../finger_workspace_spheres/pinky_workspace_spheres_15_1.txt");
  line_count = 0;
  if(workspace_spheres_file_name.is_open()){
    while(getline( workspace_spheres_file_name, line ) ){
      std::istringstream string_stream( line );
      std::string field;
      data.clear();
      while(string_stream){
        if(!getline( string_stream, field, ',' )) break;
        std::stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;
        data.push_back( f );
      }
      if(line_count==0){}
      else{
        parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
        pinky_workspace_spheres_offset->points.push_back( offset );
        pinky_workspace_spheres_parameter->points.push_back( parameter );
      }
      line_count++;
    }
  }
  workspace_spheres_file_name.close();
  // draw the point cloud of the workspace spheres
  for(unsigned int j=0; j<pinky_workspace_spheres_offset->size(); j++){
    parameter_vector << pinky_workspace_spheres_parameter->points[j].x, pinky_workspace_spheres_parameter->points[j].y, pinky_workspace_spheres_parameter->points[j].z;
    offset_vector    << pinky_workspace_spheres_offset->points[j].x, pinky_workspace_spheres_offset->points[j].y, pinky_workspace_spheres_offset->points[j].z;
    construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
    *pinky_workspace_spheres += *dummy_cloud_xyzrgb;
  }
  viewer->updatePointCloud<pcl::PointXYZRGB>(pinky_workspace_spheres, grey, "pinky spheres");
  
  
  std::vector<pcl::visualization::Camera> cam;
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    viewer->saveScreenshot("../finger_workspace_spheres/allegro_hand_workspace_spheres.png");
    
    //viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  } 
  return 0;
}

/*
run this program using:
./step_1_downsampling_object_cloud tool_box.pcd 0.003

you can view all output segmented clusters using:
pcl_viewer tool_box_downsampled.pcd


reset && cmake .. && make &&  ./registering_and_downsampling_3_view_point_clouds ../raw_object_pcd_files/storage_bin_0.pcd ../raw_object_pcd_files/storage_bin_1.pcd ../raw_object_pcd_files/storage_bin_2.pcd 0.01
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <time.h>

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

#include <pcl/filters/voxel_grid.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "include/useful_implementations.h"
#include "include/grasping_algorithm.h"





#define MAXBUFSIZE  ((int) 1e6)

Eigen::MatrixXd readMatrix(const char *filename){
  int cols = 0, rows = 0;
  double buff[MAXBUFSIZE];

  // Read numbers from file into buffer.
  std::ifstream infile;
  infile.open(filename);
  while (! infile.eof()){
    std::string line;
    std::getline(infile, line);

    int temp_cols = 0;
    std::stringstream stream(line);
    while(! stream.eof())
      stream >> buff[cols*rows+temp_cols++];

    if (temp_cols == 0)
      continue;

    if (cols == 0)
      cols = temp_cols;

    rows++;
  }

  infile.close();

  rows--;

  // Populate matrix with numbers.
  Eigen::MatrixXd result(rows,cols);
  for (int i = 0; i < rows; i++)
  for (int j = 0; j < cols; j++)
  result(i,j) = buff[ cols*i+j ];

  return result;
};




int main (int argc, char** argv){
  std::string file_name1 = argv[1];
  std::string file_name2 = argv[2];
  std::string file_name3 = argv[3];
  std::string tf_matrix_file_name = argv[4];
  double leaf_size = std::stof( argv[5] );
  clock_t begin, end;
	double time_spent;
	
	begin = clock();
  
  Eigen::MatrixXd tf = readMatrix(tf_matrix_file_name.c_str());
  Eigen::Matrix4d tf1, tf2, tf3;
  tf1 << tf(0,0),  tf(0,1),  tf(0,2),  tf(0,3),
         tf(1,0),  tf(1,1),  tf(1,2),  tf(1,3),
         tf(2,0),  tf(2,1),  tf(2,2),  tf(2,3),
         tf(3,0),  tf(3,1),  tf(3,2),  tf(3,3);
  tf2 << tf(4,0),  tf(4,1),  tf(4,2),  tf(4,3),
         tf(5,0),  tf(5,1),  tf(5,2),  tf(5,3),
         tf(6,0),  tf(6,1),  tf(6,2),  tf(6,3),
         tf(7,0),  tf(7,1),  tf(7,2),  tf(7,3);
  tf3 << tf(8,0),  tf(8,1),  tf(8,2),  tf(8,3),
         tf(9,0),  tf(9,1),  tf(9,2),  tf(9,3),
         tf(10,0), tf(10,1), tf(10,2), tf(10,3),
         tf(11,0), tf(11,1), tf(11,2), tf(11,3);
  std::cout << "tf  = " << std::endl << tf  << std::endl;
  std::cout << "tf1 = " << std::endl << tf1 << std::endl;
  std::cout << "tf2 = " << std::endl << tf2 << std::endl;
  std::cout << "tf3 = " << std::endl << tf3 << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       dummy_cloud_xyz_1                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       dummy_cloud_xyz_2                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       dummy_cloud_xyz_3                             (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  
  // load the 3 view point clouds
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name1, *scene_cloud_xyz_1);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name2, *scene_cloud_xyz_2);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name3, *scene_cloud_xyz_3);
  std::cout << "scene cloud 1 size : " << scene_cloud_xyz_1->size() << std::endl;
  std::cout << "scene cloud 2 size : " << scene_cloud_xyz_2->size() << std::endl;
  std::cout << "scene cloud 3 size : " << scene_cloud_xyz_3->size() << std::endl;
  
  // for visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("point cloud visulaizer"));
  
  viewer->setCameraPosition(-0.582661, 0.49555, 0.175435, -0.129036, 0.102673, 0.469419, 0.468416, -0.114679, -0.876034, 0);
  viewer->addCoordinateSystem(0.15);
  viewer->setBackgroundColor(255,255,255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> black  (scene_cloud_xyz_1_transformed, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_1_transformed, black, "scene cloud 1");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud 1");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red  (scene_cloud_xyz_2_transformed, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_2_transformed, red, "scene cloud 2");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud 2");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue  (scene_cloud_xyz_3_transformed, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_3_transformed, blue, "scene cloud 3");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud 3");
  
  //viewer->updatePointCloud(scene_cloud_xyz_1, black, "scene cloud 1");
  //viewer->updatePointCloud(scene_cloud_xyz_2, red,   "scene cloud 2");
  //viewer->updatePointCloud(scene_cloud_xyz_3, blue,  "scene cloud 3");
  
  pcl::transformPointCloud(*scene_cloud_xyz_1, *scene_cloud_xyz_1_transformed, tf1);
  pcl::transformPointCloud(*scene_cloud_xyz_2, *scene_cloud_xyz_2_transformed, tf2);
  pcl::transformPointCloud(*scene_cloud_xyz_3, *scene_cloud_xyz_3_transformed, tf3);
  
  //viewer->updatePointCloud(scene_cloud_xyz_1_transformed, black, "scene cloud 1");
  //viewer->updatePointCloud(scene_cloud_xyz_2_transformed, red,   "scene cloud 2");
  viewer->updatePointCloud(scene_cloud_xyz_3_transformed, blue,  "scene cloud 3");
  
  
  /*
  std::cout << "PointCloud before downsampling has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter (*cloud_downsampled);
  std::cout << "PointCloud after downsampling has: " << cloud_downsampled->points.size ()  << " data points." << std::endl; //*
  
  std::string downsampled_file_name;
  downsampled_file_name = file_name.substr(0, file_name.size()-4);
  downsampled_file_name = downsampled_file_name + "_downsampled.pcd";
  
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>(downsampled_file_name, *cloud_downsampled, false);
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent in downsampling = " << time_spent << std::endl;
  */
  
  
  std::vector<pcl::visualization::Camera> cam;
  while( !viewer->wasStopped() ){
    viewer->spinOnce();
    //viewer->saveScreenshot(save_file_name);
    
    //viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  } 
  
  return (0);
}

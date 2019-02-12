/*
run this program using:
./step_1_downsampling_object_cloud tool_box.pcd 0.003

you can view all output segmented clusters using:
pcl_viewer tool_box_downsampled.pcd
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <time.h>

int main (int argc, char** argv){
  std::string file_name = argv[1];
  double leaf_size = std::stof( argv[2] );
  clock_t begin, end;
	double time_spent;
	
	begin = clock();
  
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read(file_name, *cloud);
  
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

  return (0);
}

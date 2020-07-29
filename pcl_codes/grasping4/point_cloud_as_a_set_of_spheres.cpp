// how to use:
// reset && cmake .. && make && ./point_cloud_as_a_set_of_spheres ../finger_workspace_pcd/middle_workspace.pcd false ../finger_workspace_spheres/middle_workspace_spheres 0.015 0.032 0.001 1.5 50 10 5 255 0 0

#include <iostream>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "include/useful_implementations.h"
#include "include/grasping_algorithm.h"


int main(int argc, char **argv){
  // inputs
  std::string workspace_cloud_file    = argv[1];
  std::string show_concave_hull       = argv[2];
  std::string save_spheres_file_name  = argv[3];
  
  double radius_of_smallest_sphere    = std::stod( argv[4] );
  double radius_of_largest_sphere     = std::stod( argv[5] );
  double sphere_radius_increment      = std::stod( argv[6] );
  double overlap_distance             = std::stod( argv[7] );
  int sphere_point_cloud_samples      = std::stoi( argv[8] );
  int desired_number_of_spheres       = std::stoi( argv[9] );
  int iterations                      = std::stoi( argv[10] );
  
  int red                             = std::stoi( argv[11] );
  int green                           = std::stoi( argv[12] );
  int blue                            = std::stoi( argv[13] );
  
  // point cloud declarations
  pcl::PointCloud<pcl::PointXYZ>::Ptr       finger_workspace_cloud_xyz                    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       finger_workspace_spheres_visualization_xyz    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       finger_workspace_spheres_filtered_xyz         (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PolygonMesh mesh_out;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  pcl::PointXYZ point_xyz;
  
  // load the finger workspace point cloud
  pcl::io::loadPCDFile<pcl::PointXYZ>(workspace_cloud_file, *finger_workspace_cloud_xyz);
  std::cout << "finger_workspace_cloud_xyz: " << finger_workspace_cloud_xyz->size() << std::endl;
  
  // for visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("point cloud visulaizer"));
  
  //viewer->setCameraPosition(0.478147, -0.142372, 0.435734, 0.0205253, 0.0659763, -0.00441103, -0.640249, 0.174233, 0.748147, 0);      // thumb
  viewer->setCameraPosition(0.518683, -0.44194, 0.433086, 0.00736115, 0.0825679, 0.090681, -0.322523, 0.276551, 0.905262, 0);           // middle
  viewer->addCoordinateSystem(0.15);
  viewer->setBackgroundColor(255,255,255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> black  (finger_workspace_cloud_xyz, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(finger_workspace_cloud_xyz, black, "workspace cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "workspace cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> workspace_spheres_color  (finger_workspace_spheres_visualization_xyz, red, green, blue);
  viewer->addPointCloud<pcl::PointXYZ>(finger_workspace_spheres_visualization_xyz, workspace_spheres_color, "finger spheres");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "finger spheres");
  
  
  // get the set of spheres filling the workspace point cloud,
  // giving the sphere radius and workspace point cloud, the output is set of spheres offset
  Eigen::Vector4f far_point_in_pos_direction_4d, far_point_in_neg_direction_4d;
  
  ///
  ///
  std::vector<double> sphere_radius_filtered;
  point_cloud_as_set_of_spheres( desired_number_of_spheres, finger_workspace_cloud_xyz, radius_of_smallest_sphere, radius_of_largest_sphere, sphere_radius_increment, overlap_distance, iterations, sphere_point_cloud_samples, finger_workspace_spheres_filtered_xyz, sphere_radius_filtered, finger_workspace_spheres_visualization_xyz, far_point_in_pos_direction_4d, far_point_in_neg_direction_4d );
  
  //point_cloud_as_set_of_spheres_fixed_radius_paper_photos( finger_workspace_cloud_xyz, 0.020, iterations, sphere_point_cloud_samples, finger_workspace_spheres_filtered_xyz, finger_workspace_spheres_visualization_xyz, far_point_in_pos_direction_4d, far_point_in_neg_direction_4d );
  //finger_workspace_spheres_visualization_xyz->clear();
  std::cout << "finger_workspace_spheres_filtered_xyz: " << finger_workspace_spheres_filtered_xyz->size() << std::endl;
  
  // drawing the far points
  //point_xyz.x = far_point_in_pos_direction_4d(0);   point_xyz.y = far_point_in_pos_direction_4d(1);   point_xyz.z = far_point_in_pos_direction_4d(2);
  //viewer->addSphere<pcl::PointXYZ>(point_xyz, 0.005, 1.0, 0.0, 0.0, "far point pos");
  //point_xyz.x = far_point_in_neg_direction_4d(0);   point_xyz.y = far_point_in_neg_direction_4d(1);   point_xyz.z = far_point_in_neg_direction_4d(2);
  //viewer->addSphere<pcl::PointXYZ>(point_xyz, 0.005, 0.0, 0.0, 1.0, "far point neg");
  
  std::cout << "finger_workspace_spheres_visualization_xyz: " << finger_workspace_spheres_visualization_xyz->size() << std::endl;
  viewer->updatePointCloud(finger_workspace_spheres_visualization_xyz, workspace_spheres_color, "finger spheres");
  
  
  if(show_concave_hull=="true"){
    // draw the concave hull to make sure no generated spheres are outside the workspace point cloud
    concave_hull.setInputCloud( finger_workspace_cloud_xyz );
    concave_hull.setAlpha( 0.01f );
    concave_hull.reconstruct( mesh_out );
    viewer->addPolygonMesh(mesh_out,"hull");
  }
  
  
  // to save the spheres data
  ofstream workspace_spheres;
  std::string file_name;
  file_name = save_spheres_file_name + std::string(".txt");
  workspace_spheres.open( file_name.c_str() );
  workspace_spheres << "r, "<<"offset_x, "<<"offset_y, "<<"offset_z"<<"\n";
  for(int i=0; i<finger_workspace_spheres_filtered_xyz->size(); i++)
    workspace_spheres << sphere_radius_filtered[i]<<", "<< finger_workspace_spheres_filtered_xyz->points[i].x<<", "<< finger_workspace_spheres_filtered_xyz->points[i].y<<", "<< finger_workspace_spheres_filtered_xyz->points[i].z<<"\n";
  workspace_spheres.close();
  
  
	std::string save_spheres_photo_file_name;
	save_spheres_photo_file_name = save_spheres_file_name + std::string(".png");
	//std::string save_file_name = "middle_point_cloud_plus_far_points.png";
	//std::string save_file_name = "middle_point_cloud_plus_far_points_plus_sphere_lattice.png";
	//std::string save_file_name = "middle_point_cloud_plus_far_points_plus_sphere_lattice_filtered.png";
	//std::string save_file_name = "middle_point_cloud_plus_sphere_lattice_filtered.png";
	//std::string save_file_name = "middle_point_cloud_plus_sphere_lattice_filtered_shifted.png";
	//std::string save_file_name = "middle_point_cloud_plus_sphere_lattice_filtered_shifted_filtered2.png";
	//std::string save_file_name = "middle_point_cloud_plus_spheres_best_3.png";
	//std::string save_file_name = "middle_point_cloud_plus_spheres_before_filtering_with_hull_vertices.png";
	//std::string save_file_name = "middle_point_cloud_plus_spheres_after_filtering_with_hull_vertices.png";
	
	std::vector<pcl::visualization::Camera> cam;
  while( !viewer->wasStopped() ){
    viewer->spinOnce();
    viewer->saveScreenshot(save_spheres_photo_file_name);
    
    //viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  } 
  return 0;
}

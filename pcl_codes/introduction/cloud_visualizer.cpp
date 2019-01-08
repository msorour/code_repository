#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h> // for sleep

int main(){
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr thumb_workspace_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_workspace_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/thumb_workspace.pcd", *thumb_workspace_cloud_xyz);
  // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
  copyPointCloud(*thumb_workspace_cloud_xyz, *thumb_workspace_cloud_xyzrgb);
  *augmented_cloud = *thumb_workspace_cloud_xyzrgb;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("cloud viewer"));
  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addPointCloud(augmented_cloud,"cloud");
  
  
  // generating the concave hull
  pcl::PolygonMesh mesh_out;
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud( thumb_workspace_cloud_xyz );
  concave_hull.setAlpha( 0.05f );
  concave_hull.reconstruct( mesh_out );
  
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud = *thumb_workspace_cloud_xyz;
  /*
  // centroid of the point cloud (workspace)
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for(unsigned int i=0;i<cloud.points.size();i++)
    centroid.add( cloud.points[i] );
  // Fetch centroid using "get()"
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  std::cout<< centroid_point << std::endl;  
  viewer->addCoordinateSystem (0.05,centroid_point.x, centroid_point.y, centroid_point.z);
  Hullviewer->addCoordinateSystem (0.05,centroid_point.x, centroid_point.y, centroid_point.z);
  */
  Hullviewer->setBackgroundColor(0,0,0);
  Hullviewer->addPolygonMesh(mesh_out,"hull");
  
  // save the polygon mesh
  pcl::io::saveOBJFile("thumb_workspace_concave_hull.obj", mesh_out );
  
  /*
  // find nearest vertex to the centroid
  double euclidean_distance=0.0;
  double smallest_euclidean_distance=1000000000;
  pcl::PointXYZ nearest_vertex_point;
  */
  pcl::PointCloud<pcl::PointXYZ> poly_mesh_vertices;
  pcl::fromPCLPointCloud2(mesh_out.cloud, poly_mesh_vertices);
  /*
  std::cout<< poly_mesh_vertices.points[0].x << std::endl;
  for(unsigned int i=0;i<poly_mesh_vertices.size();i++){
    euclidean_distance = (poly_mesh_vertices.points[i].x - centroid_point.x)*(poly_mesh_vertices.points[i].x - centroid_point.x) + 
                  (poly_mesh_vertices.points[i].y - centroid_point.y)*(poly_mesh_vertices.points[i].y - centroid_point.y) + 
                  (poly_mesh_vertices.points[i].z - centroid_point.z)*(poly_mesh_vertices.points[i].z - centroid_point.z);
    if(euclidean_distance < smallest_euclidean_distance){
      smallest_euclidean_distance = euclidean_distance;
      nearest_vertex_point.x = poly_mesh_vertices.points[i].x;
      nearest_vertex_point.y = poly_mesh_vertices.points[i].y;
      nearest_vertex_point.z = poly_mesh_vertices.points[i].z;
    }
  }
  smallest_euclidean_distance = sqrt(smallest_euclidean_distance);
  std::cout<< "smallest_euclidean_distance: " << smallest_euclidean_distance << std::endl;
  std::cout<< "corresponding vertex point: " << nearest_vertex_point << std::endl;
  */
  
  // searching for the biggest ellipsoid centered at the centroid that is enclosed inside the concave hull
  double a=0.0, b=0.0, c=0.0;   // ellipsoid half-length of principal axes
  double a_best=0.0, b_best=0.0, c_best=0.0;
  double a_best_global=0.0, b_best_global=0.0, c_best_global=0.0;
  double ellipsoid_value=1.0;
  double accuracy=0.001;
  double ellipsoid_x, ellipsoid_y, ellipsoid_z;
  double volumne_of_ellipsoid_best=0.0;
  double volumne_of_ellipsoid_best_global=0.0;
  int ellipsoid_point_cloud_samples = 70;
  unsigned int delay_microseconds = 1;
  pcl::PointCloud<pcl::PointXYZRGB> ellipsoid_point_cloud;
  pcl::PointXYZRGB ellipsoid_point;
  pcl::PointXYZ ellipsoid_offset;
  pcl::PointXYZ ellipsoid_offset_best_global;
  
  /*
  ellipsoid_offset.x = centroid_point.x;
  ellipsoid_offset.y = centroid_point.y;
  ellipsoid_offset.z = centroid_point.z;
  b=smallest_euclidean_distance/2;
  c=smallest_euclidean_distance/2;
  */
  
  // iterate through all points of the point cloud !
  for(unsigned int it=0; it<cloud.points.size(); it++){
  //for(unsigned int it=5000; it<5100; it++){
  std::cout<< "cloud point: " << cloud.points[it] << std::endl;
  ellipsoid_offset.x = cloud.points[it].x;
  ellipsoid_offset.y = cloud.points[it].y;
  ellipsoid_offset.z = cloud.points[it].z;
  a=0.01;
  b=0.01;
  c=0.01;
  
  
  // for this "seed" ellipsoid center point
  //
  // find largest possible "a" dimension
  for(int i=0; i<100; i++){
    a += accuracy;
    for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
      // equation of ellipsoid
      ellipsoid_value =   pow(poly_mesh_vertices.points[j].x - ellipsoid_offset.x, 2)/pow(a, 2) 
                        + pow(poly_mesh_vertices.points[j].y - ellipsoid_offset.y, 2)/pow(b, 2) 
                        + pow(poly_mesh_vertices.points[j].z - ellipsoid_offset.z, 2)/pow(c, 2);
      if( ellipsoid_value < 1.0 )
        break;
    }
    if( ellipsoid_value < 1.0 ){
      a_best = a - accuracy;
      //std::cout<< "a_best = " << a_best << ", found at iteration: " << i << std::endl;
      //std::cout<< "b=" << b << ", c=" << c << std::endl;
      break;
    }
    else{
      // generate and view the point cloud of the ellipsoid
      ellipsoid_point_cloud.clear();
      for(int k=0; k<ellipsoid_point_cloud_samples; k++){
        ellipsoid_x = (-a+ellipsoid_offset.x) + k*2*a/ellipsoid_point_cloud_samples;
        for(int l=0; l<ellipsoid_point_cloud_samples; l++){
          ellipsoid_y = (-b+ellipsoid_offset.y) + l*2*b/ellipsoid_point_cloud_samples;
          ellipsoid_z = ellipsoid_offset.z + c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b, 2) );
          ellipsoid_point.x = ellipsoid_x;
          ellipsoid_point.y = ellipsoid_y;
          ellipsoid_point.z = ellipsoid_z;
          ellipsoid_point.r = 250;
          ellipsoid_point.g = 0;
          ellipsoid_point.b = 0;
          ellipsoid_point_cloud.points.push_back( ellipsoid_point );
          
          ellipsoid_z = ellipsoid_offset.z - c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b, 2) );
          ellipsoid_point.x = ellipsoid_x;
          ellipsoid_point.y = ellipsoid_y;
          ellipsoid_point.z = ellipsoid_z;
          ellipsoid_point.r = 250;
          ellipsoid_point.g = 0;
          ellipsoid_point.b = 0;
          ellipsoid_point_cloud.points.push_back( ellipsoid_point );          
        }
      }
      augmented_cloud->clear();
      *augmented_cloud = *thumb_workspace_cloud_xyzrgb;
      *augmented_cloud += ellipsoid_point_cloud;
      // visualize the ellipsoid inside the original workspace point cloud
      viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
      viewer->spinOnce();
    }
    usleep(delay_microseconds);
  }
  
  //
  // find largest possible "b" dimension
  for(int i=0; i<100; i++){
    b += accuracy;
    for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
      // equation of ellipsoid
      ellipsoid_value =   pow(poly_mesh_vertices.points[j].x - ellipsoid_offset.x, 2)/pow(a_best, 2) 
                        + pow(poly_mesh_vertices.points[j].y - ellipsoid_offset.y, 2)/pow(b, 2) 
                        + pow(poly_mesh_vertices.points[j].z - ellipsoid_offset.z, 2)/pow(c, 2);
      if( ellipsoid_value < 1.0 )
        break;
    }
    if( ellipsoid_value < 1.0 ){
      b_best = b - accuracy;
      //std::cout<< "b_best = " << b_best << ", found at iteration: " << i << std::endl;
      //std::cout<< "a_best=" << a_best << ", c=" << c << std::endl;
      break;
    }
    else{
      // generate and view the point cloud of the ellipsoid
      ellipsoid_point_cloud.clear();
      for(int k=0; k<ellipsoid_point_cloud_samples; k++){
        ellipsoid_x = (-a_best+ellipsoid_offset.x) + k*2*a_best/ellipsoid_point_cloud_samples;
        for(int l=0; l<ellipsoid_point_cloud_samples; l++){
          ellipsoid_y = (-b+ellipsoid_offset.y) + l*2*b/ellipsoid_point_cloud_samples;
          ellipsoid_z = ellipsoid_offset.z + c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b, 2) );
          ellipsoid_point.x = ellipsoid_x;
          ellipsoid_point.y = ellipsoid_y;
          ellipsoid_point.z = ellipsoid_z;
          ellipsoid_point.r = 250;
          ellipsoid_point.g = 0;
          ellipsoid_point.b = 0;
          ellipsoid_point_cloud.points.push_back( ellipsoid_point );
          
          ellipsoid_z = ellipsoid_offset.z - c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b, 2) );
          ellipsoid_point.x = ellipsoid_x;
          ellipsoid_point.y = ellipsoid_y;
          ellipsoid_point.z = ellipsoid_z;
          ellipsoid_point.r = 250;
          ellipsoid_point.g = 0;
          ellipsoid_point.b = 0;
          ellipsoid_point_cloud.points.push_back( ellipsoid_point );          
        }
      }
      augmented_cloud->clear();
      *augmented_cloud = *thumb_workspace_cloud_xyzrgb;
      *augmented_cloud += ellipsoid_point_cloud;
      // visualize the ellipsoid inside the original workspace point cloud
      viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
      viewer->spinOnce();
    }
    usleep(delay_microseconds);
  }
  
  //
  // find largest possible "c" dimension
  for(int i=0; i<100; i++){
    c += accuracy;
    for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
      // equation of ellipsoid
      ellipsoid_value =   pow(poly_mesh_vertices.points[j].x - ellipsoid_offset.x, 2)/pow(a_best, 2) 
                        + pow(poly_mesh_vertices.points[j].y - ellipsoid_offset.y, 2)/pow(b_best, 2) 
                        + pow(poly_mesh_vertices.points[j].z - ellipsoid_offset.z, 2)/pow(c, 2);
      if( ellipsoid_value < 1.0 )
        break;
    }
    if( ellipsoid_value < 1.0 ){
      c_best = c - accuracy;
      //std::cout<< "c_best = " << c_best << ", found at iteration: " << i << std::endl;
      //std::cout<< "a_best=" << a_best << ", b_best=" << b_best << std::endl;
      break;
    }
    else{
      // generate and view the point cloud of the ellipsoid
      ellipsoid_point_cloud.clear();
      for(int k=0; k<ellipsoid_point_cloud_samples; k++){
        ellipsoid_x = (-a_best+ellipsoid_offset.x) + k*2*a_best/ellipsoid_point_cloud_samples;
        for(int l=0; l<ellipsoid_point_cloud_samples; l++){
          ellipsoid_y = (-b_best+ellipsoid_offset.y) + l*2*b_best/ellipsoid_point_cloud_samples;
          ellipsoid_z = ellipsoid_offset.z + c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b_best, 2) );
          ellipsoid_point.x = ellipsoid_x;
          ellipsoid_point.y = ellipsoid_y;
          ellipsoid_point.z = ellipsoid_z;
          ellipsoid_point.r = 250;
          ellipsoid_point.g = 0;
          ellipsoid_point.b = 0;
          ellipsoid_point_cloud.points.push_back( ellipsoid_point );
          
          ellipsoid_z = ellipsoid_offset.z - c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b_best, 2) );
          ellipsoid_point.x = ellipsoid_x;
          ellipsoid_point.y = ellipsoid_y;
          ellipsoid_point.z = ellipsoid_z;
          ellipsoid_point.r = 250;
          ellipsoid_point.g = 0;
          ellipsoid_point.b = 0;
          ellipsoid_point_cloud.points.push_back( ellipsoid_point );          
        }
      }
      augmented_cloud->clear();
      *augmented_cloud = *thumb_workspace_cloud_xyzrgb;
      *augmented_cloud += ellipsoid_point_cloud;
      // visualize the ellipsoid inside the original workspace point cloud
      viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
      viewer->spinOnce();
    }
    usleep(delay_microseconds);
  }
  
  
  // getting best global
  
  // must compute volume of ellipsoid and use it to evaluate best global !
  volumne_of_ellipsoid_best = (4/3)*3.14159*a_best*b_best*c_best;
  if(volumne_of_ellipsoid_best > volumne_of_ellipsoid_best_global){
    a_best_global = a_best;
    b_best_global = b_best;
    c_best_global = c_best;
    volumne_of_ellipsoid_best_global = volumne_of_ellipsoid_best;
    ellipsoid_offset_best_global = cloud.points[it];
  }
  
  std::cout<< "iteration: " <<it<< ", a_best_global=" <<a_best_global<< ", b_best_global=" <<b_best_global<< ", c_best_global=" <<c_best_global<< ", volumne_best_global=" <<volumne_of_ellipsoid_best_global<<std::endl;
  
  }
  std::cout<< "Finished iterations!" <<std::endl;
  
  
  // display the best ellipsoid subset of workspace
  ellipsoid_offset.x = ellipsoid_offset_best_global.x;
  ellipsoid_offset.y = ellipsoid_offset_best_global.y;
  ellipsoid_offset.z = ellipsoid_offset_best_global.z;
  ellipsoid_point_cloud.clear();
  for(int k=0; k<ellipsoid_point_cloud_samples; k++){
    ellipsoid_x = (-a_best_global+ellipsoid_offset.x) + k*2*a_best_global/ellipsoid_point_cloud_samples;
    for(int l=0; l<ellipsoid_point_cloud_samples; l++){
      ellipsoid_y = (-b_best_global+ellipsoid_offset.y) + l*2*b_best_global/ellipsoid_point_cloud_samples;
      ellipsoid_z = ellipsoid_offset.z + c_best_global*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best_global, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b_best_global, 2) );
      ellipsoid_point.x = ellipsoid_x;
      ellipsoid_point.y = ellipsoid_y;
      ellipsoid_point.z = ellipsoid_z;
      ellipsoid_point.r = 250;
      ellipsoid_point.g = 0;
      ellipsoid_point.b = 0;
      ellipsoid_point_cloud.points.push_back( ellipsoid_point );
      
      ellipsoid_z = ellipsoid_offset.z - c_best_global*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best_global, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b_best_global, 2) );
      ellipsoid_point.x = ellipsoid_x;
      ellipsoid_point.y = ellipsoid_y;
      ellipsoid_point.z = ellipsoid_z;
      ellipsoid_point.r = 250;
      ellipsoid_point.g = 0;
      ellipsoid_point.b = 0;
      ellipsoid_point_cloud.points.push_back( ellipsoid_point );          
    }
  }
  augmented_cloud->clear();
  *augmented_cloud = *thumb_workspace_cloud_xyzrgb;
  *augmented_cloud += ellipsoid_point_cloud;
  // visualize the ellipsoid inside the original workspace point cloud
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
  viewer->spinOnce();
  
  
  // save the best workspace ellipsoid subset point cloud
  pcl::PointCloud<pcl::PointXYZ> largest_ellipsoid_point_cloud;
  largest_ellipsoid_point_cloud.height   = 1;
  largest_ellipsoid_point_cloud.width    = ellipsoid_point_cloud.points.size();
  largest_ellipsoid_point_cloud.is_dense = false;
	largest_ellipsoid_point_cloud.points.resize(largest_ellipsoid_point_cloud.width * largest_ellipsoid_point_cloud.height);
  for(unsigned int i=0; i<ellipsoid_point_cloud.size(); i++){
	  largest_ellipsoid_point_cloud.points[i].x = ellipsoid_point_cloud.points[i].x;
	  largest_ellipsoid_point_cloud.points[i].y = ellipsoid_point_cloud.points[i].y;
	  largest_ellipsoid_point_cloud.points[i].z = ellipsoid_point_cloud.points[i].z;
	}
	pcl::io::savePCDFileASCII("thumb_largest_ellipsoid_workspace_subset.pcd" , largest_ellipsoid_point_cloud);
  
  
  while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce(); 
  } 
  return 0;
}

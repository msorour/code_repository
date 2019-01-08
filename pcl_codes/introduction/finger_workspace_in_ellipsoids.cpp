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
  std::string finger = "pinky";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_cloud_light_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_workspace_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/"+finger+"_workspace.pcd", *finger_workspace_cloud_xyz);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/"+finger+"_workspace_light.pcd", *finger_workspace_cloud_light_xyz);
  // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
  copyPointCloud(*finger_workspace_cloud_xyz, *finger_workspace_cloud_xyzrgb);
  *augmented_cloud = *finger_workspace_cloud_xyzrgb;
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
  concave_hull.setInputCloud( finger_workspace_cloud_xyz );
  concave_hull.setAlpha( 0.05f );
  concave_hull.reconstruct( mesh_out );
  Hullviewer->setBackgroundColor(0,0,0);
  Hullviewer->addPolygonMesh(mesh_out,"hull");
  
  // save the polygon mesh
  pcl::io::saveOBJFile(finger+"_workspace_concave_hull.obj", mesh_out );
  
  // getting the vertices of the polygon mesh generated
  pcl::PointCloud<pcl::PointXYZ> poly_mesh_vertices;
  pcl::fromPCLPointCloud2(mesh_out.cloud, poly_mesh_vertices);
  
  
  // searching for the biggest ellipsoid that is enclosed inside the concave hull given its center point (center point is just iteration of workspace point cloud)
  double a=0.0, b=0.0, c=0.0;   // ellipsoid half-length of principal axes
  double a_best=0.0, b_best=0.0, c_best=0.0;
  double ellipsoid_value=1.0;
  double accuracy=0.001;
  double ellipsoid_x, ellipsoid_y, ellipsoid_z;
  int ellipsoid_point_cloud_samples = 50;
  unsigned int delay_microseconds = 1;
  pcl::PointCloud<pcl::PointXYZRGB> ellipsoid_point_cloud;
  pcl::PointXYZRGB ellipsoid_point;
  pcl::PointXYZ ellipsoid_offset;
  pcl::PointCloud<pcl::PointXYZRGB> workspace_as_ellipsoids_point_cloud;
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud = *finger_workspace_cloud_light_xyz;
  
  // to save the ellipsoids data
  ofstream workspace_ellipsoids;
  std::string file_name = finger+"_workspace_as_ellipsoids.txt";
  workspace_ellipsoids.open( file_name.c_str() );
  workspace_ellipsoids << "a, "<<"b, "<<"c, "<<"offset_x, "<<"offset_y, "<<"offset_z"<<"\n";
  
  
  // iterate through all points of the point cloud !
  for(unsigned int it=0; it<cloud.points.size(); it++){
    std::cout<< "cloud point: " << cloud.points[it] << std::endl;
    ellipsoid_offset.x = cloud.points[it].x;
    ellipsoid_offset.y = cloud.points[it].y;
    ellipsoid_offset.z = cloud.points[it].z;
    a=0.025;
    b=0.025;
    c=0.025;
    
    
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
        *augmented_cloud += ellipsoid_point_cloud;
        workspace_as_ellipsoids_point_cloud += ellipsoid_point_cloud;
        // visualize the ellipsoid inside the original workspace point cloud
        viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
        viewer->spinOnce();
      }
      usleep(delay_microseconds);
    }
    
    // save largest ellipsoid at this sampled workspace point to file
    workspace_ellipsoids << a_best<<", "<< b_best<<", "<< c_best<<", "<< ellipsoid_offset.x<<", "<< ellipsoid_offset.y<<", "<< ellipsoid_offset.z<<"\n";
    std::cout<< "iteration: " <<it<< ", a_best=" <<a_best<< ", b_best=" <<b_best<< ", c_best=" <<c_best<<std::endl;
  
  }
  std::cout<< "Finished iterations!" <<std::endl;
  workspace_ellipsoids.close();
  
  // save workspace ellipsoids point cloud
  workspace_as_ellipsoids_point_cloud.height   = 1;
  workspace_as_ellipsoids_point_cloud.width    = workspace_as_ellipsoids_point_cloud.points.size();
  workspace_as_ellipsoids_point_cloud.is_dense = false;
	workspace_as_ellipsoids_point_cloud.points.resize(workspace_as_ellipsoids_point_cloud.width * workspace_as_ellipsoids_point_cloud.height);
  pcl::io::savePCDFileASCII(finger+"_workspace_as_ellipsoids.pcd", workspace_as_ellipsoids_point_cloud);
	
  while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce(); 
  } 
  return 0;
}

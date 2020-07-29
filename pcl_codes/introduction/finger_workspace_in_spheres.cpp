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
  std::string finger = "index";
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
  concave_hull.setAlpha( 0.01f );
  concave_hull.reconstruct( mesh_out );
  Hullviewer->setBackgroundColor(0,0,0);
  Hullviewer->addPolygonMesh(mesh_out,"hull");
  
  // save the polygon mesh
  pcl::io::saveOBJFile(finger+"_workspace_concave_hull.obj", mesh_out );
  
  // getting the vertices of the polygon mesh generated
  pcl::PointCloud<pcl::PointXYZ> poly_mesh_vertices;
  pcl::fromPCLPointCloud2(mesh_out.cloud, poly_mesh_vertices);
  
  
  // searching for the biggest sphere that is enclosed inside the concave hull given its center point (center point is just iteration of workspace point cloud)
  double r=0.0;
  double r_best=0.0;
  double r_max=0.0;
  double sphere_value=1.0;
  double accuracy=0.001;
  double sphere_x, sphere_y, sphere_z;
  int sphere_point_cloud_samples = 25;
  unsigned int delay_microseconds = 1000;
  pcl::PointCloud<pcl::PointXYZRGB> sphere_point_cloud;
  pcl::PointXYZRGB sphere_point;
  pcl::PointXYZ sphere_offset;
  pcl::PointCloud<pcl::PointXYZRGB> workspace_as_spheres_point_cloud;
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud = *finger_workspace_cloud_light_xyz;
  
  // to save the spheres data
  ofstream workspace_spheres;
  std::string file_name = finger+"_workspace_as_spheres.txt";
  workspace_spheres.open( file_name.c_str() );
  workspace_spheres << "r, "<<"offset_x, "<<"offset_y, "<<"offset_z"<<"\n";
  
  
  // iterate through all points of the point cloud !
  //for(unsigned int it=0; it<cloud.points.size(); it++){
  for(unsigned int it=0; it<finger_workspace_cloud_light_xyz->size(); it++){
    /*std::cout<< "cloud point: " << cloud.points[it] << std::endl;
    sphere_offset.x = cloud.points[it].x;
    sphere_offset.y = cloud.points[it].y;
    sphere_offset.z = cloud.points[it].z;*/
    std::cout<< "cloud point: " << finger_workspace_cloud_light_xyz->points[it] << std::endl;
    sphere_offset.x = finger_workspace_cloud_light_xyz->points[it].x;
    sphere_offset.y = finger_workspace_cloud_light_xyz->points[it].y;
    sphere_offset.z = finger_workspace_cloud_light_xyz->points[it].z;
    r=0.001;
    r_max=0.005;
    
    bool stop = false;
    // for this "seed" sphere center point
    //
    // find largest possible "a" dimension
    while(!stop){
      r += accuracy;
      for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
        // equation of sphere
        sphere_value =   pow(poly_mesh_vertices.points[j].x - sphere_offset.x, 2)/pow(r, 2) 
                       + pow(poly_mesh_vertices.points[j].y - sphere_offset.y, 2)/pow(r, 2) 
                       + pow(poly_mesh_vertices.points[j].z - sphere_offset.z, 2)/pow(r, 2);
        if( sphere_value < 1.0 )
          break;
      }
      if( sphere_value < 1.0 or r>r_max ){
        r_best = r - accuracy;
        break;
        stop = true;
      }
      else{
        // generate and view the point cloud of the sphere
        sphere_point_cloud.clear();
        for(int k=0; k<sphere_point_cloud_samples; k++){
          sphere_x = (-r+sphere_offset.x) + k*2*r/sphere_point_cloud_samples;
          for(int l=0; l<sphere_point_cloud_samples; l++){
            sphere_y = (-r+sphere_offset.y) + l*2*r/sphere_point_cloud_samples;
            sphere_z = sphere_offset.z + r*sqrt( 1 - pow(sphere_x-sphere_offset.x, 2)/pow(r, 2) - pow(sphere_y-sphere_offset.y, 2)/pow(r, 2) );
            sphere_point.x = sphere_x;
            sphere_point.y = sphere_y;
            sphere_point.z = sphere_z;
            sphere_point.r = 250;
            sphere_point.g = 0;
            sphere_point.b = 0;
            sphere_point_cloud.points.push_back( sphere_point );
            
            sphere_z = sphere_offset.z - r*sqrt( 1 - pow(sphere_x-sphere_offset.x, 2)/pow(r, 2) - pow(sphere_y-sphere_offset.y, 2)/pow(r, 2) );
            sphere_point.x = sphere_x;
            sphere_point.y = sphere_y;
            sphere_point.z = sphere_z;
            sphere_point.r = 250;
            sphere_point.g = 0;
            sphere_point.b = 0;
            sphere_point_cloud.points.push_back( sphere_point );          
          }
        }
        *augmented_cloud += sphere_point_cloud;
        workspace_as_spheres_point_cloud += sphere_point_cloud;
        // visualize the sphere inside the original workspace point cloud
        viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
        viewer->spinOnce();
      }
      usleep(delay_microseconds);
    }
    
    
    
    // save largest sphere at this sampled workspace point to file
    workspace_spheres << r_best<<", "<< sphere_offset.x<<", "<< sphere_offset.y<<", "<< sphere_offset.z<<"\n";
    std::cout<< "iteration: " <<it<< ", r_best=" <<r_best<<std::endl;
    
    
    
  
  }
  std::cout<< "Finished iterations!" <<std::endl;
  workspace_spheres.close();
  
  // save workspace spheres point cloud
  workspace_as_spheres_point_cloud.height   = 1;
  workspace_as_spheres_point_cloud.width    = workspace_as_spheres_point_cloud.points.size();
  workspace_as_spheres_point_cloud.is_dense = false;
	workspace_as_spheres_point_cloud.points.resize(workspace_as_spheres_point_cloud.width * workspace_as_spheres_point_cloud.height);
  pcl::io::savePCDFileASCII(finger+"_workspace_as_spheres.pcd", workspace_as_spheres_point_cloud);
	
  while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce(); 
  } 
  return 0;
}

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
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_as_ellipsoids_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_workspace_as_ellipsoids_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  
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
    
    // loading finger workspace as ellipsoids point cloud
    pcl::io::loadPCDFile<pcl::PointXYZ>(finger+"_workspace_as_ellipsoids.pcd", *finger_workspace_as_ellipsoids_cloud_xyz);
    
    // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
    copyPointCloud(*finger_workspace_cloud_xyz, *finger_workspace_cloud_xyzrgb);
    copyPointCloud(*finger_workspace_as_ellipsoids_cloud_xyz, *finger_workspace_as_ellipsoids_cloud_xyzrgb);
    
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
    
    for(unsigned int i=0; i<finger_workspace_as_ellipsoids_cloud_xyzrgb->size(); i++){
      finger_workspace_as_ellipsoids_cloud_xyzrgb->points[i].r = r;
      finger_workspace_as_ellipsoids_cloud_xyzrgb->points[i].g = g;
      finger_workspace_as_ellipsoids_cloud_xyzrgb->points[i].b = b;
    }
    
    *augmented_cloud += *finger_workspace_cloud_xyzrgb;
    *augmented_cloud += *finger_workspace_as_ellipsoids_cloud_xyzrgb;
    
    viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"cloud");
    viewer->spinOnce();
  }
  
  /*
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
  */
  
  /*
  // save workspace ellipsoids point cloud
  workspace_as_ellipsoids_point_cloud.height   = 1;
  workspace_as_ellipsoids_point_cloud.width    = workspace_as_ellipsoids_point_cloud.points.size();
  workspace_as_ellipsoids_point_cloud.is_dense = false;
	workspace_as_ellipsoids_point_cloud.points.resize(workspace_as_ellipsoids_point_cloud.width * workspace_as_ellipsoids_point_cloud.height);
  pcl::io::savePCDFileASCII(finger+"_workspace_as_ellipsoids.pcd", workspace_as_ellipsoids_point_cloud);
	*/
	
	
	
	// save hand workspace ellipsoids point cloud
	pcl::io::savePCDFileASCII("allegro_hand_workspace_as_ellipsoids.pcd", *augmented_cloud);
	
	
  //while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    //Hullviewer->spinOnce(); 
  } 
  return 0;
}

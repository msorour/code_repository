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

int main(int argc, char **argv){
  int workspace_low_sampled  = std::stoi( argv[1] );
  int workspace_high_sampled = std::stoi( argv[2] );
  
  std::vector<std::string> finger_list;
  finger_list.push_back("thumb");
  finger_list.push_back("index");
  finger_list.push_back("middle");
  finger_list.push_back("pinky");
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr finger_workspace_cloud_light_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_workspace_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("allegro hand workspace spheres"));
  
  // loading allegro hand model point cloud
  pcl::PointCloud<pcl::PointXYZ> allegro_hand_cloud_xyz;
  pcl::PointCloud<pcl::PointXYZRGB> allegro_hand_cloud_xyzrgb;
  pcl::io::loadPCDFile<pcl::PointXYZ>("allegro_right_hand_model_cloud.pcd", allegro_hand_cloud_xyz);
  
  // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
  copyPointCloud(allegro_hand_cloud_xyz, allegro_hand_cloud_xyzrgb);
  
  *augmented_cloud = allegro_hand_cloud_xyzrgb;
  
  viewer->addPointCloud(augmented_cloud,"allegro hand workspace spheres");
  viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "allegro hand workspace spheres");
  
  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  
  
  // generating the concave hull
  pcl::PolygonMesh mesh_out;
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  
  
  //
  double r=0.0;
  double r_best=0.0;
  double r_max=0.0;
  double sphere_value=1.0;
  double accuracy=0.001;
  double sphere_x, sphere_y, sphere_z;
  int sphere_point_cloud_samples = 20;
  unsigned int delay_microseconds = 1;
  pcl::PointCloud<pcl::PointXYZRGB> sphere_point_cloud;
  pcl::PointXYZRGB sphere_point;
  pcl::PointXYZ sphere_offset;
  //pcl::PointCloud<pcl::PointXYZRGB> workspace_as_spheres_point_cloud;
  //pcl::PointCloud<pcl::PointXYZRGB> workspace_as_spheres_point_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr workspace_as_spheres_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(workspace_as_spheres_point_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(workspace_as_spheres_point_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(workspace_as_spheres_point_cloud, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color(workspace_as_spheres_point_cloud, 100, 100, 100);
  
  // iterating through all fingers
  for(int finger_index=0; finger_index<finger_list.size(); finger_index++){
    
    workspace_as_spheres_point_cloud->clear();
    
    pcl::io::loadPCDFile<pcl::PointXYZ>   (finger_list[finger_index]+"_workspace_"+std::to_string(workspace_high_sampled)+"_normal.pcd", *finger_workspace_cloud_xyz);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(finger_list[finger_index]+"_workspace_"+std::to_string(workspace_high_sampled)+"_normal.pcd", *finger_workspace_cloud_xyzrgb);
    pcl::io::loadPCDFile<pcl::PointXYZ>   (finger_list[finger_index]+"_workspace_"+std::to_string(workspace_low_sampled) +"_convex_shape_generation.pcd", *finger_workspace_cloud_light_xyz);
    
    *augmented_cloud += *finger_workspace_cloud_xyzrgb;
    
    // generating concave hull out of the higher resolution workspace point cloud
    concave_hull.setInputCloud( finger_workspace_cloud_xyz );
    concave_hull.setAlpha( 0.01f );
    concave_hull.reconstruct( mesh_out );
    Hullviewer->setBackgroundColor(0,0,0);
    Hullviewer->addPolygonMesh(mesh_out,"hull");
    
    // save the polygon mesh
    pcl::io::saveOBJFile(finger_list[finger_index]+"_workspace_concave_hull.obj", mesh_out );
    
    // getting the vertices of the polygon mesh generated
    pcl::PointCloud<pcl::PointXYZ> poly_mesh_vertices;
    pcl::fromPCLPointCloud2(mesh_out.cloud, poly_mesh_vertices);
    
    
    // searching for the biggest sphere that is enclosed inside the concave hull given its center point (center point is just iteration of workspace point cloud)
    
    // to save the spheres data
    ofstream workspace_spheres;
    std::string file_name = finger_list[finger_index]+"_workspace_spheres_"+std::to_string(workspace_low_sampled)+".txt";
    workspace_spheres.open( file_name.c_str() );
    workspace_spheres << "r, "<<"offset_x, "<<"offset_y, "<<"offset_z"<<"\n";
    
    // iterate through all points of the point cloud !
    for(unsigned int it=0; it<finger_workspace_cloud_light_xyz->size(); it++){
      sphere_offset.x = finger_workspace_cloud_light_xyz->points[it].x;
      sphere_offset.y = finger_workspace_cloud_light_xyz->points[it].y;
      sphere_offset.z = finger_workspace_cloud_light_xyz->points[it].z;
      r=0.001;
      r_max=0.05;
      
      
      
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
          //break;
          stop = true;
        }
        
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
            if(finger_list[finger_index]=="thumb"){
                sphere_point.r = 255;  sphere_point.g = 0;  sphere_point.b = 0;
            }
            else if(finger_list[finger_index]=="index"){
              sphere_point.r = 0;  sphere_point.g = 255;  sphere_point.b = 0;
            }
            else if(finger_list[finger_index]=="middle"){
              sphere_point.r = 0;  sphere_point.g = 0;  sphere_point.b = 255;
            }
            else if(finger_list[finger_index]=="pinky"){
              sphere_point.r = 100;  sphere_point.g = 100;  sphere_point.b = 100;
            }
            sphere_point_cloud.points.push_back( sphere_point );
            
            sphere_z = sphere_offset.z - r*sqrt( 1 - pow(sphere_x-sphere_offset.x, 2)/pow(r, 2) - pow(sphere_y-sphere_offset.y, 2)/pow(r, 2) );
            sphere_point.x = sphere_x;
            sphere_point.y = sphere_y;
            sphere_point.z = sphere_z;
            if(finger_list[finger_index]=="thumb"){
                sphere_point.r = 255;  sphere_point.g = 0;  sphere_point.b = 0;
            }
            else if(finger_list[finger_index]=="index"){
              sphere_point.r = 0;  sphere_point.g = 255;  sphere_point.b = 0;
            }
            else if(finger_list[finger_index]=="middle"){
              sphere_point.r = 0;  sphere_point.g = 0;  sphere_point.b = 255;
            }
            else if(finger_list[finger_index]=="pinky"){
              sphere_point.r = 100;  sphere_point.g = 100;  sphere_point.b = 100;
            }
            sphere_point_cloud.points.push_back( sphere_point );          
          }
        }
        //*augmented_cloud += sphere_point_cloud;
        *workspace_as_spheres_point_cloud += sphere_point_cloud;
        // visualize the sphere inside the original workspace point cloud
        viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"allegro hand workspace spheres");
        
        if(finger_list[finger_index]=="thumb"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, red_color, "red cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, red_color, "red cloud");
        }
        else if(finger_list[finger_index]=="index"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, green_color, "green cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, green_color, "green cloud");
        }
        else if(finger_list[finger_index]=="middle"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, blue_color, "blue cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, blue_color, "blue cloud");
        }
        else if(finger_list[finger_index]=="pinky"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, grey_color, "grey cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_spheres_point_cloud, grey_color, "grey cloud");
        }
        
        viewer->spinOnce();
        usleep(delay_microseconds);
      }
      
      
      // save largest sphere at this sampled workspace point to file
    workspace_spheres << r_best<<", "<< sphere_offset.x<<", "<< sphere_offset.y<<", "<< sphere_offset.z<<"\n";
    std::cout<< "iteration: " <<it<< ", r_best=" <<r_best<<std::endl;
      
    
    }   // end of iteration through all point cloud loop
    
    
    
    std::cout<< "Finished iterations!" <<std::endl;
    workspace_spheres.close();
    
    // save workspace spheres point cloud
    workspace_as_spheres_point_cloud->height   = 1;
    workspace_as_spheres_point_cloud->width    = workspace_as_spheres_point_cloud->points.size();
    workspace_as_spheres_point_cloud->is_dense = false;
	  workspace_as_spheres_point_cloud->points.resize(workspace_as_spheres_point_cloud->width * workspace_as_spheres_point_cloud->height);
    pcl::io::savePCDFileASCII(finger_list[finger_index]+"_workspace_spheres_"+std::to_string(workspace_low_sampled)+".pcd", *workspace_as_spheres_point_cloud);
	
	
	
	}   // end of fingers loop
	
	
	viewer->saveScreenshot("allegro_hand_workspace_spheres_"+std::to_string(workspace_low_sampled)+".png");
	
  while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce(); 
  } 
  return 0;
}

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
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("allegro hand workspace ellipsoids"));
  
  // loading allegro hand model point cloud
  pcl::PointCloud<pcl::PointXYZ> allegro_hand_cloud_xyz;
  pcl::PointCloud<pcl::PointXYZRGB> allegro_hand_cloud_xyzrgb;
  pcl::io::loadPCDFile<pcl::PointXYZ>("allegro_right_hand_model_cloud.pcd", allegro_hand_cloud_xyz);
  
  // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
  copyPointCloud(allegro_hand_cloud_xyz, allegro_hand_cloud_xyzrgb);
  
  *augmented_cloud = allegro_hand_cloud_xyzrgb;
  
  viewer->addPointCloud(augmented_cloud,"allegro hand workspace ellipsoids");
  viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "allegro hand workspace ellipsoids");
  
  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  
  
  // generating the concave hull
  pcl::PolygonMesh mesh_out;
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  
  
  //
  double a=0.0, b=0.0, c=0.0;   // ellipsoid half-length of principal axes
  double a_best=0.0, b_best=0.0, c_best=0.0;
  double ellipsoid_value=1.0;
  double accuracy=0.001;
  double ellipsoid_x, ellipsoid_y, ellipsoid_z;
  int ellipsoid_point_cloud_samples = 25;
  unsigned int delay_microseconds = 1;
  pcl::PointCloud<pcl::PointXYZRGB> ellipsoid_point_cloud;
  pcl::PointXYZRGB ellipsoid_point;
  pcl::PointXYZ ellipsoid_offset;
  //pcl::PointCloud<pcl::PointXYZRGB> workspace_as_ellipsoids_point_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr workspace_as_ellipsoids_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(workspace_as_ellipsoids_point_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(workspace_as_ellipsoids_point_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(workspace_as_ellipsoids_point_cloud, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color(workspace_as_ellipsoids_point_cloud, 100, 100, 100);
  
    
  // iterating through all fingers
  for(int finger_index=0; finger_index<finger_list.size(); finger_index++){
    
    workspace_as_ellipsoids_point_cloud->clear();
    
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
    
    
    // searching for the biggest ellipsoid that is enclosed inside the concave hull given its center point (center point is just iteration of workspace point cloud)
    
    // to save the ellipsoids data
    ofstream workspace_ellipsoids;
    std::string file_name = finger_list[finger_index]+"_workspace_ellipsoids_"+std::to_string(workspace_low_sampled)+".txt";
    workspace_ellipsoids.open( file_name.c_str() );
    workspace_ellipsoids << "a, "<<"b, "<<"c, "<<"offset_x, "<<"offset_y, "<<"offset_z"<<"\n";
    
    
    // iterate through all points of the point cloud !
    for(unsigned int it=0; it<finger_workspace_cloud_light_xyz->size(); it++){
      ellipsoid_offset.x = finger_workspace_cloud_light_xyz->points[it].x;
      ellipsoid_offset.y = finger_workspace_cloud_light_xyz->points[it].y;
      ellipsoid_offset.z = finger_workspace_cloud_light_xyz->points[it].z;
      a=0.004;
      b=0.004;
      c=0.004;
      
      
      
      
      
      
      // find nearest vertex point to the the current seed point
      double euclidean_distance, smallest_distance=1000000;
      pcl::PointXYZ nearest_point;
      for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
        euclidean_distance = sqrt(pow(poly_mesh_vertices.points[j].x - ellipsoid_offset.x, 2) 
                                + pow(poly_mesh_vertices.points[j].y - ellipsoid_offset.y, 2) 
                                + pow(poly_mesh_vertices.points[j].z - ellipsoid_offset.z, 2));
        if( euclidean_distance < smallest_distance ){
          smallest_distance = euclidean_distance;
          nearest_point = poly_mesh_vertices.points[j];
        }
      }
      
      
      bool stop = false;
      bool a_stop = false;
      bool b_stop = false;
      //
      // find largest possible "c" dimension
      while(!stop){
        
        // a
        if(!a_stop){
          a += accuracy;
          for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
            // equation of ellipsoid
            ellipsoid_value =   pow(poly_mesh_vertices.points[j].x - ellipsoid_offset.x, 2)/pow(a, 2) 
                              + pow(poly_mesh_vertices.points[j].y - ellipsoid_offset.y, 2)/pow(b, 2) 
                              + pow(poly_mesh_vertices.points[j].z - ellipsoid_offset.z, 2)/pow(c, 2);
            if( ellipsoid_value < 1.0 )
              break;
          }
          if( ellipsoid_value < 1.0 or a >= smallest_distance ){
            a_best = a - accuracy;
            a_stop = true;
          }
          std::cout<< "1... a = "<< a <<std::endl;
        }
        
        
        // b
        if(!b_stop){
          b += accuracy;
          for(unsigned int j=0;j<poly_mesh_vertices.size();j++){
            // equation of ellipsoid
            ellipsoid_value =   pow(poly_mesh_vertices.points[j].x - ellipsoid_offset.x, 2)/pow(a_best, 2) 
                              + pow(poly_mesh_vertices.points[j].y - ellipsoid_offset.y, 2)/pow(b, 2) 
                              + pow(poly_mesh_vertices.points[j].z - ellipsoid_offset.z, 2)/pow(c, 2);
            if( ellipsoid_value < 1.0 )
              break;
          }
          if( ellipsoid_value < 1.0 or b >= smallest_distance){
            b_best = b - accuracy;
            b_stop = true;
          }
          std::cout<< "2... b = "<< b <<std::endl;
        }
        
        
        // c
        if(c < smallest_distance){
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
            stop = true;
          }
          std::cout<< "3... "<<std::endl;
        }
        else
          stop = true;
        
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
            if(finger_list[finger_index]=="thumb"){
              ellipsoid_point.r = 255;  ellipsoid_point.g = 0;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="index"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 255;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="middle"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 0;  ellipsoid_point.b = 255;
            }
            else if(finger_list[finger_index]=="pinky"){
              ellipsoid_point.r = 100;  ellipsoid_point.g = 100;  ellipsoid_point.b = 100;
            }
            ellipsoid_point_cloud.points.push_back( ellipsoid_point );
            
            ellipsoid_z = ellipsoid_offset.z - c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b_best, 2) );
            ellipsoid_point.x = ellipsoid_x;
            ellipsoid_point.y = ellipsoid_y;
            ellipsoid_point.z = ellipsoid_z;
            if(finger_list[finger_index]=="thumb"){
              ellipsoid_point.r = 255;  ellipsoid_point.g = 0;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="index"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 255;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="middle"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 0;  ellipsoid_point.b = 255;
            }
            else if(finger_list[finger_index]=="pinky"){
              ellipsoid_point.r = 100;  ellipsoid_point.g = 100;  ellipsoid_point.b = 100;
            }
            ellipsoid_point_cloud.points.push_back( ellipsoid_point );          
          }
        }
        //*augmented_cloud += ellipsoid_point_cloud;
        *workspace_as_ellipsoids_point_cloud += ellipsoid_point_cloud;
        // visualize the ellipsoid inside the original workspace point cloud
        viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"allegro hand workspace ellipsoids");
        
        if(finger_list[finger_index]=="thumb"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, red_color, "red cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, red_color, "red cloud");
        }
        else if(finger_list[finger_index]=="index"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, green_color, "green cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, green_color, "green cloud");
        }
        else if(finger_list[finger_index]=="middle"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, blue_color, "blue cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, blue_color, "blue cloud");
        }
        else if(finger_list[finger_index]=="pinky"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, grey_color, "grey cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, grey_color, "grey cloud");
        }
        
                
        viewer->spinOnce();
        usleep(delay_microseconds);
      }
      
      
      
      
      
      
      
      
      
      
      
      /*
      bool stop = false;
      // for this "seed" ellipsoid center point
      //
      // find largest possible "a" dimension
      while(!stop){
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
          //break;
          stop = true;
        }  
        std::cout<< "1... a = "<< a <<std::endl;   
      }
      
      
      
      stop = false;
      //
      // find largest possible "b" dimension
      while(!stop){
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
          //break;
          stop = true;
        }  
        std::cout<< "2... b = "<< b <<std::endl;
      }
      
      
      
      stop = false;
      //
      // find largest possible "c" dimension
      while(!stop){
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
          //break;
          stop = true;
        }
        std::cout<< "3... "<<std::endl;
        
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
            if(finger_list[finger_index]=="thumb"){
              ellipsoid_point.r = 255;  ellipsoid_point.g = 0;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="index"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 255;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="middle"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 0;  ellipsoid_point.b = 255;
            }
            else if(finger_list[finger_index]=="pinky"){
              ellipsoid_point.r = 100;  ellipsoid_point.g = 100;  ellipsoid_point.b = 100;
            }
            ellipsoid_point_cloud.points.push_back( ellipsoid_point );
            
            ellipsoid_z = ellipsoid_offset.z - c*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(a_best, 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(b_best, 2) );
            ellipsoid_point.x = ellipsoid_x;
            ellipsoid_point.y = ellipsoid_y;
            ellipsoid_point.z = ellipsoid_z;
            if(finger_list[finger_index]=="thumb"){
              ellipsoid_point.r = 255;  ellipsoid_point.g = 0;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="index"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 255;  ellipsoid_point.b = 0;
            }
            else if(finger_list[finger_index]=="middle"){
              ellipsoid_point.r = 0;  ellipsoid_point.g = 0;  ellipsoid_point.b = 255;
            }
            else if(finger_list[finger_index]=="pinky"){
              ellipsoid_point.r = 100;  ellipsoid_point.g = 100;  ellipsoid_point.b = 100;
            }
            ellipsoid_point_cloud.points.push_back( ellipsoid_point );          
          }
        }
        //*augmented_cloud += ellipsoid_point_cloud;
        *workspace_as_ellipsoids_point_cloud += ellipsoid_point_cloud;
        // visualize the ellipsoid inside the original workspace point cloud
        viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"allegro hand workspace ellipsoids");
        
        if(finger_list[finger_index]=="thumb"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, red_color, "red cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, red_color, "red cloud");
        }
        else if(finger_list[finger_index]=="index"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, green_color, "green cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, green_color, "green cloud");
        }
        else if(finger_list[finger_index]=="middle"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, blue_color, "blue cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, blue_color, "blue cloud");
        }
        else if(finger_list[finger_index]=="pinky"){
          if(!viewer->addPointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, grey_color, "grey cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(workspace_as_ellipsoids_point_cloud, grey_color, "grey cloud");
        }
        
                
        viewer->spinOnce();
        usleep(delay_microseconds);
      }
      */
      
      // save largest ellipsoid at this sampled workspace point to file
      workspace_ellipsoids << a_best<<", "<< b_best<<", "<< c_best<<", "<< ellipsoid_offset.x<<", "<< ellipsoid_offset.y<<", "<< ellipsoid_offset.z<<"\n";
      std::cout<< "iteration: " <<it<< ", a_best=" <<a_best<< ", b_best=" <<b_best<< ", c_best=" <<c_best<<std::endl;
      
      
      
    
    }   // end of iteration through all point cloud loop
    
    
    
    std::cout<< "Finished iterations!" <<std::endl;
    workspace_ellipsoids.close();
    
    // save workspace ellipsoids point cloud
    workspace_as_ellipsoids_point_cloud->height   = 1;
    workspace_as_ellipsoids_point_cloud->width    = workspace_as_ellipsoids_point_cloud->points.size();
    workspace_as_ellipsoids_point_cloud->is_dense = false;
	  workspace_as_ellipsoids_point_cloud->points.resize(workspace_as_ellipsoids_point_cloud->width * workspace_as_ellipsoids_point_cloud->height);
    pcl::io::savePCDFileASCII(finger_list[finger_index]+"_workspace_ellipsoids_"+std::to_string(workspace_low_sampled)+".pcd", *workspace_as_ellipsoids_point_cloud);
	
	
	
	}   // end of fingers loop
	
	viewer->saveScreenshot("allegro_hand_workspace_ellipsoids.png");
	
  while ( !viewer->wasStopped() and !Hullviewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce(); 
  } 
  return 0;
}

#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h>   // for sleep
#include "../../ros_packages/include/Eigen/Dense"
#include "../../ros_packages/include/useful_implementations.h"
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <time.h>
#include "QuadProg++.hh"
#include "grasping_algorithm.h"

int main(int argc, char **argv){
  std::string object_name, workspace_type, sampling;
  object_name = argv[1];
  workspace_type = argv[2];
  sampling = argv[3];
  
  clock_t begin, end;
	double time_spent;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr    object_cloud_xyz    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud     (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  begin = clock();
  
  // load object point cloud
  pcl::io::loadPCDFile<pcl::PointXYZ>(object_name+".pcd", *object_cloud_xyz);
  std::cout << "object point cloud size: " << object_cloud_xyz->size() << std::endl;
  
  copyPointCloud(*object_cloud_xyz, *object_cloud_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  *augmented_cloud = *object_cloud_xyzrgb;
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to load point clouds = " << time_spent << std::endl << std::endl;
  
  
  // for visualizing point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("cloud viewer"));
  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  viewer->addPointCloud(augmented_cloud,rgb,"object cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object cloud");
  
  viewer->setCameraPosition(0.859057, 0.544554, 0.609515, -0.0194028, 0.0614014, -0.0200482, -0.38203, -0.231229, 0.894755, 0);
  
  
  
  // for visualizing polygon mesh (.obj)
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  Hullviewer->setBackgroundColor(0,0,0);
  
  //
  viewer->addCoordinateSystem(0.2);
  
  
  
  
  
  
  begin = clock();
  // loading mesh to save development time!
  pcl::PolygonMesh object_mesh;
  pcl::io::loadOBJFile(object_name+".obj", object_mesh );
  
  // getting the vertices of the polygon mesh loaded
  pcl::PointCloud<pcl::PointXYZ> object_mesh_vertices;
  pcl::PointCloud<pcl::PointXYZRGB> object_mesh_vertices_xyzrgb;
  pcl::fromPCLPointCloud2(object_mesh.cloud, object_mesh_vertices);
  
  copyPointCloud(object_mesh_vertices, object_mesh_vertices_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  //*augmented_cloud += object_mesh_vertices_xyzrgb;
  
  // override in case of not using .obj files (.ply file for example)
  //pcl::io::loadPLYFile<pcl::PointXYZ>(object_name+".ply", *object_cloud_xyz);
  //object_mesh_vertices = *object_cloud_xyz;
  
  std::cout << "number of vertices: " << object_mesh_vertices.size() << std::endl << std::endl;
  Hullviewer->addPolygonMesh(object_mesh,"hull");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to load mesh file = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  // STEP#4 : compute object transformation matrix
  Eigen::Matrix3f object_rotation;
  Eigen::Vector3f object_translation;
  Eigen::Matrix4f object_transform;
  object_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector4f object_far_point_in_pos_direction_in_global_frame;
  Eigen::Vector4f object_far_point_in_neg_direction_in_global_frame;
  Eigen::Vector3f object_major_dimensions;
  step_4_object_pose_approximation( object_mesh_vertices, object_transform, object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions );
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.matrix() = object_transform;
  viewer->addCoordinateSystem(0.1, transform, "object coordinate frame", 0);
  viewer->addCoordinateSystem(0.1,object_far_point_in_pos_direction_in_global_frame(0),object_far_point_in_pos_direction_in_global_frame(1),object_far_point_in_pos_direction_in_global_frame(2));
  viewer->addCoordinateSystem(0.1,object_far_point_in_neg_direction_in_global_frame(0),object_far_point_in_neg_direction_in_global_frame(1),object_far_point_in_neg_direction_in_global_frame(2));
  
  object_rotation    << object_transform(0,0), object_transform(0,1), object_transform(0,2),
                        object_transform(1,0), object_transform(1,1), object_transform(1,2),
                        object_transform(2,0), object_transform(2,1), object_transform(2,2);
  object_translation << object_transform(0,3), object_transform(1,3), object_transform(2,3);
  
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  
  
  
  
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid;
  pcl::PointXYZ object_centroid_point;
  for(unsigned int i=0;i<object_mesh_vertices.points.size();i++)
    object_centroid.add( object_mesh_vertices.points[i] );
  object_centroid.get(object_centroid_point);
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  Eigen::Matrix4f dummy_transform;
  Eigen::Vector3f dummy_translation;
  Eigen::Matrix3f dummy_rotation;
  
  
  begin = clock();
  std::vector<std::string> finger_list;
  finger_list.push_back("thumb");
  finger_list.push_back("index");
  finger_list.push_back("middle");
  finger_list.push_back("pinky");
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr index_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_workspace_convex_xyzrgb       (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pinky_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_workspace_convex_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  // offset of convex shape (ellipsoid or sphere)
  pcl::PointCloud<pcl::PointXYZ> thumb_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> index_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> middle_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> pinky_workspace_convex_offset;
  pcl::PointXYZ offset;
  
  // parameter of convex shape = {a,b,c} for ellipsoid or {r} for sphere
  pcl::PointCloud<pcl::PointXYZ> thumb_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> index_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> middle_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> pinky_workspace_convex_parameter;
  pcl::PointXYZ parameter;
  
  
  for(int i=0; i<finger_list.size(); i++){
    std::vector<double> data;
    std::string line;
    ifstream convex_workspace_file(finger_list[i]+"_workspace_" +workspace_type+"_"+sampling+".txt");
    if(convex_workspace_file.is_open()){
      while(getline( convex_workspace_file, line ) ){
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
        
        if(workspace_type == "ellipsoids"){
          parameter.x = data[0];
          parameter.y = data[1];
          parameter.z = data[2];
          offset.x = data[3];
          offset.y = data[4];
          offset.z = data[5];
        }
        else if(workspace_type == "spheres"){
          parameter.x = data[0];
          parameter.y = data[0];
          parameter.z = data[0];
          offset.x = data[1];
          offset.y = data[2];
          offset.z = data[3];
        }
        
        if(finger_list[i]=="thumb"){
          thumb_workspace_convex_offset.push_back( offset );
          thumb_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="index"){
          index_workspace_convex_offset.push_back( offset );
          index_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="middle"){
          middle_workspace_convex_offset.push_back( offset );
          middle_workspace_convex_parameter.push_back( parameter );
        }
        else if(finger_list[i]=="pinky"){
          pinky_workspace_convex_offset.push_back( offset );
          pinky_workspace_convex_parameter.push_back( parameter );
        }
      }
    }
    convex_workspace_file.close();
  }
  
  
  
  // generate and view the point cloud of the convex workspace
  int point_cloud_samples = 30;
  double value_x, value_y, value_z;
  pcl::PointXYZRGB point_xyzrgb;
  pcl::PointXYZ convex_shape_offset, convex_shape_parameter;
  for(int i=0; i<finger_list.size(); i++){
    for(unsigned int j=0; j<thumb_workspace_convex_offset.size(); j++){
      if(finger_list[i]=="thumb"){
        convex_shape_offset.x = thumb_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = thumb_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = thumb_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = thumb_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = thumb_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = thumb_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="index"){
        convex_shape_offset.x = index_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = index_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = index_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = index_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = index_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = index_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="middle"){
        convex_shape_offset.x = middle_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = middle_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = middle_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = middle_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = middle_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = middle_workspace_convex_parameter.points[j].z;
      }
      else if(finger_list[i]=="pinky"){
        convex_shape_offset.x = pinky_workspace_convex_offset.points[j].x;
        convex_shape_offset.y = pinky_workspace_convex_offset.points[j].y;
        convex_shape_offset.z = pinky_workspace_convex_offset.points[j].z;
        convex_shape_parameter.x = pinky_workspace_convex_parameter.points[j].x;
        convex_shape_parameter.y = pinky_workspace_convex_parameter.points[j].y;
        convex_shape_parameter.z = pinky_workspace_convex_parameter.points[j].z;
      }
      for(unsigned int k=0; k<point_cloud_samples; k++){
        value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
        for(unsigned int l=0; l<point_cloud_samples; l++){
          value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
          value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          point_xyzrgb.z = value_z;
          
          if(finger_list[i]=="thumb"){
            thumb_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
          }
          else if(finger_list[i]=="index"){
            index_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 0;  point_xyzrgb.g = 255;  point_xyzrgb.b = 0;
          }
          else if(finger_list[i]=="middle"){
            middle_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;
          }
          else if(finger_list[i]=="pinky"){
            pinky_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 100;  point_xyzrgb.g = 100;  point_xyzrgb.b = 100;
          }
          
          value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          point_xyzrgb.z = value_z;
          
          if(finger_list[i]=="thumb"){
            thumb_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
          }
          else if(finger_list[i]=="index"){
            index_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 0;  point_xyzrgb.g = 255;  point_xyzrgb.b = 0;
          }
          else if(finger_list[i]=="middle"){
            middle_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 0;  point_xyzrgb.g = 0;  point_xyzrgb.b = 255;
          }
          else if(finger_list[i]=="pinky"){
            pinky_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
            point_xyzrgb.r = 100;  point_xyzrgb.g = 100;  point_xyzrgb.b = 100;
          } 
        }
      }
    }
  }
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color  (thumb_workspace_convex_xyzrgb, 255, 0, 0);
  //viewer->addPointCloud<pcl::PointXYZRGB>(thumb_workspace_convex_xyzrgb, red_color, "red cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(index_workspace_convex_xyzrgb, 0, 255, 0);
  //viewer->addPointCloud<pcl::PointXYZRGB>(index_workspace_convex_xyzrgb, green_color, "green cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color (middle_workspace_convex_xyzrgb, 0, 0, 255);
  //viewer->addPointCloud<pcl::PointXYZRGB>(middle_workspace_convex_xyzrgb, blue_color, "blue cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color (pinky_workspace_convex_xyzrgb, 100, 100, 100);
  //viewer->addPointCloud<pcl::PointXYZRGB>(pinky_workspace_convex_xyzrgb, grey_color, "grey cloud");
  
  /*
  *augmented_cloud += *thumb_workspace_convex_xyzrgb;
  *augmented_cloud += *index_workspace_convex_xyzrgb;
  *augmented_cloud += *middle_workspace_convex_xyzrgb;
  *augmented_cloud += *pinky_workspace_convex_xyzrgb;
  */
  
  // load allegro hand only
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_right_hand_model_cloud.pcd", *allegro_hand_xyzrgb);
  
  //*augmented_cloud += *allegro_hand_xyzrgb;
  
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to load and make convex workspace point cloud = " << time_spent << std::endl << std::endl;
  
  
  
  /*
  
  
  // load allegro hand workspace
  
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("thumb_workspace_" +workspace_type+"_"+sampling+".pcd", *thumb_workspace_convex_xyzrgb);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("index_workspace_" +workspace_type+"_"+sampling+".pcd", *index_workspace_convex_xyzrgb);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("middle_workspace_"+workspace_type+"_"+sampling+".pcd", *middle_workspace_convex_xyzrgb);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("pinky_workspace_" +workspace_type+"_"+sampling+".pcd", *pinky_workspace_convex_xyzrgb);
  
  *allegro_hand_workspace_convex_xyzrgb = *thumb_workspace_convex_xyzrgb;
  *allegro_hand_workspace_convex_xyzrgb += *index_workspace_convex_xyzrgb;
  *allegro_hand_workspace_convex_xyzrgb += *middle_workspace_convex_xyzrgb;
  *allegro_hand_workspace_convex_xyzrgb += *pinky_workspace_convex_xyzrgb;
  */
  
  
  
  
  begin = clock();
  
  // load allegro whole workspace point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_workspace_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_right_hand_workspace_"+sampling+".pcd", *allegro_hand_workspace_xyzrgb);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_right_hand_workspace_10_normal.pcd", *allegro_hand_workspace_xyzrgb);
  
  // get the maximum hand opening (approximate)
  // first get centroid
  // get the centroid first
  pcl::PointXYZRGB hand_workspace_centroid_point;
  pcl::CentroidPoint<pcl::PointXYZRGB> hand_workspace_centroid;
  for(unsigned int i=0;i<allegro_hand_workspace_xyzrgb->size();i++)
    hand_workspace_centroid.add( allegro_hand_workspace_xyzrgb->points[i] );
  hand_workspace_centroid.get( hand_workspace_centroid_point );
  //viewer->addCoordinateSystem(0.1,hand_workspace_centroid_point.x,hand_workspace_centroid_point.y,hand_workspace_centroid_point.z);
  
  // get max hand opening
  double hand_workspace_far_pos_point_in_z = hand_workspace_centroid_point.z;
  double hand_workspace_far_neg_point_in_z = hand_workspace_centroid_point.z;
  double max_hand_opening_approx;
  for(unsigned int i=0;i<allegro_hand_workspace_xyzrgb->size();i++){
    if( allegro_hand_workspace_xyzrgb->points[i].z > hand_workspace_far_pos_point_in_z )
      hand_workspace_far_pos_point_in_z = allegro_hand_workspace_xyzrgb->points[i].z;
    if( allegro_hand_workspace_xyzrgb->points[i].z < hand_workspace_far_neg_point_in_z )
      hand_workspace_far_neg_point_in_z = allegro_hand_workspace_xyzrgb->points[i].z;
  }
  max_hand_opening_approx = (hand_workspace_far_pos_point_in_z-hand_workspace_far_neg_point_in_z)*2/3;
  std::cout << "max_hand_opening_approx = " << max_hand_opening_approx << std::endl;
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to get max hand openning = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  // object cloud in its own frame
  Eigen::Matrix4f inverse_object_transform;
  inverse_object_transform << object_rotation.transpose(), -object_rotation.transpose()*object_translation,  // from khalil's book page 21
                              0, 0, 0, 1;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_in_object_frame_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(object_mesh_vertices_xyzrgb, *object_cloud_in_object_frame_xyzrgb, inverse_object_transform);
  
  //*augmented_cloud += *object_cloud_in_object_frame_xyzrgb;
  *augmented_cloud += object_mesh_vertices_xyzrgb;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  /////////
  // STEP#5 : region of interest
  
  double roi_h = 0.14/2;  // 28mm is finger tip diameter - along object z-axis
  double roi_w = max_hand_opening_approx/2;
  double roi_l = 0.3/2;
  pcl::PointCloud<pcl::PointXYZRGB> roi_point_cloud;
  
  // draw the special ellipsoid
  // centered at origin with orientation coincident to that of origin frame
  roi_point_cloud.clear();
  point_cloud_samples = 250;
  for(unsigned int k=0; k<point_cloud_samples; k++){
    value_x = (-roi_l) + k*2*roi_l/point_cloud_samples;
    for(unsigned int l=0; l<point_cloud_samples; l++){
      value_y = (-roi_w) + l*2*roi_w/point_cloud_samples;
      value_z = roi_h*sqrt( 1 - pow(value_x, 10)/pow(roi_l, 10) - pow(value_y, 10)/pow(roi_w, 10) );
      point_xyzrgb.x = value_x;
      point_xyzrgb.y = value_y;
      point_xyzrgb.z = value_z;
      point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
      roi_point_cloud.points.push_back( point_xyzrgb );
      
      value_z = - roi_h*sqrt( 1 - pow(value_x, 10)/pow(roi_l, 10) - pow(value_y, 10)/pow(roi_w, 10) );
      point_xyzrgb.x = value_x;
      point_xyzrgb.y = value_y;
      point_xyzrgb.z = value_z;
      point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
      roi_point_cloud.points.push_back( point_xyzrgb );    
    }
  }
  
  Eigen::Matrix3f roi_rotation;
  Eigen::Vector3f roi_translation;
  Eigen::Matrix4f roi_transform;
  roi_transform = object_transform;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_in_global_frame (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_in_object_frame (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(roi_point_cloud, *roi_in_global_frame, roi_transform);
  
  
  unsigned int counter = 0;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> roi_color  (roi_in_global_frame, 255, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB>(roi_in_global_frame, roi_color, "roi cloud");
  
  
  
  /*
  Eigen::Vector3f roi_dimensions;
  Eigen::Matrix4f roi_transform_in_object_frame_again;
  roi_dimensions << roi_l, roi_w, roi_h;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_object_global_xyzrgb_again (new pcl::PointCloud<pcl::PointXYZRGB>());
  
  step_5_region_of_interest( object_mesh_vertices, object_transform , object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions, roi_dimensions, roi_transform_in_object_frame_again, object_roi_sub_cloud_in_object_global_xyzrgb_again );
  
  
  
  pcl::transformPointCloud(roi_point_cloud, *roi_in_object_frame, roi_transform_in_object_frame_again);
  pcl::transformPointCloud(*roi_in_object_frame, *roi_in_global_frame, roi_transform);
  viewer->updatePointCloud<pcl::PointXYZRGB>(roi_in_global_frame, roi_color, "roi cloud");
  viewer->spinOnce();
  
  
  */
  
  
  
  Eigen::Vector3f roi_dimensions;
  Eigen::Matrix4f roi_transform_in_object_frame_again;
  Eigen::Matrix4f hand_transform;
  Eigen::Matrix3f hand_rotation;
  Eigen::Vector3f hand_translation;
  roi_dimensions << roi_l, roi_w, roi_h;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_object_global_xyzrgb_again (new pcl::PointCloud<pcl::PointXYZRGB>());
  Eigen::Vector3f hand_dimensions;
  hand_dimensions << 0.2477, 0.13948, 0.0548;
  
  step_5_region_of_interest( object_mesh_vertices, object_transform , object_far_point_in_pos_direction_in_global_frame, object_far_point_in_neg_direction_in_global_frame, object_major_dimensions, roi_dimensions, roi_transform_in_object_frame_again, object_roi_sub_cloud_in_object_global_xyzrgb_again, hand_transform, hand_dimensions );
  
  
  
  pcl::transformPointCloud(roi_point_cloud, *roi_in_object_frame, roi_transform_in_object_frame_again);
  pcl::transformPointCloud(*roi_in_object_frame, *roi_in_global_frame, roi_transform);
  viewer->updatePointCloud<pcl::PointXYZRGB>(roi_in_global_frame, roi_color, "roi cloud");
  viewer->spinOnce();
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  begin = clock();
  // STEP#5
  // get the nearest object point to the palm center point
  Eigen::Vector4f palm_center_point_in_hand_frame, palm_center_point_in_global_frame;
  palm_center_point_in_hand_frame << 0.0, 0.0, 0.0475, 1.0;
  palm_center_point_in_global_frame = hand_transform*palm_center_point_in_hand_frame;
  
  // find the nearest 3 points on object to these 3 palm points
  double distance_to_palm_center_point = 1000000;
  double euclidean_distance;
  pcl::PointXYZRGB nearest_object_point_to_palm_center_point;
  pcl::PointXYZRGB nearest_object_point_to_palm_right_point;
  pcl::PointXYZRGB nearest_object_point_to_palm_left_point;
  
  // I need to use the object point cloud expressed in hand frame!!!
  Eigen::Matrix4f inverse_hand_transform;
  hand_rotation <<  hand_transform(0,0), hand_transform(0,1), hand_transform(0,2),
                    hand_transform(1,0), hand_transform(1,1), hand_transform(1,2),
                    hand_transform(2,0), hand_transform(2,1), hand_transform(2,2);
  hand_translation << hand_transform(0,3), hand_transform(1,3), hand_transform(2,3);
  inverse_hand_transform << hand_rotation.transpose(), -hand_rotation.transpose()*hand_translation,  // from khalil's book page 21
                            0, 0, 0, 1;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_in_hand_frame_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(object_mesh_vertices_xyzrgb, *object_cloud_in_hand_frame_xyzrgb, inverse_hand_transform);
  //*augmented_cloud += *object_cloud_in_hand_frame_xyzrgb;
  //double euclidean_distance;
  for(unsigned int i=0; i<object_cloud_in_hand_frame_xyzrgb->size(); i++){
    euclidean_distance = pow( palm_center_point_in_hand_frame(0)-object_cloud_in_hand_frame_xyzrgb->points[i].x ,2) + 
                         pow( palm_center_point_in_hand_frame(1)-object_cloud_in_hand_frame_xyzrgb->points[i].y ,2) + 
                         pow( palm_center_point_in_hand_frame(2)-object_cloud_in_hand_frame_xyzrgb->points[i].z ,2);
    if( euclidean_distance < distance_to_palm_center_point ){
      nearest_object_point_to_palm_center_point = object_cloud_in_hand_frame_xyzrgb->points[i];
      distance_to_palm_center_point = euclidean_distance;
    }
  }
  
  // use the palm center point to get the distance to object, the translate the hand in +ve x-axis to touch the object
  // convert nearest point found in previous step to the transformed hand frame
  Eigen::Vector3f dummy_vector;
  dummy_vector << nearest_object_point_to_palm_center_point.x-palm_center_point_in_hand_frame(0), 
                  nearest_object_point_to_palm_center_point.y-palm_center_point_in_hand_frame(1),
                  nearest_object_point_to_palm_center_point.z-palm_center_point_in_hand_frame(2);
  
  // translate the hand to touch the object at nearest point
  dummy_translation << dummy_vector(0),0,0;
  dummy_transform << Eigen::Matrix3f::Identity(), dummy_translation,
                     0,0,0,1;
  hand_transform = hand_transform*dummy_transform;

  //pcl::transformPointCloud(*allegro_hand_workspace_ellipsoids_xyzrgb, *hand_transformed_cloud, hand_transform);
  
  
  
  
  // TO DEBUG
  ///////
  /////// DONOT do the transform to see the workspace in hand frame !!!
  
  
  pcl::transformPointCloud(*allegro_hand_xyzrgb, *allegro_hand_xyzrgb, hand_transform);
  /*
  pcl::transformPointCloud(*thumb_workspace_convex_xyzrgb, *thumb_workspace_convex_xyzrgb, hand_transform);
  pcl::transformPointCloud(*index_workspace_convex_xyzrgb, *index_workspace_convex_xyzrgb, hand_transform);
  pcl::transformPointCloud(*middle_workspace_convex_xyzrgb, *middle_workspace_convex_xyzrgb, hand_transform);
  pcl::transformPointCloud(*pinky_workspace_convex_xyzrgb, *pinky_workspace_convex_xyzrgb, hand_transform);
  */
  /*
  viewer->addPointCloud<pcl::PointXYZRGB>(thumb_workspace_convex_xyzrgb, red_color, "red cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(index_workspace_convex_xyzrgb, green_color, "green cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(middle_workspace_convex_xyzrgb, blue_color, "blue cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(pinky_workspace_convex_xyzrgb, grey_color, "grey cloud");
  */
  
  
  *augmented_cloud += *allegro_hand_xyzrgb;
  
  
  // very important !!!
  // after transforming the hand, update the object transform with respect to hand
  hand_rotation <<  hand_transform(0,0), hand_transform(0,1), hand_transform(0,2),
                    hand_transform(1,0), hand_transform(1,1), hand_transform(1,2),
                    hand_transform(2,0), hand_transform(2,1), hand_transform(2,2);
  hand_translation << hand_transform(0,3), hand_transform(1,3), hand_transform(2,3);
  inverse_hand_transform << hand_rotation.transpose(), -hand_rotation.transpose()*hand_translation,  // from khalil's book page 21
                            0, 0, 0, 1;
  pcl::transformPointCloud(object_mesh_vertices_xyzrgb, *object_cloud_in_hand_frame_xyzrgb, inverse_hand_transform);
  //*augmented_cloud += *object_cloud_in_hand_frame_xyzrgb;
  
  
  
  /*
  *augmented_cloud += *thumb_workspace_convex_xyzrgb;
  *augmented_cloud += *index_workspace_convex_xyzrgb;
  *augmented_cloud += *middle_workspace_convex_xyzrgb;
  *augmented_cloud += *pinky_workspace_convex_xyzrgb;
  */
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to get the hand to touch object = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  begin = clock();
  // STEP#6
  // filter the convex workspace shapes merging with object surface
  double convex_shape_value, largest_convex_shape_value;
  Eigen::Vector4f object_centroid_point_in_hand_frame_4d;
  object_centroid_point_in_hand_frame_4d << object_centroid_point.x, object_centroid_point.y, object_centroid_point.z, 1;
  object_centroid_point_in_hand_frame_4d = inverse_hand_transform*object_centroid_point_in_hand_frame_4d;
  
  //
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_hand_frame_xyzrgb    (new pcl::PointCloud<pcl::PointXYZRGB>());
  //pcl::transformPointCloud(*object_roi_sub_cloud_in_object_global_xyzrgb, *object_roi_sub_cloud_in_hand_frame_xyzrgb, inverse_hand_transform);
  pcl::transformPointCloud(*object_roi_sub_cloud_in_object_global_xyzrgb_again, *object_roi_sub_cloud_in_hand_frame_xyzrgb, inverse_hand_transform);
  
  
  //*augmented_cloud += *object_roi_sub_cloud_in_hand_frame_xyzrgb;
  //*augmented_cloud += *object_roi_sub_cloud_in_object_global_xyzrgb;
  
  // offset of convex shape (ellipsoid or sphere)
  pcl::PointCloud<pcl::PointXYZ> filtered_thumb_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> filtered_index_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> filtered_middle_workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> filtered_pinky_workspace_convex_offset;
  
  // parameter of convex shape = {a,b,c} for ellipsoid or {r} for sphere
  pcl::PointCloud<pcl::PointXYZ> filtered_thumb_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> filtered_index_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> filtered_middle_workspace_convex_parameter;
  pcl::PointCloud<pcl::PointXYZ> filtered_pinky_workspace_convex_parameter;
  
  // filtered object points
  pcl::PointCloud<pcl::PointXYZRGB> filtered_object_points_near_thumb_in_hand_frame;
  pcl::PointCloud<pcl::PointXYZRGB> filtered_object_points_near_index_in_hand_frame;
  pcl::PointCloud<pcl::PointXYZRGB> filtered_object_points_near_middle_in_hand_frame;
  pcl::PointCloud<pcl::PointXYZRGB> filtered_object_points_near_pinky_in_hand_frame;
  
  //
  pcl::PointXYZ convex_shape_offset_to_save, convex_shape_parameter_to_save;
  pcl::PointXYZRGB object_point_to_save;
  bool point_to_save_exist = false;
  
  // filtering
  for(int i=0; i<finger_list.size(); i++){
    for(unsigned int j=0; j<thumb_workspace_convex_offset.size(); j++){
      largest_convex_shape_value = 0.0;
      point_to_save_exist = false;
      
      // iterate through all object points 
      // (we can do better by iterating through object points in ROI only)
      //for(unsigned int k=0; k<object_cloud_in_hand_frame_xyzrgb->size(); k++){
      for(unsigned int k=0; k<object_roi_sub_cloud_in_hand_frame_xyzrgb->size(); k++){
      
      
      
        if(finger_list[i]=="thumb"){
          convex_shape_offset.x = thumb_workspace_convex_offset.points[j].x;
          convex_shape_offset.y = thumb_workspace_convex_offset.points[j].y;
          convex_shape_offset.z = thumb_workspace_convex_offset.points[j].z;
          convex_shape_parameter.x = thumb_workspace_convex_parameter.points[j].x;
          convex_shape_parameter.y = thumb_workspace_convex_parameter.points[j].y;
          convex_shape_parameter.z = thumb_workspace_convex_parameter.points[j].z;
          
          //convex_shape_value =  pow(object_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          convex_shape_value =  pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          
          if(convex_shape_value < 1.0){
            if(convex_shape_offset.z < object_centroid_point_in_hand_frame_4d(2)){    // filter thumb convex shapes on -ve z-axis (object centroid expressed in hand frame)
              if( largest_convex_shape_value < convex_shape_value ){
                convex_shape_offset_to_save = convex_shape_offset;
                convex_shape_parameter_to_save = convex_shape_parameter;
                //object_point_to_save = object_cloud_in_hand_frame_xyzrgb->points[k];
                object_point_to_save = object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k];
                point_to_save_exist = true;
              }
            }
          }
        }
        
        
        else if(finger_list[i]=="index"){
          convex_shape_offset.x = index_workspace_convex_offset.points[j].x;
          convex_shape_offset.y = index_workspace_convex_offset.points[j].y;
          convex_shape_offset.z = index_workspace_convex_offset.points[j].z;
          convex_shape_parameter.x = index_workspace_convex_parameter.points[j].x;
          convex_shape_parameter.y = index_workspace_convex_parameter.points[j].y;
          convex_shape_parameter.z = index_workspace_convex_parameter.points[j].z;
          
          //convex_shape_value =  pow(object_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          convex_shape_value =  pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          
          if(convex_shape_value < 1.0){
            if(convex_shape_offset.z > object_centroid_point_in_hand_frame_4d(2)){    // filter index convex shapes on +ve z-axis (object centroid expressed in hand frame)
              if( largest_convex_shape_value < convex_shape_value ){
                convex_shape_offset_to_save = convex_shape_offset;
                convex_shape_parameter_to_save = convex_shape_parameter;
                //object_point_to_save = object_cloud_in_hand_frame_xyzrgb->points[k];
                object_point_to_save = object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k];
                point_to_save_exist = true;
              }
            }
          }
        }
        
        
        else if(finger_list[i]=="middle"){
          convex_shape_offset.x = middle_workspace_convex_offset.points[j].x;
          convex_shape_offset.y = middle_workspace_convex_offset.points[j].y;
          convex_shape_offset.z = middle_workspace_convex_offset.points[j].z;
          convex_shape_parameter.x = middle_workspace_convex_parameter.points[j].x;
          convex_shape_parameter.y = middle_workspace_convex_parameter.points[j].y;
          convex_shape_parameter.z = middle_workspace_convex_parameter.points[j].z;
          
          //convex_shape_value =  pow(object_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          convex_shape_value =  pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          
          if(convex_shape_value < 1.0){
            if(convex_shape_offset.z > object_centroid_point_in_hand_frame_4d(2)){    // filter middle convex shapes on +ve z-axis (object centroid expressed in hand frame)
              if( largest_convex_shape_value < convex_shape_value ){
                convex_shape_offset_to_save = convex_shape_offset;
                convex_shape_parameter_to_save = convex_shape_parameter;
                //object_point_to_save = object_cloud_in_hand_frame_xyzrgb->points[k];
                object_point_to_save = object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k];
                point_to_save_exist = true;
              }
            }
          }
        }
        
        
        else if(finger_list[i]=="pinky"){
          convex_shape_offset.x = pinky_workspace_convex_offset.points[j].x;
          convex_shape_offset.y = pinky_workspace_convex_offset.points[j].y;
          convex_shape_offset.z = pinky_workspace_convex_offset.points[j].z;
          convex_shape_parameter.x = pinky_workspace_convex_parameter.points[j].x;
          convex_shape_parameter.y = pinky_workspace_convex_parameter.points[j].y;
          convex_shape_parameter.z = pinky_workspace_convex_parameter.points[j].z;
          
          //convex_shape_value =  pow(object_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
          //                    + pow(object_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          convex_shape_value =  pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].x - convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].y - convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) 
                              + pow(object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k].z - convex_shape_offset.z, 2)/pow(convex_shape_parameter.z, 2);
          
          if(convex_shape_value < 1.0){
            if(convex_shape_offset.z > object_centroid_point_in_hand_frame_4d(2)){    // filter pinky convex shapes on +ve z-axis (object centroid expressed in hand frame)
              if( largest_convex_shape_value < convex_shape_value ){
                convex_shape_offset_to_save = convex_shape_offset;
                convex_shape_parameter_to_save = convex_shape_parameter;
                //object_point_to_save = object_cloud_in_hand_frame_xyzrgb->points[k];
                object_point_to_save = object_roi_sub_cloud_in_hand_frame_xyzrgb->points[k];
                point_to_save_exist = true;
              }
            }
          }
        }
        
      
      }
      
      
      if(finger_list[i]=="thumb" and point_to_save_exist){
        filtered_thumb_workspace_convex_offset.push_back( convex_shape_offset_to_save );
        filtered_thumb_workspace_convex_parameter.push_back( convex_shape_parameter_to_save );
        filtered_object_points_near_thumb_in_hand_frame.push_back( object_point_to_save );
      }
      else if(finger_list[i]=="index" and point_to_save_exist){
        filtered_index_workspace_convex_offset.push_back( convex_shape_offset );
        filtered_index_workspace_convex_parameter.push_back( convex_shape_parameter );
        filtered_object_points_near_index_in_hand_frame.push_back( object_point_to_save );
      }
      else if(finger_list[i]=="middle" and point_to_save_exist){
        filtered_middle_workspace_convex_offset.push_back( convex_shape_offset );
        filtered_middle_workspace_convex_parameter.push_back( convex_shape_parameter );
        filtered_object_points_near_middle_in_hand_frame.push_back( object_point_to_save );
      }
      else if(finger_list[i]=="pinky" and point_to_save_exist){
        filtered_pinky_workspace_convex_offset.push_back( convex_shape_offset );
        filtered_pinky_workspace_convex_parameter.push_back( convex_shape_parameter );
        filtered_object_points_near_pinky_in_hand_frame.push_back( object_point_to_save );
      }
      
      
      
    }
  }
  
  
  
  
  // generate and view the point cloud of the convex workspace
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_thumb_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_index_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_middle_workspace_convex_xyzrgb       (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pinky_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> filtered_red_color  (filtered_thumb_workspace_convex_xyzrgb, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> filtered_green_color(filtered_index_workspace_convex_xyzrgb, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> filtered_blue_color (filtered_middle_workspace_convex_xyzrgb, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> filtered_grey_color (filtered_pinky_workspace_convex_xyzrgb, 100, 100, 100);
  
  std::cout << "filtered_thumb_workspace_convex_offset.size() = "<<  filtered_thumb_workspace_convex_offset.size()  << endl;
  std::cout << "filtered_index_workspace_convex_offset.size() = "<<  filtered_index_workspace_convex_offset.size()  << endl;
  std::cout << "filtered_middle_workspace_convex_offset.size() = "<< filtered_middle_workspace_convex_offset.size() << endl;
  std::cout << "filtered_pinky_workspace_convex_offset.size() = "<<  filtered_pinky_workspace_convex_offset.size()  << endl<< endl;
  
  std::cout << "filtered_object_points_near_thumb_in_hand_frame.size() = "<<  filtered_object_points_near_thumb_in_hand_frame.size()  << endl;
  std::cout << "filtered_object_points_near_index_in_hand_frame.size() = "<<  filtered_object_points_near_index_in_hand_frame.size()  << endl;
  std::cout << "filtered_object_points_near_middle_in_hand_frame.size() = "<< filtered_object_points_near_middle_in_hand_frame.size() << endl;
  std::cout << "filtered_object_points_near_pinky_in_hand_frame.size() = "<<  filtered_object_points_near_pinky_in_hand_frame.size()  << endl<< endl;
  
  point_cloud_samples = 30;
  for(unsigned int j=0; j<filtered_thumb_workspace_convex_offset.size(); j++){
    convex_shape_offset.x = filtered_thumb_workspace_convex_offset.points[j].x;
    convex_shape_offset.y = filtered_thumb_workspace_convex_offset.points[j].y;
    convex_shape_offset.z = filtered_thumb_workspace_convex_offset.points[j].z;
    convex_shape_parameter.x = filtered_thumb_workspace_convex_parameter.points[j].x;
    convex_shape_parameter.y = filtered_thumb_workspace_convex_parameter.points[j].y;
    convex_shape_parameter.z = filtered_thumb_workspace_convex_parameter.points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        filtered_thumb_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        
        filtered_thumb_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0; 
      }
    }
  }
    
  for(unsigned int j=0; j<filtered_index_workspace_convex_offset.size(); j++){
    convex_shape_offset.x = filtered_index_workspace_convex_offset.points[j].x;
    convex_shape_offset.y = filtered_index_workspace_convex_offset.points[j].y;
    convex_shape_offset.z = filtered_index_workspace_convex_offset.points[j].z;
    convex_shape_parameter.x = filtered_index_workspace_convex_parameter.points[j].x;
    convex_shape_parameter.y = filtered_index_workspace_convex_parameter.points[j].y;
    convex_shape_parameter.z = filtered_index_workspace_convex_parameter.points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        filtered_index_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        
        filtered_index_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0; 
      }
    }
  }
    
  
  for(unsigned int j=0; j<filtered_middle_workspace_convex_offset.size(); j++){
    convex_shape_offset.x = filtered_middle_workspace_convex_offset.points[j].x;
    convex_shape_offset.y = filtered_middle_workspace_convex_offset.points[j].y;
    convex_shape_offset.z = filtered_middle_workspace_convex_offset.points[j].z;
    convex_shape_parameter.x = filtered_middle_workspace_convex_parameter.points[j].x;
    convex_shape_parameter.y = filtered_middle_workspace_convex_parameter.points[j].y;
    convex_shape_parameter.z = filtered_middle_workspace_convex_parameter.points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        filtered_middle_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        
        filtered_middle_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0; 
      }
    }
  }
  
  
  for(unsigned int j=0; j<filtered_pinky_workspace_convex_offset.size(); j++){
    convex_shape_offset.x = filtered_pinky_workspace_convex_offset.points[j].x;
    convex_shape_offset.y = filtered_pinky_workspace_convex_offset.points[j].y;
    convex_shape_offset.z = filtered_pinky_workspace_convex_offset.points[j].z;
    convex_shape_parameter.x = filtered_pinky_workspace_convex_parameter.points[j].x;
    convex_shape_parameter.y = filtered_pinky_workspace_convex_parameter.points[j].y;
    convex_shape_parameter.z = filtered_pinky_workspace_convex_parameter.points[j].z;
    
    for(unsigned int k=0; k<point_cloud_samples; k++){
      value_x = (-convex_shape_parameter.x+convex_shape_offset.x) + k*2*convex_shape_parameter.x/point_cloud_samples;
      for(unsigned int l=0; l<point_cloud_samples; l++){
        value_y = (-convex_shape_parameter.y+convex_shape_offset.y) + l*2*convex_shape_parameter.y/point_cloud_samples;
        value_z = convex_shape_offset.z + convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        filtered_pinky_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0;
        
        value_z = convex_shape_offset.z - convex_shape_parameter.z*sqrt( 1 - pow(value_x-convex_shape_offset.x, 2)/pow(convex_shape_parameter.x, 2) - pow(value_y-convex_shape_offset.y, 2)/pow(convex_shape_parameter.y, 2) );
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        
        filtered_pinky_workspace_convex_xyzrgb->points.push_back( point_xyzrgb );
        point_xyzrgb.r = 255;  point_xyzrgb.g = 0;  point_xyzrgb.b = 0; 
      }
    }
  }
  
  pcl::transformPointCloud(*filtered_thumb_workspace_convex_xyzrgb, *filtered_thumb_workspace_convex_xyzrgb, hand_transform);
  pcl::transformPointCloud(*filtered_index_workspace_convex_xyzrgb, *filtered_index_workspace_convex_xyzrgb, hand_transform);
  pcl::transformPointCloud(*filtered_middle_workspace_convex_xyzrgb, *filtered_middle_workspace_convex_xyzrgb, hand_transform);
  pcl::transformPointCloud(*filtered_pinky_workspace_convex_xyzrgb, *filtered_pinky_workspace_convex_xyzrgb, hand_transform);
  
  viewer->addPointCloud<pcl::PointXYZRGB>(filtered_thumb_workspace_convex_xyzrgb, filtered_red_color, "filtered red cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(filtered_index_workspace_convex_xyzrgb, filtered_green_color, "filtered green cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(filtered_middle_workspace_convex_xyzrgb, filtered_blue_color, "filtered blue cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(filtered_pinky_workspace_convex_xyzrgb, filtered_grey_color, "filtered grey cloud");
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to filter convex workspace merging with object surface = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  
  begin = clock();
  // STEP#7
  // OPtimization
  
  // optimization using QuadProg++
  //
  Eigen::MatrixXd G(12,12);
  Eigen::VectorXd g(12);
  
	Eigen::MatrixXd A(10,12);
	Eigen::VectorXd B(10);
	
	quadprogpp::Matrix<double> G2, CE, CI;
  quadprogpp::Vector<double> g0, ce0, ci0, x2;
	int n, m, p;
	char ch;
  
  G << 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2;
  
  A <<  0, -1, 0, 0, 0, 0, 0,  1, 0, 0, 0, 0,    // C#1 : thumb & middle y-coordinates must be almost equal
        0,  1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,    // C#2 : thumb & middle y-coordinates must be almost equal
        -1, 0, 0, 0, 0, 0,  1, 0, 0, 0, 0, 0,    // C#3 : thumb & middle x-coordinates must be almost equal
        1,  0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0,    // C#4 : thumb & middle x-coordinates must be almost equal
        
        0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // C#5 : thumb  z-coordinates should be -ve (with respect to workspace centroid coordinate frame !!!)
        0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0, 0,     // C#6 : index  z-coordinates should be +ve
        0, 0,  0, 0, 0, 0, 0, 0, 1, 0, 0, 0,     // C#7 : middle z-coordinates should be +ve
        0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 1,     // C#8 : pinky  z-coordinates should be +ve
        
        0, 0, 0, 0, 1, 0, 0, -1, 0, 0,  0, 0,    // C#9 : index & middle y-coordinates distance bigger than finger diameter
        0, 0, 0, 0, 0, 0, 0,  1, 0, 0, -1, 0;    // C#10: index & middle y-coordinates distance bigger than finger diameter
  
        //0, 0, 0, 0, -1, 0, 0,  2, 0, 0, -1, 0,   // C#11: index & middle y-coordinates distance must be almost equal to that of pinky and middle to ensure zero moment on object
        //0, 0, 0, 0,  1, 0, 0, -2, 0, 0,  1, 0,   // C#12: index & middle y-coordinates distance must be almost equal to that of pinky and middle to ensure zero moment on object
        //0, 0, 0, 0,  1, 0, 0,  0, 0, 0, -1, 0,   // C#13: index & middle y-coordinates distance must be almost equal to that of pinky and middle to ensure zero moment on object
        //0, 0, 0, 0, -1, 0, 0,  0, 0, 0,  1, 0;   // C#14: index & middle y-coordinates distance must be almost equal to that of pinky and middle to ensure zero moment on object
  
  
  double delta1 = 0.003, delta2 = 0.035, delta3 = 0.005;
  
  //B << delta1, delta1, delta1, delta1, hand_workspace_centroid_point.z, hand_workspace_centroid_point.z, hand_workspace_centroid_point.z, hand_workspace_centroid_point.z;
  //B << delta1, delta1, delta1, delta1, object_centroid_point_in_hand_frame_4d(2), object_centroid_point_in_hand_frame_4d(2), object_centroid_point_in_hand_frame_4d(2), object_centroid_point_in_hand_frame_4d(2), -delta2, -delta2, delta3, delta3, delta3, delta3;
  B << delta1, delta1, delta1, delta1, object_centroid_point_in_hand_frame_4d(2), object_centroid_point_in_hand_frame_4d(2), object_centroid_point_in_hand_frame_4d(2), object_centroid_point_in_hand_frame_4d(2), -delta2, -delta2;
  
  n = 12;
  G2.resize(n, n);
  g0.resize(n);
  
  p = 10;
  CI.resize(n, p);
  ci0.resize(p);
  
  m = 0;
	CE.resize(n, m);
	ce0.resize(m);
	
  x2.resize(n);
  
  
  double Po1x, Po1y, Po1z;  // object points
  double Po2x, Po2y, Po2z;
  double Po3x, Po3y, Po3z;
  double Po4x, Po4y, Po4z;
  double additional_element;
  std::vector<double> cost;
  std::vector<double> thumb_point_x, thumb_point_y, thumb_point_z;
  std::vector<double> index_point_x, index_point_y, index_point_z;
  std::vector<double> middle_point_x, middle_point_y, middle_point_z;
  std::vector<double> pinky_point_x, pinky_point_y, pinky_point_z;
  counter = 0;
  
  /*
  Eigen::Vector4f hand_workspace_centroid_point_in_global_frame;
  hand_workspace_centroid_point_in_global_frame << hand_workspace_centroid_point.x, hand_workspace_centroid_point.y, hand_workspace_centroid_point.z, 1;
  hand_workspace_centroid_point_in_global_frame = hand_transform*hand_workspace_centroid_point_in_global_frame;
  viewer->addCoordinateSystem(0.1,hand_workspace_centroid_point_in_global_frame(0),hand_workspace_centroid_point_in_global_frame(1),hand_workspace_centroid_point_in_global_frame(2));
  */
  
  for(unsigned int i=0; i<filtered_object_points_near_thumb_in_hand_frame.size(); i++){
    Po1x = filtered_object_points_near_thumb_in_hand_frame.points[i].x;
    Po1y = filtered_object_points_near_thumb_in_hand_frame.points[i].y;
    Po1z = filtered_object_points_near_thumb_in_hand_frame.points[i].z;
    for(unsigned int j=0; j<filtered_object_points_near_index_in_hand_frame.size(); j++){
      Po2x = filtered_object_points_near_index_in_hand_frame.points[i].x;
      Po2y = filtered_object_points_near_index_in_hand_frame.points[i].y;
      Po2z = filtered_object_points_near_index_in_hand_frame.points[i].z;
      for(unsigned int k=0; k<filtered_object_points_near_middle_in_hand_frame.size(); k++){
        Po3x = filtered_object_points_near_middle_in_hand_frame.points[i].x;
        Po3y = filtered_object_points_near_middle_in_hand_frame.points[i].y;
        Po3z = filtered_object_points_near_middle_in_hand_frame.points[i].z;
        for(unsigned int l=0; l<filtered_object_points_near_pinky_in_hand_frame.size(); l++){
          Po4x = filtered_object_points_near_pinky_in_hand_frame.points[i].x;
          Po4y = filtered_object_points_near_pinky_in_hand_frame.points[i].y;
          Po4z = filtered_object_points_near_pinky_in_hand_frame.points[i].z;
          
          
          // cost function
	        // 1/2 * xT * G * x + g0 * x
	        // x = [ Pfx Pfy Pfz ]T  -> f: finger index (thumb, index, middle, pinky), x: finger end tip point
	        g(0)= -2*Po1x;
	        g(1)= -2*Po1y;
	        g(2)= -2*Po1z;
	        g(3)= -2*Po2x;
	        g(4)= -2*Po2y;
	        g(5)= -2*Po2z;
	        g(6)= -2*Po3x;
	        g(7)= -2*Po3y;
	        g(8)= -2*Po3z;
	        g(9)= -2*Po4x;
	        g(10)=-2*Po4y;
	        g(11)=-2*Po4z;
	        additional_element = pow(Po1x,2) + pow(Po1y,2) + pow(Po1z,2) + pow(Po2x,2) + pow(Po2y,2) + pow(Po2z,2) + pow(Po3x,2) + pow(Po3y,2) + pow(Po3z,2) + pow(Po4x,2) + pow(Po4y,2) + pow(Po4z,2);
	        
	        {for (int s=0; s<n; s++)	
		        for (int t=0; t<n; t++)
			        G2[s][t] = G(s,t);}
	        {for (int s=0; s<n; s++)
	          g0[s] = g(s);}
	        
          
          // inequality constraints
          // A*x + B >= 0
	        /////////////////////
	        {for (int s=0; s<n; s++)
	          for (int t=0; t<p; t++)
		          CI[s][t] = A(t,s);}
	        {for (int s=0; s<p; s++)
			      ci0[s] = B(s);}
	        
	        
	        // equality constraints (we don't have any, so set to zero!)
	        // A*x + B = 0
	        ///////////////////////
	        {std::istringstream is("0.0, "
													        "0.0 ");
		        for (int s=0; s<n; s++)
			        for (int t=0; t<m; t++)
				        is >> CE[s][t] >> ch;}
          {std::istringstream is("0.0 ");
		        for (int s=0; s<m; s++)
			        is >> ce0[s] >> ch;}
	        
	        
	        // solve
	        cost.push_back( solve_quadprog(G2, g0, CE, ce0, CI, ci0, x2) + additional_element );
	        thumb_point_x.push_back( x2[0] );
	        thumb_point_y.push_back( x2[1] );
	        thumb_point_z.push_back( x2[2] );
	        
	        index_point_x.push_back( x2[3] );
	        index_point_y.push_back( x2[4] );
	        index_point_z.push_back( x2[5] );
	        
	        middle_point_x.push_back( x2[6] );
	        middle_point_y.push_back( x2[7] );
	        middle_point_z.push_back( x2[8] );
	        
	        pinky_point_x.push_back( x2[9] );
	        pinky_point_y.push_back( x2[10] );
	        pinky_point_z.push_back( x2[11] );
	        
	        std::cout << "counter = " << counter << ", cost = " << cost.back() << endl;
          counter++;
          
        }
      }
    }
  }
  
  int index_of_min=0;
  int index_of_max=0;
  for(unsigned int i=0; i<cost.size(); i++){
    if(cost[i]<cost[index_of_min])
      index_of_min = i;
    if(cost[i]>cost[index_of_max])
      index_of_max = i;
  }
  
	std::cout << "index of min element: " << index_of_min << std::endl;
	std::cout << "smallest element    : " << cost[index_of_min] << std::endl;
	
	std::cout << "index of max element: " << index_of_max << std::endl;
	std::cout << "largest element     : " << cost[index_of_max] << std::endl;
	
	
	// Draw the result graping points
	pcl::PointXYZ thumb_grasp_point;
	Eigen::Vector4f thumb_grasp_point_4d;
	std::basic_string<char> id="min sphere thumb";
	thumb_grasp_point.x = thumb_point_x[index_of_min];
	thumb_grasp_point.y = thumb_point_y[index_of_min];
	thumb_grasp_point.z = thumb_point_z[index_of_min];
	thumb_grasp_point_4d << thumb_grasp_point.x, thumb_grasp_point.y, thumb_grasp_point.z, 1;
	thumb_grasp_point_4d = hand_transform*thumb_grasp_point_4d;
	thumb_grasp_point.x = thumb_grasp_point_4d(0);
	thumb_grasp_point.y = thumb_grasp_point_4d(1);
	thumb_grasp_point.z = thumb_grasp_point_4d(2);
	viewer->addSphere<pcl::PointXYZ>(thumb_grasp_point, 0.01, 1.0, 0.0, 0.0, id);
	
	pcl::PointXYZ index_grasp_point;
	Eigen::Vector4f index_grasp_point_4d;
	id="min sphere index";
	index_grasp_point.x = index_point_x[index_of_min];
	index_grasp_point.y = index_point_y[index_of_min];
	index_grasp_point.z = index_point_z[index_of_min];
	index_grasp_point_4d << index_grasp_point.x, index_grasp_point.y, index_grasp_point.z, 1;
	index_grasp_point_4d = hand_transform*index_grasp_point_4d;
	index_grasp_point.x = index_grasp_point_4d(0);
	index_grasp_point.y = index_grasp_point_4d(1);
	index_grasp_point.z = index_grasp_point_4d(2);
	viewer->addSphere<pcl::PointXYZ>(index_grasp_point, 0.01, 0.0, 1.0, 0.0, id);
	
	pcl::PointXYZ middle_grasp_point;
	Eigen::Vector4f middle_grasp_point_4d;
	id="min sphere middle";
	middle_grasp_point.x = middle_point_x[index_of_min];
	middle_grasp_point.y = middle_point_y[index_of_min];
	middle_grasp_point.z = middle_point_z[index_of_min];
	middle_grasp_point_4d << middle_grasp_point.x, middle_grasp_point.y, middle_grasp_point.z, 1;
	middle_grasp_point_4d = hand_transform*middle_grasp_point_4d;
	middle_grasp_point.x = middle_grasp_point_4d(0);
	middle_grasp_point.y = middle_grasp_point_4d(1);
	middle_grasp_point.z = middle_grasp_point_4d(2);
	viewer->addSphere<pcl::PointXYZ>(middle_grasp_point, 0.01, 0.0, 0.0, 1.0, id);
	
	pcl::PointXYZ pinky_grasp_point;
	Eigen::Vector4f pinky_grasp_point_4d;
	id="min sphere pinky";
	pinky_grasp_point.x = pinky_point_x[index_of_min];
	pinky_grasp_point.y = pinky_point_y[index_of_min];
	pinky_grasp_point.z = pinky_point_z[index_of_min];
	pinky_grasp_point_4d << pinky_grasp_point.x, pinky_grasp_point.y, pinky_grasp_point.z, 1;
	pinky_grasp_point_4d = hand_transform*pinky_grasp_point_4d;
	pinky_grasp_point.x = pinky_grasp_point_4d(0);
	pinky_grasp_point.y = pinky_grasp_point_4d(1);
	pinky_grasp_point.z = pinky_grasp_point_4d(2);
	viewer->addSphere<pcl::PointXYZ>(pinky_grasp_point, 0.01, 0.3, 0.3, 0.3, id);
  
  
  
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent by optimization algorithm = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  // add to visualized point cloud
  //*augmented_cloud += *roi_in_global_frame;
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
	
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce();
    //viewer->saveScreenshot("palm_contact.png");
  } 
  return 0;
}

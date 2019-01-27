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
  
  
  
  
  
  
  
  
  begin = clock();
  //
  // creating object coordinate frame
  //
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid;
  pcl::PointXYZ object_centroid_point;
  for(unsigned int i=0;i<object_mesh_vertices.points.size();i++)
    object_centroid.add( object_mesh_vertices.points[i] );
  object_centroid.get(object_centroid_point);
  
  // object major axis estimation (z-axis)
  // generating the object's principal axis (axis of symmetry) we should align the graspable volume around it!
  // get the longest distance accross object vertices in x, y, and z directions, to know along which axis exists the axis of symmetry
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = object_centroid_point;
  far_point_in_neg_direction = object_centroid_point;
  for(unsigned int i=0;i<object_mesh_vertices.size();i++){
    // get the extreme points on object
    if( object_mesh_vertices.points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = object_mesh_vertices.points[i].x;
    if( object_mesh_vertices.points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = object_mesh_vertices.points[i].x;
    
    if( object_mesh_vertices.points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = object_mesh_vertices.points[i].y;
    if( object_mesh_vertices.points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = object_mesh_vertices.points[i].y;
    
    if( object_mesh_vertices.points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = object_mesh_vertices.points[i].z;
    if( object_mesh_vertices.points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = object_mesh_vertices.points[i].z;
  }
  
  // divide the object verices into two sets, one on the positive side of centroid and the other on the negative side
  pcl::PointCloud<pcl::PointXYZ> cloud_far_pos;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_neg;
  pcl::PointXYZ point;
  double distance_to_far_point_pos;
  double distance_to_far_point_neg;
  for(unsigned int i=0;i<object_mesh_vertices.size();i++){
    distance_to_far_point_pos = (object_mesh_vertices.points[i].x - far_point_in_pos_direction.x)*(object_mesh_vertices.points[i].x - far_point_in_pos_direction.x) + 
                                (object_mesh_vertices.points[i].y - far_point_in_pos_direction.y)*(object_mesh_vertices.points[i].y - far_point_in_pos_direction.y) + 
                                (object_mesh_vertices.points[i].z - far_point_in_pos_direction.z)*(object_mesh_vertices.points[i].z - far_point_in_pos_direction.z);
    distance_to_far_point_neg = (object_mesh_vertices.points[i].x - far_point_in_neg_direction.x)*(object_mesh_vertices.points[i].x - far_point_in_neg_direction.x) + 
                                (object_mesh_vertices.points[i].y - far_point_in_neg_direction.y)*(object_mesh_vertices.points[i].y - far_point_in_neg_direction.y) + 
                                (object_mesh_vertices.points[i].z - far_point_in_neg_direction.z)*(object_mesh_vertices.points[i].z - far_point_in_neg_direction.z);
    if( distance_to_far_point_pos < distance_to_far_point_neg )
      cloud_far_pos.points.push_back( object_mesh_vertices.points[i] );
    else
      cloud_far_neg.points.push_back( object_mesh_vertices.points[i] );
  }
  
  // getting the 2 far sides points (centroids)
  pcl::PointXYZ centroid_far_pos_point;
  pcl::PointXYZ centroid_far_neg_point;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg;
  
  for(unsigned int i=0;i<cloud_far_pos.points.size();i++)
    centroid_far_pos.add( cloud_far_pos.points[i] );
  centroid_far_pos.get(centroid_far_pos_point);
  
  for(unsigned int i=0;i<cloud_far_neg.points.size();i++)
    centroid_far_neg.add( cloud_far_neg.points[i] );
  centroid_far_neg.get(centroid_far_neg_point);
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  
  
  
  // computing object's major axis (z-axis) orientation
  // major axis vector
  Eigen::Vector3d object_z_axis_vector;
  object_z_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  object_z_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  object_z_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  
  /*
  // normalize the major axis vector:
  Eigen::Vector3d object_z_axis_vector_normalized;
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  double alpha;   // angle about (+ve) x-axis
  double beta;    // angle about (+ve) y-axis
  double gamma;   // angle about (+ve) z-axis
  alpha = acos( object_z_axis_vector_normalized[0] );
  beta  = acos( object_z_axis_vector_normalized[1] );
  gamma = acos( object_z_axis_vector_normalized[2] );
  //std::cout << "alpha = " << alpha << std::endl;
  //std::cout << "beta  = " << beta << std::endl;
  //std::cout << "gamma = " << gamma << std::endl << std::endl;
  */
  
  // computing perpendicular axis to the major one (x-axis)
  // we have to take into account intersection with y-axis to make it more versatile and robust
  double slope = object_z_axis_vector[1]/object_z_axis_vector[0];
  double y_intercept = centroid_far_pos_point.y - slope*centroid_far_pos_point.x;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_pos_x_axis;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_neg_x_axis;
  for(unsigned int i=0;i<object_mesh_vertices.size();i++){
    if( object_mesh_vertices.points[i].y > (slope*object_mesh_vertices.points[i].x + y_intercept) )
      cloud_far_pos_x_axis.points.push_back( object_mesh_vertices.points[i] );
    else
      cloud_far_neg_x_axis.points.push_back( object_mesh_vertices.points[i] );
  }
  // getting the 2 far sides points (centroids)
  pcl::PointXYZ centroid_far_pos_point_x_axis;
  pcl::PointXYZ centroid_far_neg_point_x_axis;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos_x_axis;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg_x_axis;
  
  for(unsigned int i=0;i<cloud_far_pos_x_axis.points.size();i++)
    centroid_far_pos_x_axis.add( cloud_far_pos_x_axis.points[i] );
  centroid_far_pos_x_axis.get(centroid_far_pos_point_x_axis);
  
  for(unsigned int i=0;i<cloud_far_neg_x_axis.points.size();i++)
    centroid_far_neg_x_axis.add( cloud_far_neg_x_axis.points[i] );
  centroid_far_neg_x_axis.get(centroid_far_neg_point_x_axis);
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  Eigen::Vector3d object_x_axis_vector;
  object_x_axis_vector[0] = centroid_far_pos_point_x_axis.x - centroid_far_neg_point_x_axis.x;
  object_x_axis_vector[1] = centroid_far_pos_point_x_axis.y - centroid_far_neg_point_x_axis.y;
  object_x_axis_vector[2] = centroid_far_pos_point_x_axis.z - centroid_far_neg_point_x_axis.z;
  
  
  
  
  // computing y-axis
  // generating an orthogornal frame out of the z-axis and the x-axis vectors
  Eigen::Vector3d object_x_axis_vector_normalized;
  Eigen::Vector3d object_y_axis_vector_normalized;
  Eigen::Vector3d object_z_axis_vector_normalized;
  Eigen::Vector3d unit_x, unit_y, unit_z;
  unit_x << 1,0,0;
  unit_y << 0,1,0;
  unit_z << 0,0,1;
  object_x_axis_vector_normalized = object_x_axis_vector.normalized();
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  object_y_axis_vector_normalized = object_z_axis_vector_normalized.cross(object_x_axis_vector_normalized);
  object_y_axis_vector_normalized.normalize();
  
  
  
  // compute object transform and show its coordinate system
  Eigen::Matrix3f object_rotation;
  Eigen::Vector3f object_translation;
  Eigen::Matrix4f object_transform;
  
  object_rotation <<  object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x),
                      object_x_axis_vector_normalized.dot(unit_y), object_y_axis_vector_normalized.dot(unit_y), object_z_axis_vector_normalized.dot(unit_y),
                      object_x_axis_vector_normalized.dot(unit_z), object_y_axis_vector_normalized.dot(unit_z), object_z_axis_vector_normalized.dot(unit_z);
  object_translation << object_centroid_point.x,object_centroid_point.y,object_centroid_point.z;
  object_transform << object_rotation, object_translation,
                      0,0,0,1;
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.matrix() = object_transform;
  
  //viewer->addCoordinateSystem(0.1, transform, "object coordinate frame", 0);
  
  std::cout << "object transformation matrix: " << std::endl << object_transform << std::endl << std::endl;
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to estimate object orientation = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  
  
  
  
  
  
  
  
  
  
  
  
  
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
  /*
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color  (thumb_workspace_convex_xyzrgb, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(thumb_workspace_convex_xyzrgb, red_color, "red cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(index_workspace_convex_xyzrgb, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(index_workspace_convex_xyzrgb, green_color, "green cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color (middle_workspace_convex_xyzrgb, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB>(middle_workspace_convex_xyzrgb, blue_color, "blue cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color (pinky_workspace_convex_xyzrgb, 100, 100, 100);
  viewer->addPointCloud<pcl::PointXYZRGB>(pinky_workspace_convex_xyzrgb, grey_color, "grey cloud");
  */
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
  
  
  
  
  
  
  // computing the object major dimentions along its x, y, and z-axes
  // get the centroid first
  pcl::PointXYZRGB centroid_point_transformed_object;
  pcl::CentroidPoint<pcl::PointXYZRGB> centroid_transformed_object;
  for(unsigned int i=0; i<object_cloud_in_object_frame_xyzrgb->size(); i++)
    centroid_transformed_object.add( object_cloud_in_object_frame_xyzrgb->points[i] );
  centroid_transformed_object.get(centroid_point_transformed_object);
  
  // computing the object major dimentions
  pcl::PointXYZRGB object_major_dimensions;
  pcl::PointXYZRGB farest_point_pos;
  pcl::PointXYZRGB farest_point_neg;
  farest_point_pos = centroid_point_transformed_object;
  farest_point_neg = centroid_point_transformed_object;
  for(unsigned int i=0; i<object_cloud_in_object_frame_xyzrgb->size(); i++){
    if( object_cloud_in_object_frame_xyzrgb->points[i].x > farest_point_pos.x )
      farest_point_pos.x = object_cloud_in_object_frame_xyzrgb->points[i].x;
    if( object_cloud_in_object_frame_xyzrgb->points[i].x < farest_point_neg.x )
      farest_point_neg.x = object_cloud_in_object_frame_xyzrgb->points[i].x;
    
    if( object_cloud_in_object_frame_xyzrgb->points[i].y > farest_point_pos.y )
      farest_point_pos.y = object_cloud_in_object_frame_xyzrgb->points[i].y;
    if( object_cloud_in_object_frame_xyzrgb->points[i].y < farest_point_neg.y )
      farest_point_neg.y = object_cloud_in_object_frame_xyzrgb->points[i].y;
    
    if( object_cloud_in_object_frame_xyzrgb->points[i].z > farest_point_pos.z )
      farest_point_pos.z = object_cloud_in_object_frame_xyzrgb->points[i].z;
    if( object_cloud_in_object_frame_xyzrgb->points[i].z < farest_point_neg.z )
      farest_point_neg.z = object_cloud_in_object_frame_xyzrgb->points[i].z;
  }
  object_major_dimensions.x = farest_point_pos.x - farest_point_neg.x;
  object_major_dimensions.y = farest_point_pos.y - farest_point_neg.y;
  object_major_dimensions.z = farest_point_pos.z - farest_point_neg.z;
  std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  
  
  
  
  
  begin = clock();
  //
  // REGION OF INTEREST (ROI)
  // a cuboid volume of width = maximum hand openning, height = finger thickness, length = arbitrary for the moment
  // we scan the object starting from: 1. centroid location 2. at z-axis of hand parallel to either x-axis or y-axis of object (to be determined later!)
  // the whole subset of object cloud with thickness = height of cuboid, must be completely contained in the region of interest cuboid
  double roi_h = 0.14/2;  // 28mm is finger tip diameter - along object z-axis
  //double roi_w = max_hand_opening_approx/2 + 0.1;
  //double roi_l = 0.3/2 + 0.05;
  double roi_w = max_hand_opening_approx/2;
  double roi_l = 0.3/2;
  pcl::PointCloud<pcl::PointXYZRGB> roi_point_cloud;
  
  // draw the special ellipsoid
  // centered at origin with orientation coincident to that of origin frame
  roi_point_cloud.clear();
  point_cloud_samples = 500;
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
  //*augmented_cloud += roi_point_cloud;
  
  // orient the region of interest scanner with the object orientation
  // and translate the roi scanner to the centroid of object
  Eigen::Matrix3f roi_rotation;
  Eigen::Vector3f roi_translation;
  Eigen::Matrix4f roi_transform;
  roi_transform = object_transform;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_in_global_frame (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_in_object_frame (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(roi_point_cloud, *roi_in_global_frame, roi_transform);
  //*augmented_cloud += *roi_in_global_frame;
  
  
  
  
  
  // STEP#1 : Check if the object subcloud is completely contained in the region of interest special ellipsoid
  // obtain the complete object subcloud with height as that of region of interest starting from object centroid
  // to make it easy we use the object point cloud transformed to the object frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_object_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_object_global_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>());
  Eigen::Vector4f object_centroid_point_in_object_frame_4d;
  
  object_centroid_point_in_object_frame_4d << object_centroid_point.x, object_centroid_point.y, object_centroid_point.z, 1;
  object_centroid_point_in_object_frame_4d = inverse_object_transform*object_centroid_point_in_object_frame_4d;
  
  for(unsigned int i=0; i<object_cloud_in_object_frame_xyzrgb->size(); i++){
    if( (object_cloud_in_object_frame_xyzrgb->points[i].z > -roi_h) and (object_cloud_in_object_frame_xyzrgb->points[i].z < roi_h) )
      object_roi_sub_cloud_in_object_frame_xyzrgb->push_back( object_cloud_in_object_frame_xyzrgb->points[i] );
  }
  
  pcl::transformPointCloud(*object_roi_sub_cloud_in_object_frame_xyzrgb, *object_roi_sub_cloud_in_object_global_xyzrgb, object_transform);
  //*augmented_cloud += *object_roi_sub_cloud_in_object_frame_xyzrgb;
  //*augmented_cloud += *object_roi_sub_cloud_in_object_global_xyzrgb;
  
  std::cout << "object_centroid_point_in_object_frame_4d = " << object_centroid_point_in_object_frame_4d.transpose() << std::endl;
  
  // check if the object subcloud is completely contained in the region of interest
  bool object_subcloud_fully_contained = false;
  double special_ellipsoid_value;
  for(unsigned int i=0; i<object_roi_sub_cloud_in_object_frame_xyzrgb->size(); i++){
    special_ellipsoid_value =  pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].x - object_centroid_point_in_object_frame_4d(0), 10)/pow(roi_l, 10) 
                             + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].y - object_centroid_point_in_object_frame_4d(1), 10)/pow(roi_w, 10) 
                             + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].z - object_centroid_point_in_object_frame_4d(2), 2) /pow(roi_h, 2);
    if( special_ellipsoid_value <= 1.0 )
      object_subcloud_fully_contained = true;
    else{
      object_subcloud_fully_contained = false;
      break;
    }
  }
  std::cout << "object_subcloud_fully_contained ... " << object_subcloud_fully_contained << std::endl;
  
  
  
  
  
  // STEP#2 : If NOT completely contained, start scanning action
  // translate from down to top of object major axis (z-axis)
  float linear_scanning_resolution = 0.1;    // in meters
  float rotary_scanning_resolution = 0.3;    // in radians
  int number_of_scan_translations = ceil( object_major_dimensions.z/linear_scanning_resolution );
  int number_of_scan_rotations = ceil( 3.14159/rotary_scanning_resolution );
  Eigen::Vector4f object_far_point_in_neg_direction_4f;
  Eigen::Matrix3f roi_rotation_in_object_frame;
  Eigen::Vector3f roi_translation_in_object_frame;
  Eigen::Matrix4f roi_transform_in_object_frame_incremental;
  
  Eigen::Matrix3f roi_rotation_in_object_frame_total;
  Eigen::Vector3f roi_translation_in_object_frame_total;
  Eigen::Matrix4f roi_transform_in_object_frame = Eigen::Matrix4f::Identity();
  
  object_far_point_in_neg_direction_4f << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  object_far_point_in_neg_direction_4f = object_transform*object_far_point_in_neg_direction_4f;
  // if object is not fully contained in the roi, move the roi scanner (special ellipsoid) to start scanning location
  if(!object_subcloud_fully_contained){
    *roi_in_object_frame = roi_point_cloud;
    
    roi_rotation_in_object_frame = Eigen::Matrix3f::Identity();
    roi_translation_in_object_frame << 0,0,(abs(object_far_point_in_neg_direction_4f(2)-object_centroid_point.z) - roi_h);
    roi_transform_in_object_frame_incremental <<  roi_rotation_in_object_frame, roi_translation_in_object_frame,
                                      0, 0, 0, 1;
    pcl::transformPointCloud(*roi_in_object_frame, *roi_in_object_frame, roi_transform_in_object_frame_incremental);
    
    pcl::transformPointCloud(*roi_in_object_frame, *roi_in_global_frame, roi_transform);
    //*augmented_cloud += *roi_in_global_frame;
  }
  roi_transform_in_object_frame = roi_transform_in_object_frame_incremental;
  
  // STEP#3
  // scanning for roi
  unsigned int counter = 0;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> roi_color  (roi_in_global_frame, 255, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB>(roi_in_global_frame, roi_color, "roi cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> object_subcloud_roi_color  (object_roi_sub_cloud_in_object_frame_xyzrgb, 127, 50, 255);
  //viewer->addPointCloud<pcl::PointXYZRGB>(object_roi_sub_cloud_in_object_frame_xyzrgb, object_subcloud_roi_color, "object subcloud roi cloud");
  dummy_translation = roi_translation_in_object_frame;
  //number_of_scan_translations = 1;
  //number_of_scan_rotations = 1;
  Eigen::Vector4f dummy_translation_4d;
  if(!object_subcloud_fully_contained){
    for(unsigned int i=0; i<number_of_scan_translations; i++){
      if(i) // add translation step except for the first iteration
        roi_translation_in_object_frame(2) += linear_scanning_resolution;
      for(unsigned int j=0; j<number_of_scan_rotations; j++){
        counter += 1;
        std::cout << counter << endl;
        
        roi_rotation_in_object_frame = roi_rotation_in_object_frame*Rotz_float( rotary_scanning_resolution );
        roi_transform_in_object_frame_incremental <<  roi_rotation_in_object_frame, (roi_translation_in_object_frame-dummy_translation),
                                          0, 0, 0, 1;
        dummy_translation = roi_translation_in_object_frame;
        
        // total transform in object frame
        roi_transform_in_object_frame = roi_transform_in_object_frame*roi_transform_in_object_frame_incremental;
        pcl::transformPointCloud(roi_point_cloud, *roi_in_object_frame, roi_transform_in_object_frame);
        
        /*
        dummy_translation_4d << roi_translation_in_object_frame(0), roi_translation_in_object_frame(1), roi_translation_in_object_frame(2), 1;
        dummy_translation_4d = object_transform*dummy_translation_4d;
        viewer->addCoordinateSystem(0.1,dummy_translation_4d(0),dummy_translation_4d(1),dummy_translation_4d(2));
        */
        
        // check if the object subcloud is completely contained in the region of interest
        object_roi_sub_cloud_in_object_frame_xyzrgb->clear();
        for(unsigned int i=0; i<object_cloud_in_object_frame_xyzrgb->size(); i++){
          if( (object_cloud_in_object_frame_xyzrgb->points[i].z > roi_translation_in_object_frame(2)-roi_h) and (object_cloud_in_object_frame_xyzrgb->points[i].z < roi_translation_in_object_frame(2)+roi_h) )
            object_roi_sub_cloud_in_object_frame_xyzrgb->push_back( object_cloud_in_object_frame_xyzrgb->points[i] );
        }
        //*augmented_cloud += *object_roi_sub_cloud_in_object_frame_xyzrgb;
        
        pcl::transformPointCloud(*object_roi_sub_cloud_in_object_frame_xyzrgb, *object_roi_sub_cloud_in_object_global_xyzrgb, object_transform);
        for(unsigned int i=0; i<object_roi_sub_cloud_in_object_frame_xyzrgb->size(); i++){
          special_ellipsoid_value =  pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].x - roi_translation_in_object_frame(0), 10)/pow(roi_l, 10) 
                                   + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].y - roi_translation_in_object_frame(1), 10)/pow(roi_w, 10) 
                                   + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].z - roi_translation_in_object_frame(2), 2) /pow(roi_h, 2);
          if( special_ellipsoid_value <= 1.0 )
            object_subcloud_fully_contained = true;
          else{
            object_subcloud_fully_contained = false;
            break;
          }
        }
        std::cout << "object_subcloud_fully_contained ... " << object_subcloud_fully_contained << std::endl;
        
        // update the roi_in_global_frame to be able to view the point cloud!
        pcl::transformPointCloud(*roi_in_object_frame, *roi_in_global_frame, roi_transform);
        //pcl::transformPointCloud(roi_point_cloud, *roi_in_global_frame, roi_transform);
        viewer->updatePointCloud<pcl::PointXYZRGB>(roi_in_global_frame, roi_color, "roi cloud");
        //viewer->updatePointCloud<pcl::PointXYZRGB>(object_roi_sub_cloud_in_object_frame_xyzrgb, object_subcloud_roi_color, "object subcloud roi cloud");
        viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
        
        viewer->spinOnce();
        usleep(500000);
        
        if(object_subcloud_fully_contained){break;}
      }
      
      if(object_subcloud_fully_contained){break;}
    }
  }
  
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to find region of interest = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  // add to visualized point cloud
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
	
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce();
    viewer->saveScreenshot("region_of_interest.png");
  } 
  return 0;
}

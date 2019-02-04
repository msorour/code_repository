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
  
  // for visualizing polygon mesh (.obj)
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  Hullviewer->setBackgroundColor(0,0,0);
  
  //
  viewer->addCoordinateSystem(0.2);
  Hullviewer->addCoordinateSystem(0.2);
  
  
  viewer->addPointCloud(augmented_cloud,"cloud viewer");
  viewer->setCameraPosition(0.859057, 0.544554, 0.609515, -0.0194028, 0.0614014, -0.0200482, -0.38203, -0.231229, 0.894755, 0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud viewer");
  
  Hullviewer->setCameraPosition(0.859057, 0.544554, 0.609515, -0.0194028, 0.0614014, -0.0200482, -0.38203, -0.231229, 0.894755, 0);
  
  
  begin = clock();
  // loading mesh to save development time!
  pcl::PolygonMesh object_mesh;
  pcl::io::loadOBJFile(object_name+".obj", object_mesh );
  
  // getting the vertices of the polygon mesh loaded
  pcl::PointCloud<pcl::PointXYZ> object_mesh_vertices;
  pcl::PointCloud<pcl::PointXYZRGB> object_mesh_vertices_xyzrgb;
  pcl::fromPCLPointCloud2(object_mesh.cloud, object_mesh_vertices);
  
  copyPointCloud(object_mesh_vertices, object_mesh_vertices_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  *augmented_cloud += object_mesh_vertices_xyzrgb;
  
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
  double largest_euclidean_distance_pos=0.0;
  double largest_euclidean_distance_neg=0.0;
  double euclidean_distance;
  
  // TRIAL#1
  /*
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
  */
  
  // TRIAL#2
  /*
  for(unsigned int i=0;i<object_mesh_vertices.size();i++){
    // get the extreme points on object
    euclidean_distance = pow(object_mesh_vertices.points[i].x-object_centroid_point.x,2) + pow(object_mesh_vertices.points[i].y-object_centroid_point.y,2) + pow(object_mesh_vertices.points[i].z-object_centroid_point.z,2);
    if( object_mesh_vertices.points[i].x > object_centroid_point.x and object_mesh_vertices.points[i].y > object_centroid_point.y and object_mesh_vertices.points[i].z > object_centroid_point.z ){
      if( euclidean_distance > largest_euclidean_distance_pos ){
        far_point_in_pos_direction.x = object_mesh_vertices.points[i].x;
        far_point_in_pos_direction.y = object_mesh_vertices.points[i].y;
        far_point_in_pos_direction.z = object_mesh_vertices.points[i].z;
        largest_euclidean_distance_pos = euclidean_distance;
      }
    }
  if( object_mesh_vertices.points[i].x < object_centroid_point.x and object_mesh_vertices.points[i].y < object_centroid_point.y and object_mesh_vertices.points[i].z < object_centroid_point.z ){
    if( euclidean_distance > largest_euclidean_distance_neg ){
        far_point_in_neg_direction.x = object_mesh_vertices.points[i].x;
        far_point_in_neg_direction.y = object_mesh_vertices.points[i].y;
        far_point_in_neg_direction.z = object_mesh_vertices.points[i].z;
        largest_euclidean_distance_neg = euclidean_distance;
      }
    }
  }
  */
  
  // TRIAL#3 (Best till now, there is always small error but it is robust)
  // get the extreme points on object
  std::vector<double> eu_distance;
  int index_of_max_pos=0;
  int index_of_max_neg=0;
  
  // get farthest point from centroid
  for(unsigned int i=0;i<object_mesh_vertices.size();i++)
    eu_distance.push_back( pow(object_mesh_vertices.points[i].x-object_centroid_point.x,2) + pow(object_mesh_vertices.points[i].y-object_centroid_point.y,2) + pow(object_mesh_vertices.points[i].z-object_centroid_point.z,2) );
  
  // fetch this point (name it: "far point in positive side")
  for(unsigned int i=0; i<eu_distance.size(); i++)
    if(eu_distance[i]>eu_distance[index_of_max_pos])
      index_of_max_pos = i;
  
  // get the farthest point from "far point in positive side"
  eu_distance.clear();
  for(unsigned int i=0;i<object_mesh_vertices.size();i++)
    eu_distance.push_back( pow(object_mesh_vertices.points[i].x-object_mesh_vertices.points[index_of_max_pos].x,2) + pow(object_mesh_vertices.points[i].y-object_mesh_vertices.points[index_of_max_pos].y,2) + pow(object_mesh_vertices.points[i].z-object_mesh_vertices.points[index_of_max_pos].z,2) );
  
  // fetch the point (name it: "far point in negative side")
  for(unsigned int i=0; i<eu_distance.size(); i++)
    if(eu_distance[i]>eu_distance[index_of_max_neg])
      index_of_max_neg = i;
  
  far_point_in_pos_direction.x = object_mesh_vertices.points[index_of_max_pos].x;
  far_point_in_pos_direction.y = object_mesh_vertices.points[index_of_max_pos].y;
  far_point_in_pos_direction.z = object_mesh_vertices.points[index_of_max_pos].z;
  
  far_point_in_neg_direction.x = object_mesh_vertices.points[index_of_max_neg].x;
  far_point_in_neg_direction.y = object_mesh_vertices.points[index_of_max_neg].y;
  far_point_in_neg_direction.z = object_mesh_vertices.points[index_of_max_neg].z;
  
  //viewer->addCoordinateSystem(0.1, far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z);
  //viewer->addCoordinateSystem(0.1, far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z);
  
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
  
  /*
  std::basic_string<char> name = "z-axis (object's longest dimension)";
  viewer->addArrow<pcl::PointXYZ>(centroid_far_pos_point, centroid_far_neg_point, 0.0, 0.0, 1.0, 0, name, 0);
  
  name = "x axis";
  viewer->addArrow<pcl::PointXYZ>(centroid_far_pos_point_x_axis, centroid_far_neg_point_x_axis, 1.0, 0.0, 0.0, 0, name, 0);
  */
  
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
  
  
  
  
  
  
  
  // compute the object far points in object frame then make correction
  Eigen::Vector4f far_point_in_pos_direction_in_object_frame;
  Eigen::Vector4f far_point_in_neg_direction_in_object_frame;
  Eigen::Matrix4f inverse_object_transform;
  inverse_object_transform << object_rotation.transpose(), -object_rotation.transpose()*object_translation,  // from khalil's book page 21
                              0, 0, 0, 1;
  
  far_point_in_pos_direction_in_object_frame << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_in_object_frame << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  far_point_in_pos_direction_in_object_frame = inverse_object_transform*far_point_in_pos_direction_in_object_frame;
  far_point_in_neg_direction_in_object_frame = inverse_object_transform*far_point_in_neg_direction_in_object_frame;
  
  //std::cout << "far_point_in_pos_direction_in_object_frame: " << std::endl << far_point_in_pos_direction_in_object_frame << std::endl;
  //std::cout << "far_point_in_neg_direction_in_object_frame: " << std::endl << far_point_in_neg_direction_in_object_frame << std::endl;
  
  // set x and y components to 0 and leave the z component
  far_point_in_pos_direction_in_object_frame(0) = 0;
  far_point_in_pos_direction_in_object_frame(1) = 0;
  far_point_in_neg_direction_in_object_frame(0) = 0;
  far_point_in_neg_direction_in_object_frame(1) = 0;
  
  // convert to global frame
  Eigen::Vector4f far_point_in_pos_direction_in_global_frame;
  Eigen::Vector4f far_point_in_neg_direction_in_global_frame;
  
  far_point_in_pos_direction_in_global_frame = object_transform*far_point_in_pos_direction_in_object_frame;
  far_point_in_neg_direction_in_global_frame = object_transform*far_point_in_neg_direction_in_object_frame;
  
  
  
  
  
  //
  //
  // use the updated points to compute a more accurate object frame
  far_point_in_pos_direction.x = far_point_in_pos_direction_in_global_frame(0);
  far_point_in_pos_direction.y = far_point_in_pos_direction_in_global_frame(1);
  far_point_in_pos_direction.z = far_point_in_pos_direction_in_global_frame(2);
  
  far_point_in_neg_direction.x = far_point_in_neg_direction_in_global_frame(0);
  far_point_in_neg_direction.y = far_point_in_neg_direction_in_global_frame(1);
  far_point_in_neg_direction.z = far_point_in_neg_direction_in_global_frame(2);
  
  viewer->addCoordinateSystem(0.1, far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z);
  viewer->addCoordinateSystem(0.1, far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z);
  
  // divide the object verices into two sets, one on the positive side of centroid and the other on the negative side
  cloud_far_pos.clear();
  cloud_far_neg.clear();
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
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos_again;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg_again;
  for(unsigned int i=0;i<cloud_far_pos.points.size();i++)
    centroid_far_pos_again.add( cloud_far_pos.points[i] );
  centroid_far_pos_again.get(centroid_far_pos_point);
  for(unsigned int i=0;i<cloud_far_neg.points.size();i++)
    centroid_far_neg.add( cloud_far_neg.points[i] );
  centroid_far_neg.get(centroid_far_neg_point);
  
  // computing object's major axis (z-axis) orientation
  // major axis vector
  object_z_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  object_z_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  object_z_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  
  // computing perpendicular axis to the major one (x-axis)
  // we have to take into account intersection with y-axis to make it more versatile and robust
  slope = object_z_axis_vector[1]/object_z_axis_vector[0];
  y_intercept = centroid_far_pos_point.y - slope*centroid_far_pos_point.x;
  cloud_far_pos_x_axis.clear();
  cloud_far_neg_x_axis.clear();
  for(unsigned int i=0;i<object_mesh_vertices.size();i++){
    if( object_mesh_vertices.points[i].y > (slope*object_mesh_vertices.points[i].x + y_intercept) )
      cloud_far_pos_x_axis.points.push_back( object_mesh_vertices.points[i] );
    else
      cloud_far_neg_x_axis.points.push_back( object_mesh_vertices.points[i] );
  }
  
  // getting the 2 far sides points (centroids)
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_pos_x_axis_again;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_far_neg_x_axis_again;
  for(unsigned int i=0;i<cloud_far_pos_x_axis.points.size();i++)
    centroid_far_pos_x_axis_again.add( cloud_far_pos_x_axis.points[i] );
  centroid_far_pos_x_axis_again.get(centroid_far_pos_point_x_axis);
  for(unsigned int i=0;i<cloud_far_neg_x_axis.points.size();i++)
    centroid_far_neg_x_axis_again.add( cloud_far_neg_x_axis.points[i] );
  centroid_far_neg_x_axis_again.get(centroid_far_neg_point_x_axis);
  
  object_x_axis_vector[0] = centroid_far_pos_point_x_axis.x - centroid_far_neg_point_x_axis.x;
  object_x_axis_vector[1] = centroid_far_pos_point_x_axis.y - centroid_far_neg_point_x_axis.y;
  object_x_axis_vector[2] = centroid_far_pos_point_x_axis.z - centroid_far_neg_point_x_axis.z;
  
  
  /*
  std::basic_string<char> name = "z-axis (object's longest dimension)";
  viewer->addArrow<pcl::PointXYZ>(centroid_far_pos_point, centroid_far_neg_point, 0.0, 0.0, 1.0, 0, name, 0);
  
  name = "x axis";
  viewer->addArrow<pcl::PointXYZ>(centroid_far_pos_point_x_axis_again, centroid_far_neg_point_x_axis, 1.0, 0.0, 0.0, 0, name, 0);
  */
  
  // computing y-axis
  // generating an orthogornal frame out of the z-axis and the x-axis vectors
  unit_x << 1,0,0;
  unit_y << 0,1,0;
  unit_z << 0,0,1;
  object_x_axis_vector_normalized = object_x_axis_vector.normalized();
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  object_y_axis_vector_normalized = object_z_axis_vector_normalized.cross(object_x_axis_vector_normalized);
  object_y_axis_vector_normalized.normalize();
  
  // compute object transform and show its coordinate system
  object_rotation <<  object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x),
                      object_x_axis_vector_normalized.dot(unit_y), object_y_axis_vector_normalized.dot(unit_y), object_z_axis_vector_normalized.dot(unit_y),
                      object_x_axis_vector_normalized.dot(unit_z), object_y_axis_vector_normalized.dot(unit_z), object_z_axis_vector_normalized.dot(unit_z);
  object_translation << object_centroid_point.x,object_centroid_point.y,object_centroid_point.z;
  object_transform << object_rotation, object_translation,
                      0,0,0,1;
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.matrix() = object_transform;
  
  viewer->addCoordinateSystem(0.1, transform, "object coordinate frame", 0);
  
  std::cout << "object transformation matrix: " << std::endl << object_transform << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  end = clock();
	
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	
	std::cout << "time spent to estimate object orientation = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  // add to visualized point cloud
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  
  
	std::vector<pcl::visualization::Camera> cam; 
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce();
    
    //viewer->saveScreenshot("object_pose_approximation.png");
    //viewer->saveScreenshot("object_pose_approximation_z_axis.png");
    //viewer->saveScreenshot("object_pose_approximation_xz_axis.png");
    Hullviewer->saveScreenshot("object_pose_approximation_hull.png");
    
    // use the code below to get the camera settings required to adjust orientation of view
	  // this works perfectly !
	  //Save the position of the camera           
    //viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  } 
  return 0;
}

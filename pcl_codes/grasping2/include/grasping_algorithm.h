/*
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
*/


void step_4_object_pose_approximation( pcl::PointCloud<pcl::PointXYZ> object_cloud_in_global_frame, 
                                       Eigen::Matrix4f& object_transform, 
                                       Eigen::Vector4f& far_point_in_pos_direction_in_global_frame, 
                                       Eigen::Vector4f& far_point_in_neg_direction_in_global_frame, 
                                       Eigen::Vector3f& object_major_dimensions ){
  clock_t begin, end;
  begin = clock();
  double time_spent;
  
  //std::cout << "size of object point cloud: " << object_cloud_in_global_frame.size() << std::endl << std::endl;
  
  
  
  
  //
  // creating object coordinate frame
  //
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid;
  pcl::PointXYZ object_centroid_point;
  for(unsigned int i=0;i<object_cloud_in_global_frame.points.size();i++)
    object_centroid.add( object_cloud_in_global_frame.points[i] );
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
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    // get the extreme points on object
    if( object_cloud_in_global_frame.points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = object_cloud_in_global_frame.points[i].x;
    if( object_cloud_in_global_frame.points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = object_cloud_in_global_frame.points[i].x;
    
    if( object_cloud_in_global_frame.points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = object_cloud_in_global_frame.points[i].y;
    if( object_cloud_in_global_frame.points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = object_cloud_in_global_frame.points[i].y;
    
    if( object_cloud_in_global_frame.points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = object_cloud_in_global_frame.points[i].z;
    if( object_cloud_in_global_frame.points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = object_cloud_in_global_frame.points[i].z;
  }
  */
  
  // TRIAL#2
  /*
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    // get the extreme points on object
    euclidean_distance = pow(object_cloud_in_global_frame.points[i].x-object_centroid_point.x,2) + pow(object_cloud_in_global_frame.points[i].y-object_centroid_point.y,2) + pow(object_cloud_in_global_frame.points[i].z-object_centroid_point.z,2);
    if( object_cloud_in_global_frame.points[i].x > object_centroid_point.x and object_cloud_in_global_frame.points[i].y > object_centroid_point.y and object_cloud_in_global_frame.points[i].z > object_centroid_point.z ){
      if( euclidean_distance > largest_euclidean_distance_pos ){
        far_point_in_pos_direction.x = object_cloud_in_global_frame.points[i].x;
        far_point_in_pos_direction.y = object_cloud_in_global_frame.points[i].y;
        far_point_in_pos_direction.z = object_cloud_in_global_frame.points[i].z;
        largest_euclidean_distance_pos = euclidean_distance;
      }
    }
  if( object_cloud_in_global_frame.points[i].x < object_centroid_point.x and object_cloud_in_global_frame.points[i].y < object_centroid_point.y and object_cloud_in_global_frame.points[i].z < object_centroid_point.z ){
    if( euclidean_distance > largest_euclidean_distance_neg ){
        far_point_in_neg_direction.x = object_cloud_in_global_frame.points[i].x;
        far_point_in_neg_direction.y = object_cloud_in_global_frame.points[i].y;
        far_point_in_neg_direction.z = object_cloud_in_global_frame.points[i].z;
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
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++)
    eu_distance.push_back( pow(object_cloud_in_global_frame.points[i].x-object_centroid_point.x,2) + pow(object_cloud_in_global_frame.points[i].y-object_centroid_point.y,2) + pow(object_cloud_in_global_frame.points[i].z-object_centroid_point.z,2) );
  
  // fetch this point (name it: "far point in positive side")
  for(unsigned int i=0; i<eu_distance.size(); i++)
    if(eu_distance[i]>eu_distance[index_of_max_pos])
      index_of_max_pos = i;
  
  // get the farthest point from "far point in positive side"
  eu_distance.clear();
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++)
    eu_distance.push_back( pow(object_cloud_in_global_frame.points[i].x-object_cloud_in_global_frame.points[index_of_max_pos].x,2) + pow(object_cloud_in_global_frame.points[i].y-object_cloud_in_global_frame.points[index_of_max_pos].y,2) + pow(object_cloud_in_global_frame.points[i].z-object_cloud_in_global_frame.points[index_of_max_pos].z,2) );
  
  // fetch the point (name it: "far point in negative side")
  for(unsigned int i=0; i<eu_distance.size(); i++)
    if(eu_distance[i]>eu_distance[index_of_max_neg])
      index_of_max_neg = i;
  
  far_point_in_pos_direction.x = object_cloud_in_global_frame.points[index_of_max_pos].x;
  far_point_in_pos_direction.y = object_cloud_in_global_frame.points[index_of_max_pos].y;
  far_point_in_pos_direction.z = object_cloud_in_global_frame.points[index_of_max_pos].z;
  
  far_point_in_neg_direction.x = object_cloud_in_global_frame.points[index_of_max_neg].x;
  far_point_in_neg_direction.y = object_cloud_in_global_frame.points[index_of_max_neg].y;
  far_point_in_neg_direction.z = object_cloud_in_global_frame.points[index_of_max_neg].z;
  /*
  object_major_dimensions <<  far_point_in_pos_direction.x-far_point_in_neg_direction.x,
                              far_point_in_pos_direction.y-far_point_in_neg_direction.y,
                              far_point_in_pos_direction.z-far_point_in_neg_direction.z;
  
  std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  */
  // divide the object verices into two sets, one on the positive side of centroid and the other on the negative side
  pcl::PointCloud<pcl::PointXYZ> cloud_far_pos;
  pcl::PointCloud<pcl::PointXYZ> cloud_far_neg;
  pcl::PointXYZ point;
  double distance_to_far_point_pos;
  double distance_to_far_point_neg;
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    distance_to_far_point_pos = (object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z);
    distance_to_far_point_neg = (object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z);
    if( distance_to_far_point_pos < distance_to_far_point_neg )
      cloud_far_pos.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg.points.push_back( object_cloud_in_global_frame.points[i] );
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
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    if( object_cloud_in_global_frame.points[i].y > (slope*object_cloud_in_global_frame.points[i].x + y_intercept) )
      cloud_far_pos_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
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
  
  Eigen::Vector3d object_x_axis_vector;
  object_x_axis_vector[0] = centroid_far_pos_point_x_axis.x - centroid_far_neg_point_x_axis.x;
  object_x_axis_vector[1] = centroid_far_pos_point_x_axis.y - centroid_far_neg_point_x_axis.y;
  object_x_axis_vector[2] = centroid_far_pos_point_x_axis.z - centroid_far_neg_point_x_axis.z;
  
  // however this just computed x-axis vector might not be perfectly perpendicular to the z-axis previously computed
  // we shall project it onto a perpendicular axis to the z-axis
  // first get a unit vector perpendicular to z-axis
  object_x_axis_vector(1) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(1);
  //object_x_axis_vector(2) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(1)*object_z_axis_vector(1) )/object_z_axis_vector(2);
  //object_x_axis_vector(0) = -( object_x_axis_vector(1)*object_z_axis_vector(1) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(0);
  
  //std::cout<<"object_z_axis_vector.dot(object_x_axis_vector) = "<<object_z_axis_vector.dot(object_x_axis_vector)<<std::endl;
  
  //unit_vector_perpendicular_to_z_axis.normalize();
  // then project the computed x_axis onto it
  //Eigen::Matrix3d dummy_rotation;
  //dummy_rotation << object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x);
  
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
  //object_y_axis_vector_normalized = object_z_axis_vector_normalized.cross(object_x_axis_vector_normalized);
  object_y_axis_vector_normalized = object_z_axis_vector.cross(object_x_axis_vector);
  object_y_axis_vector_normalized.normalize();
  
  
  
  // compute object transform and show its coordinate system
  Eigen::Matrix3f object_rotation;
  Eigen::Vector3f object_translation;
  
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
  
  /*
  object_major_dimensions <<  fabs(far_point_in_pos_direction_in_object_frame(0)-far_point_in_neg_direction_in_object_frame(0)),
                              fabs(far_point_in_pos_direction_in_object_frame(1)-far_point_in_neg_direction_in_object_frame(1)),
                              fabs(far_point_in_pos_direction_in_object_frame(2)-far_point_in_neg_direction_in_object_frame(2));
  std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  */
  //std::cout << "far_point_in_pos_direction_in_object_frame: " << std::endl << far_point_in_pos_direction_in_object_frame << std::endl;
  //std::cout << "far_point_in_neg_direction_in_object_frame: " << std::endl << far_point_in_neg_direction_in_object_frame << std::endl;
  
  // set x and y components to 0 and leave the z component
  far_point_in_pos_direction_in_object_frame(0) = 0;
  far_point_in_pos_direction_in_object_frame(1) = 0;
  far_point_in_neg_direction_in_object_frame(0) = 0;
  far_point_in_neg_direction_in_object_frame(1) = 0;
  
  // convert to global frame
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
  
  // divide the object verices into two sets, one on the positive side of centroid and the other on the negative side
  cloud_far_pos.clear();
  cloud_far_neg.clear();
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    distance_to_far_point_pos = (object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_pos_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_pos_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_pos_direction.z);
    distance_to_far_point_neg = (object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x)*(object_cloud_in_global_frame.points[i].x - far_point_in_neg_direction.x) + 
                                (object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y)*(object_cloud_in_global_frame.points[i].y - far_point_in_neg_direction.y) + 
                                (object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z)*(object_cloud_in_global_frame.points[i].z - far_point_in_neg_direction.z);
    if( distance_to_far_point_pos < distance_to_far_point_neg )
      cloud_far_pos.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg.points.push_back( object_cloud_in_global_frame.points[i] );
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
  for(unsigned int i=0;i<object_cloud_in_global_frame.size();i++){
    if( object_cloud_in_global_frame.points[i].y > (slope*object_cloud_in_global_frame.points[i].x + y_intercept) )
      cloud_far_pos_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
    else
      cloud_far_neg_x_axis.points.push_back( object_cloud_in_global_frame.points[i] );
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
  
  // however this just computed x-axis vector might not be perfectly perpendicular to the z-axis previously computed
  // we shall project it onto a perpendicular axis to the z-axis
  // first get a unit vector perpendicular to z-axis
  object_x_axis_vector(1) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(1);
  //object_x_axis_vector(2) = -( object_x_axis_vector(0)*object_z_axis_vector(0) + object_x_axis_vector(1)*object_z_axis_vector(1) )/object_z_axis_vector(2);
  //object_x_axis_vector(0) = -( object_x_axis_vector(1)*object_z_axis_vector(1) + object_x_axis_vector(2)*object_z_axis_vector(2) )/object_z_axis_vector(0);
  
  // computing y-axis
  // generating an orthogornal frame out of the z-axis and the x-axis vectors
  unit_x << 1,0,0;
  unit_y << 0,1,0;
  unit_z << 0,0,1;
  object_x_axis_vector_normalized = object_x_axis_vector.normalized();
  object_z_axis_vector_normalized = object_z_axis_vector.normalized();
  //object_y_axis_vector_normalized = object_z_axis_vector_normalized.cross(object_x_axis_vector_normalized);
  object_y_axis_vector_normalized = object_z_axis_vector.cross(object_x_axis_vector);
  object_y_axis_vector_normalized.normalize();
  
  // compute object transform and show its coordinate system
  object_rotation <<  object_x_axis_vector_normalized.dot(unit_x), object_y_axis_vector_normalized.dot(unit_x), object_z_axis_vector_normalized.dot(unit_x),
                      object_x_axis_vector_normalized.dot(unit_y), object_y_axis_vector_normalized.dot(unit_y), object_z_axis_vector_normalized.dot(unit_y),
                      object_x_axis_vector_normalized.dot(unit_z), object_y_axis_vector_normalized.dot(unit_z), object_z_axis_vector_normalized.dot(unit_z);
  object_translation << object_centroid_point.x,object_centroid_point.y,object_centroid_point.z;
  object_transform << object_rotation, object_translation,
                      0,0,0,1;
  
  
  /*
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.matrix() = object_transform;
  viewer->addCoordinateSystem(0.1, transform, "object coordinate frame", 0);
  */
  //std::cout << "object transformation matrix: " << std::endl << object_transform << std::endl << std::endl;
  
  
  
  
  
  
  // to compute object major dimensions properly
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_in_object_frame_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(object_cloud_in_global_frame, *object_cloud_in_object_frame_xyz, inverse_object_transform);
  
  far_point_in_pos_direction.x = 0.0;
  far_point_in_pos_direction.y = 0.0;
  far_point_in_pos_direction.z = 0.0;
  
  far_point_in_neg_direction.x = 0.0;
  far_point_in_neg_direction.y = 0.0;
  far_point_in_neg_direction.z = 0.0;
  
  for(unsigned int i=0;i<object_cloud_in_object_frame_xyz->size();i++){
    if( object_cloud_in_object_frame_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = object_cloud_in_object_frame_xyz->points[i].x;
    if( object_cloud_in_object_frame_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = object_cloud_in_object_frame_xyz->points[i].x;
    
    if( object_cloud_in_object_frame_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = object_cloud_in_object_frame_xyz->points[i].y;
    if( object_cloud_in_object_frame_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = object_cloud_in_object_frame_xyz->points[i].y;
    
    if( object_cloud_in_object_frame_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = object_cloud_in_object_frame_xyz->points[i].z;
    if( object_cloud_in_object_frame_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = object_cloud_in_object_frame_xyz->points[i].z;
  }
  
  object_major_dimensions <<  fabs(far_point_in_pos_direction.x-far_point_in_neg_direction.x),
                              fabs(far_point_in_pos_direction.y-far_point_in_neg_direction.y),
                              fabs(far_point_in_pos_direction.z-far_point_in_neg_direction.z);
  //std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  
  
  
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to estimate object pose = " << time_spent << std::endl;
}



























void step_5_region_of_interest( pcl::PointCloud<pcl::PointXYZ> object_cloud_in_global_frame, 
                                Eigen::Matrix4f& object_transform, 
                                Eigen::Vector4f& object_far_point_in_pos_direction_in_global_frame, 
                                Eigen::Vector4f& object_far_point_in_neg_direction_in_global_frame, 
                                Eigen::Vector3f& object_major_dimensions, 
                                Eigen::Vector3f& roi_dimensions, 
                                Eigen::Matrix4f& roi_transform_in_object_frame, 
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object_roi_sub_cloud_in_object_global_xyzrgb,
                                Eigen::Matrix4f& hand_transform,
                                Eigen::Vector3f& hand_dimensions ){
  clock_t begin, end;
  begin = clock();
  double time_spent;


  pcl::PointCloud<pcl::PointXYZRGB> object_cloud_in_global_frame_xyzrgb;
  copyPointCloud(object_cloud_in_global_frame, object_cloud_in_global_frame_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  std::cout << "number of vertices: " << object_cloud_in_global_frame.size() << std::endl << std::endl;
  
  
  
  
  
  
  
  
  // object cloud in its own frame
  Eigen::Matrix3f object_rotation;
  Eigen::Vector3f object_translation;
  object_rotation    << object_transform(0,0), object_transform(0,1), object_transform(0,2),
                        object_transform(1,0), object_transform(1,1), object_transform(1,2),
                        object_transform(2,0), object_transform(2,1), object_transform(2,2);
  object_translation << object_transform(0,3), object_transform(1,3), object_transform(2,3);
  
  Eigen::Matrix4f inverse_object_transform;
  inverse_object_transform << object_rotation.transpose(), -object_rotation.transpose()*object_translation,  // from khalil's book page 21
                              0, 0, 0, 1;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_in_object_frame_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(object_cloud_in_global_frame_xyzrgb, *object_cloud_in_object_frame_xyzrgb, inverse_object_transform);
  
  
  
  //
  // REGION OF INTEREST (ROI)
  // a cuboid volume of width = maximum hand openning, height = finger thickness, length = arbitrary for the moment
  // we scan the object starting from: 1. centroid location 2. at z-axis of hand parallel to either x-axis or y-axis of object (to be determined later!)
  // the whole subset of object cloud with thickness = height of cuboid, must be completely contained in the region of interest cuboid
  double roi_l = roi_dimensions(0);
  double roi_w = roi_dimensions(1);
  double roi_h = roi_dimensions(2);
  pcl::PointCloud<pcl::PointXYZRGB> roi_point_cloud;
  
  // orient the region of interest scanner with the object orientation
  // and translate the roi scanner to the centroid of object
  Eigen::Matrix3f roi_rotation;
  Eigen::Vector3f roi_translation;
  
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> object_centroid;
  pcl::PointXYZ object_centroid_point;
  for(unsigned int i=0;i<object_cloud_in_global_frame.points.size();i++)
    object_centroid.add( object_cloud_in_global_frame.points[i] );
  object_centroid.get(object_centroid_point);
  
  // STEP#1 : Check if the object subcloud is completely contained in the region of interest special ellipsoid
  // obtain the complete object subcloud with height as that of region of interest starting from object centroid
  // to make it easy we use the object point cloud transformed to the object frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_object_frame_xyzrgb  (new pcl::PointCloud<pcl::PointXYZRGB>());
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_roi_sub_cloud_in_object_global_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>());
  Eigen::Vector4f object_centroid_point_in_object_frame_4d;
  
  object_centroid_point_in_object_frame_4d << object_centroid_point.x, object_centroid_point.y, object_centroid_point.z, 1;
  object_centroid_point_in_object_frame_4d = inverse_object_transform*object_centroid_point_in_object_frame_4d;
  
  for(unsigned int i=0; i<object_cloud_in_object_frame_xyzrgb->size(); i++){
    if( (object_cloud_in_object_frame_xyzrgb->points[i].z > -roi_h) and (object_cloud_in_object_frame_xyzrgb->points[i].z < roi_h) )
      object_roi_sub_cloud_in_object_frame_xyzrgb->points.push_back( object_cloud_in_object_frame_xyzrgb->points[i] );
  }
  
  pcl::transformPointCloud(*object_roi_sub_cloud_in_object_frame_xyzrgb, *object_roi_sub_cloud_in_object_global_xyzrgb, object_transform);
  
  // check if the object subcloud is completely contained in the region of interest
  bool object_subcloud_fully_contained = false;
  double special_ellipsoid_value;
  for(unsigned int i=0; i<object_roi_sub_cloud_in_object_frame_xyzrgb->size(); i++){
    special_ellipsoid_value =  pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].x - object_centroid_point_in_object_frame_4d(0), 30)/pow(roi_l, 30) 
                             + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].y - object_centroid_point_in_object_frame_4d(1), 30)/pow(roi_w, 30) 
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
  float linear_scanning_resolution = 0.01;    // in meters
  float rotary_scanning_resolution = 0.314159;    // in radians
  int number_of_scan_translations = object_major_dimensions(2)/linear_scanning_resolution;
  std::cout << "number_of_scan_translations = " << number_of_scan_translations << std::endl;
  int number_of_scan_rotations = 3.14159/rotary_scanning_resolution;
  std::cout << "number_of_scan_rotations = " << number_of_scan_rotations << std::endl;
  Eigen::Vector4f object_far_point_in_neg_direction_in_object_frame;
  Eigen::Matrix3f roi_rotation_in_object_frame;
  Eigen::Vector3f roi_translation_in_object_frame;
  Eigen::Matrix4f roi_transform_in_object_frame_incremental;
  
  Eigen::Matrix3f roi_rotation_in_object_frame_total;
  Eigen::Vector3f roi_translation_in_object_frame_total;
  //Eigen::Matrix4f roi_transform_in_object_frame = Eigen::Matrix4f::Identity();
  
  //object_far_point_in_neg_direction_in_object_frame << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  object_far_point_in_neg_direction_in_object_frame = object_far_point_in_neg_direction_in_global_frame;
  object_far_point_in_neg_direction_in_object_frame = inverse_object_transform*object_far_point_in_neg_direction_in_object_frame;
  
  // if object is not fully contained in the roi, move the roi scanner (special ellipsoid) to start scanning location
  if(!object_subcloud_fully_contained){
    roi_rotation_in_object_frame = Eigen::Matrix3f::Identity();
    roi_translation_in_object_frame << 0,0,(object_far_point_in_neg_direction_in_object_frame(2) + roi_h);
    roi_transform_in_object_frame_incremental <<  roi_rotation_in_object_frame, roi_translation_in_object_frame,
                                                  0, 0, 0, 1;
    roi_transform_in_object_frame = roi_transform_in_object_frame_incremental;
  }
  else
    roi_transform_in_object_frame = Eigen::Matrix4f::Identity();
  
  
  // STEP#3
  // scanning for roi
  unsigned int counter = 0;
  Eigen::Vector3f dummy_translation;
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
        //std::cout << counter << endl;
        
        roi_rotation_in_object_frame = roi_rotation_in_object_frame*Rotz_float( rotary_scanning_resolution );
        roi_transform_in_object_frame_incremental <<  roi_rotation_in_object_frame, (roi_translation_in_object_frame-dummy_translation),
                                          0, 0, 0, 1;
        dummy_translation = roi_translation_in_object_frame;
        
        // total transform in object frame
        roi_transform_in_object_frame = roi_transform_in_object_frame*roi_transform_in_object_frame_incremental;
        
        // check if the object subcloud is completely contained in the region of interest
        object_roi_sub_cloud_in_object_frame_xyzrgb->clear();
        for(unsigned int i=0; i<object_cloud_in_object_frame_xyzrgb->size(); i++){
          if( (object_cloud_in_object_frame_xyzrgb->points[i].z > roi_translation_in_object_frame(2)-roi_h) and (object_cloud_in_object_frame_xyzrgb->points[i].z < roi_translation_in_object_frame(2)+roi_h) )
            object_roi_sub_cloud_in_object_frame_xyzrgb->points.push_back( object_cloud_in_object_frame_xyzrgb->points[i] );
        }
        
        pcl::transformPointCloud(*object_roi_sub_cloud_in_object_frame_xyzrgb, *object_roi_sub_cloud_in_object_global_xyzrgb, object_transform);
        for(unsigned int i=0; i<object_roi_sub_cloud_in_object_frame_xyzrgb->size(); i++){
          special_ellipsoid_value =  pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].x - roi_translation_in_object_frame(0), 30)/pow(roi_l, 30) 
                                   + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].y - roi_translation_in_object_frame(1), 30)/pow(roi_w, 30) 
                                   + pow(object_roi_sub_cloud_in_object_frame_xyzrgb->points[i].z - roi_translation_in_object_frame(2), 2) /pow(roi_h, 2);
          if( special_ellipsoid_value <= 1.0 )
            object_subcloud_fully_contained = true;
          else{
            object_subcloud_fully_contained = false;
            break;
          }
        }
        
        //std::cout << "object_subcloud_fully_contained ... " << object_subcloud_fully_contained << std::endl;
        if(object_subcloud_fully_contained){break;}
      }
      
      if(object_subcloud_fully_contained){break;}
    }
  }
  
  
  std::cout << "roi_transform_in_object_frame = " << std::endl << roi_transform_in_object_frame << std::endl;
  
  
  
  // transformation to allign gripper with region of interest
  Eigen::Matrix4f dummy_transform;
  
  dummy_translation << 0,0,0;
  dummy_transform << Rotx_float(-1.57), dummy_translation,
                     0,0,0,1;
  hand_transform = roi_transform_in_object_frame*dummy_transform;
  hand_transform = object_transform*hand_transform;
  
  // apply another translation to finish the allignment
  dummy_translation << -roi_l,0,-hand_dimensions(0)/3;
  dummy_transform << Eigen::Matrix3f::Identity(), dummy_translation,
                     0,0,0,1;
  hand_transform = hand_transform*dummy_transform;
  
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to find region of interest = " << time_spent << std::endl << std::endl;
}
























void construct_special_ellipsoid_point_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& special_ellipsoid_point_cloud, 
                                              Eigen::Vector3f parameter,
                                              Eigen::Vector3f offset,
                                              int point_cloud_samples, 
                                              int power, 
                                              int red_value, 
                                              int green_value, 
                                              int blue_value ){
  special_ellipsoid_point_cloud->clear();
  pcl::PointXYZRGB point_xyzrgb;
  double value_x, value_y, value_z;
  for(unsigned int k=0; k<point_cloud_samples; k++){
    value_x = (-parameter(0) + offset(0)) + k*2*parameter(0)/point_cloud_samples;
    for(unsigned int l=0; l<point_cloud_samples; l++){
      value_y = (-parameter(1) + offset(1)) + l*2*parameter(1)/point_cloud_samples;
      value_z = offset(2) + parameter(2)*sqrt( 1 - pow(value_x-offset(0), power)/pow(parameter(0), power) - pow(value_y-offset(1), power)/pow(parameter(1), power) );
      
      if(!std::isnan(value_z)){
        point_xyzrgb.x = value_x;
        point_xyzrgb.y = value_y;
        point_xyzrgb.z = value_z;
        point_xyzrgb.r = red_value;  point_xyzrgb.g = green_value;  point_xyzrgb.b = blue_value;
        special_ellipsoid_point_cloud->points.push_back( point_xyzrgb );
        
        value_z = offset(2) - parameter(2)*sqrt( 1 - pow(value_x-offset(0), power)/pow(parameter(0), power) - pow(value_y-offset(1), power)/pow(parameter(1), power) );
        if(!std::isnan(value_z)){
          point_xyzrgb.x = value_x;
          point_xyzrgb.y = value_y;
          point_xyzrgb.z = value_z;
          point_xyzrgb.r = red_value;  point_xyzrgb.g = green_value;  point_xyzrgb.b = blue_value;
          special_ellipsoid_point_cloud->points.push_back( point_xyzrgb );
        }
      }
    }
  }
}












void load_allegro_right_hand_workspace_spheres( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& gripper_augmented_workspace_xyzrgb, 
                                                Eigen::Vector3f& gripper_workspace_centroid_point_in_gripper_frame,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& thumb_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& thumb_workspace_convex_offset, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& index_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& index_workspace_convex_offset, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& middle_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& middle_workspace_convex_offset, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& pinky_workspace_convex_parameter,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& pinky_workspace_convex_offset ){
  gripper_augmented_workspace_xyzrgb ->clear();
  thumb_workspace_convex_parameter   ->clear();
  thumb_workspace_convex_offset      ->clear();
  index_workspace_convex_parameter   ->clear();
  index_workspace_convex_offset      ->clear();
  middle_workspace_convex_parameter  ->clear();
  middle_workspace_convex_offset     ->clear();
  pinky_workspace_convex_parameter   ->clear();
  pinky_workspace_convex_offset      ->clear();
  
  // declarations for allegro hand
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr index_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_workspace_convex_xyzrgb       (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pinky_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  dummy_cloud_xyzrgb                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZ> workspace_convex_offset;
  pcl::PointCloud<pcl::PointXYZ> workspace_convex_parameter;
  
  pcl::PointXYZ offset;
  pcl::PointXYZ parameter;
  std::vector<std::string> finger_list;
  
  Eigen::Vector3f parameter_vector, offset_vector;
  
  
  
  
  
  finger_list.push_back("thumb");  finger_list.push_back("index");  finger_list.push_back("middle");  finger_list.push_back("pinky");
  for(int i=0; i<finger_list.size(); i++){
    std::vector<double> data;
    std::string line;
    ifstream convex_workspace_file("../gripper_workspace/"+finger_list[i]+"_workspace_spheres_3.txt");
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
        parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
                
        if(finger_list[i]=="thumb"){
          thumb_workspace_convex_offset->points.push_back( offset );
          thumb_workspace_convex_parameter->points.push_back( parameter );}
        else if(finger_list[i]=="index"){
          index_workspace_convex_offset->points.push_back( offset );
          index_workspace_convex_parameter->points.push_back( parameter );}
        else if(finger_list[i]=="middle"){
          middle_workspace_convex_offset->points.push_back( offset );
          middle_workspace_convex_parameter->points.push_back( parameter );}
        else if(finger_list[i]=="pinky"){
          pinky_workspace_convex_offset->points.push_back( offset );
          pinky_workspace_convex_parameter->points.push_back( parameter );}
      }
    }
    convex_workspace_file.close();
  }
      
  // generate/draw the point cloud of the workspace spheres
  for(int i=0; i<finger_list.size(); i++){
    for(unsigned int j=0; j<thumb_workspace_convex_offset->size(); j++){
      if(finger_list[i]=="thumb"){
        parameter_vector << thumb_workspace_convex_parameter->points[j].x, thumb_workspace_convex_parameter->points[j].y, thumb_workspace_convex_parameter->points[j].z;
        offset_vector    << thumb_workspace_convex_offset->points[j].x, thumb_workspace_convex_offset->points[j].y, thumb_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
        *thumb_workspace_convex_xyzrgb += *dummy_cloud_xyzrgb;
      }
      else if(finger_list[i]=="index"){
        parameter_vector << index_workspace_convex_parameter->points[j].x, index_workspace_convex_parameter->points[j].y, index_workspace_convex_parameter->points[j].z;
        offset_vector    << index_workspace_convex_offset->points[j].x, index_workspace_convex_offset->points[j].y, index_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 0, 255, 0 );
        *index_workspace_convex_xyzrgb += *dummy_cloud_xyzrgb;
      }
      else if(finger_list[i]=="middle"){
        parameter_vector << middle_workspace_convex_parameter->points[j].x, middle_workspace_convex_parameter->points[j].y, middle_workspace_convex_parameter->points[j].z;
        offset_vector    << middle_workspace_convex_offset->points[j].x, middle_workspace_convex_offset->points[j].y, middle_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 0, 0, 255 );
        *middle_workspace_convex_xyzrgb += *dummy_cloud_xyzrgb;
      }
      else if(finger_list[i]=="pinky"){
        parameter_vector << pinky_workspace_convex_parameter->points[j].x, pinky_workspace_convex_parameter->points[j].y, pinky_workspace_convex_parameter->points[j].z;
        offset_vector    << pinky_workspace_convex_offset->points[j].x, pinky_workspace_convex_offset->points[j].y, pinky_workspace_convex_offset->points[j].z;
        construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 100, 100, 100 );
        *pinky_workspace_convex_xyzrgb += *dummy_cloud_xyzrgb;
      }
    }
  }
  *gripper_augmented_workspace_xyzrgb = *thumb_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *index_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *middle_workspace_convex_xyzrgb;
  *gripper_augmented_workspace_xyzrgb += *pinky_workspace_convex_xyzrgb;
  
  
  
  
  
  // gripper augmented workspace centroid location
  pcl::CentroidPoint<pcl::PointXYZRGB> gripper_workspace_centroid;
  pcl::PointXYZRGB gripper_workspace_centroid_point_in_gripper_frame_xyzrgb;
  for(unsigned int i=0;i<gripper_augmented_workspace_xyzrgb->points.size();i++)
    gripper_workspace_centroid.add( gripper_augmented_workspace_xyzrgb->points[i] );
  gripper_workspace_centroid.get(gripper_workspace_centroid_point_in_gripper_frame_xyzrgb);
  gripper_workspace_centroid_point_in_gripper_frame << gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.x, gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.y, gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.z;
  
  
  
  
  
  // filtering thumb workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<thumb_workspace_convex_offset->points.size();i++){
    if(thumb_workspace_convex_offset->points[i].z < gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.z){
      workspace_convex_offset.points.push_back( thumb_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( thumb_workspace_convex_parameter->points[i] );
    }
  }
  thumb_workspace_convex_offset->clear();
  thumb_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    thumb_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    thumb_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  // filtering index workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<index_workspace_convex_offset->points.size();i++){
    if(index_workspace_convex_offset->points[i].z > gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.z){
      workspace_convex_offset.points.push_back( index_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( index_workspace_convex_parameter->points[i] );
    }
  }
  index_workspace_convex_offset->clear();
  index_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    index_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    index_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  // filtering middle workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<middle_workspace_convex_offset->points.size();i++){
    if(middle_workspace_convex_offset->points[i].z > gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.z){
      workspace_convex_offset.points.push_back( middle_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( middle_workspace_convex_parameter->points[i] );
    }
  }
  middle_workspace_convex_offset->clear();
  middle_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    middle_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    middle_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  // filtering pinky workspace spheres
  workspace_convex_offset.clear();
  workspace_convex_parameter.clear();
  for(unsigned int i=0;i<pinky_workspace_convex_offset->points.size();i++){
    if(pinky_workspace_convex_offset->points[i].z > gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.z){
      workspace_convex_offset.points.push_back( pinky_workspace_convex_offset->points[i] );
      workspace_convex_parameter.points.push_back( pinky_workspace_convex_parameter->points[i] );
    }
  }
  pinky_workspace_convex_offset->clear();
  pinky_workspace_convex_parameter->clear();
  for(unsigned int i=0;i<workspace_convex_offset.points.size();i++){
    pinky_workspace_convex_offset->points.push_back( workspace_convex_offset.points[i] );
    pinky_workspace_convex_parameter->points.push_back( workspace_convex_parameter.points[i] );
  }
  
  
  
}




















void load_franka_gripper_workspace_spheres( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& gripper_augmented_workspace_xyzrgb, 
                                            Eigen::Vector3f& gripper_workspace_centroid_point_in_gripper_frame,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& right_finger_workspace_convex_parameter,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& right_finger_workspace_convex_offset, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& left_finger_workspace_convex_parameter,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& left_finger_workspace_convex_offset ){
  gripper_augmented_workspace_xyzrgb      ->clear();
  right_finger_workspace_convex_parameter ->clear();
  right_finger_workspace_convex_offset    ->clear();
  left_finger_workspace_convex_parameter  ->clear();
  left_finger_workspace_convex_offset     ->clear();
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_finger_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_finger_workspace_convex_xyzrgb        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy_cloud_xyzrgb                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointXYZ offset;
  pcl::PointXYZ parameter;
  std::vector<std::string> finger_list;
  Eigen::Vector3f parameter_vector, offset_vector;
  
  
  
  
  
  finger_list.push_back("franka_right_finger");    finger_list.push_back("franka_left_finger");
    
    for(int i=0; i<finger_list.size(); i++){
      std::vector<double> data;
      std::string line;
      ifstream convex_workspace_file("../gripper_workspace/"+finger_list[i]+"_workspace_spheres_10.txt");
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
          parameter.x = data[0];   parameter.y = data[0];   parameter.z = data[0];   offset.x = data[1];   offset.y = data[2];   offset.z = data[3];
                  
          if(finger_list[i]=="franka_right_finger"){
            right_finger_workspace_convex_offset->points.push_back( offset );
            right_finger_workspace_convex_parameter->points.push_back( parameter );}
          else if(finger_list[i]=="franka_left_finger"){
            left_finger_workspace_convex_offset->points.push_back( offset );
            left_finger_workspace_convex_parameter->points.push_back( parameter );}
        }
      }
      convex_workspace_file.close();
    }
    
    // generate/draw the point cloud of the workspace spheres
    for(int i=0; i<finger_list.size(); i++){
      for(unsigned int j=0; j<right_finger_workspace_convex_offset->size(); j++){
        if(finger_list[i]=="franka_right_finger"){
          parameter_vector << right_finger_workspace_convex_parameter->points[j].x, right_finger_workspace_convex_parameter->points[j].y, right_finger_workspace_convex_parameter->points[j].z;
          offset_vector    << right_finger_workspace_convex_offset->points[j].x, right_finger_workspace_convex_offset->points[j].y, right_finger_workspace_convex_offset->points[j].z;
          construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 255, 0, 0 );
          *right_finger_workspace_convex_xyzrgb += *dummy_cloud_xyzrgb;
        }
        else if(finger_list[i]=="franka_left_finger"){
          parameter_vector << left_finger_workspace_convex_parameter->points[j].x, left_finger_workspace_convex_parameter->points[j].y, left_finger_workspace_convex_parameter->points[j].z;
          offset_vector    << left_finger_workspace_convex_offset->points[j].x, left_finger_workspace_convex_offset->points[j].y, left_finger_workspace_convex_offset->points[j].z;
          construct_special_ellipsoid_point_cloud( dummy_cloud_xyzrgb, parameter_vector, offset_vector, 30, 2, 0, 255, 0 );
          *left_finger_workspace_convex_xyzrgb += *dummy_cloud_xyzrgb;
        }
      }
    }
    *gripper_augmented_workspace_xyzrgb = *right_finger_workspace_convex_xyzrgb;
    *gripper_augmented_workspace_xyzrgb += *left_finger_workspace_convex_xyzrgb;
  
  
  
  
  
  // gripper augmented workspace centroid location
  pcl::CentroidPoint<pcl::PointXYZRGB> gripper_workspace_centroid;
  pcl::PointXYZRGB gripper_workspace_centroid_point_in_gripper_frame_xyzrgb;
  for(unsigned int i=0;i<gripper_augmented_workspace_xyzrgb->points.size();i++)
    gripper_workspace_centroid.add( gripper_augmented_workspace_xyzrgb->points[i] );
  gripper_workspace_centroid.get(gripper_workspace_centroid_point_in_gripper_frame_xyzrgb);
  gripper_workspace_centroid_point_in_gripper_frame << gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.x, gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.y, gripper_workspace_centroid_point_in_gripper_frame_xyzrgb.z;
  
}

















void metric_1_number_of_active_workspace_spheres_and_corresponding_object_points( pcl::PointCloud<pcl::PointXYZRGB>::Ptr&  object_cloud_transformed_in_gripper_frame_xyzrgb, 
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_convex_offset,
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_convex_parameter, 
                                                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr&  object_points_in_finger_workspace,
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_active_spheres_offset, 
                                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr&     finger_workspace_active_spheres_parameter ){
  int counter;
  double sphere_value;
  // Evaluation Metric#1 : number of object points && number of workspace spheres
  object_points_in_finger_workspace->clear();
  finger_workspace_active_spheres_offset->clear();
  finger_workspace_active_spheres_parameter->clear();
  // for each thumb workspace sphere
  for(unsigned int l=0; l<finger_workspace_convex_offset->size(); l++){
    counter = 0;
    // for each point in object downsampled cloud
    for(unsigned int k=0; k<object_cloud_transformed_in_gripper_frame_xyzrgb->size(); k++){
      sphere_value = ( pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].x - finger_workspace_convex_offset->points[l].x, 2)
                     + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].y - finger_workspace_convex_offset->points[l].y, 2) 
                     + pow(object_cloud_transformed_in_gripper_frame_xyzrgb->points[k].z - finger_workspace_convex_offset->points[l].z, 2) )/pow(finger_workspace_convex_parameter->points[l].x, 2);
      // add all object points inside this workspace sphere
      if( sphere_value < 1 ){
        object_points_in_finger_workspace->points.push_back( object_cloud_transformed_in_gripper_frame_xyzrgb->points[k] );
        counter++;
      }
    } // end of iteration through all object points
    if(counter > 0){
      //add this sphere to set of active spheres
      finger_workspace_active_spheres_offset->points.push_back   ( finger_workspace_convex_offset    ->points[l] );
      finger_workspace_active_spheres_parameter->points.push_back( finger_workspace_convex_parameter ->points[l] );
    }    
  } // end of iteration through all finger workspace spheres
  
}

















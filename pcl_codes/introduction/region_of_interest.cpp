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
#include <pcl/common/transforms.h>  
#include "../../ros_packages/include/useful_implementations.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>

#include "/home/work/software/QuadProgpp-master/src/Array.hh"
#include "/home/work/software/QuadProgpp-master/src/QuadProg++.hh"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

int main(int argc, char **argv){
  std::string object_name;
  object_name = argv[1];
  //std::string object_name = "cocacola_bottle";
  //std::string object_name = "object_with_major_axis_in_x";
  //std::string object_name = "cocacola_bottle_in_x";
  //std::string object_name = "cocacola_bottle_in_y";
  //std::string object_name = "cocacola_bottle_in_xyz";
  
  Eigen::Matrix4d dummy_transform;
  Eigen::Vector3d dummy_translation;
  Eigen::Matrix3d dummy_rotation;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(object_name+".pcd", *object_cloud_xyz);
  
  copyPointCloud(*object_cloud_xyz, *object_cloud_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  *augmented_cloud = *object_cloud_xyzrgb;
  
  // for visualizing point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("cloud viewer"));
  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  viewer->addPointCloud(augmented_cloud,"object cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object cloud");
  
  std::cout << "object point cloud size: " << object_cloud_xyz->size() << std::endl;
  
  // for visualizing polygon mesh (.obj)
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  Hullviewer->setBackgroundColor(0,0,0);
  
  // load allegro hand workspace as ellipsoids
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_workspace_ellipsoids_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_hand_workspace_as_ellipsoids.pcd", *allegro_hand_workspace_ellipsoids_xyzrgb);
  //Hullviewer->addPointCloud<pcl::PointXYZRGB>(allegro_hand_workspace_ellipsoids_xyzrgb, rgb,"hull viewer");
  
  // load allegro whole workspace point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_workspace_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_hand_workspace.pcd", *allegro_hand_workspace_xyzrgb);
  
  
  
  
  // get the maximum hand opening (approximate)
  // first get centroid
  // get the centroid first
  pcl::PointXYZRGB hand_workspace_centroid_point;
  pcl::CentroidPoint<pcl::PointXYZRGB> hand_workspace_centroid;
  for(unsigned int i=0;i<allegro_hand_workspace_xyzrgb->size();i++)
    hand_workspace_centroid.add( allegro_hand_workspace_xyzrgb->points[i] );
  hand_workspace_centroid.get( hand_workspace_centroid_point );
  viewer->addCoordinateSystem(0.1,hand_workspace_centroid_point.x,hand_workspace_centroid_point.y,hand_workspace_centroid_point.z);
  
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
  
  
  /*
  // generating the concave hull
  pcl::PolygonMesh mesh_out;
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  concave_hull.setInputCloud( object_cloud_xyz );
  concave_hull.setAlpha( 0.01f );
  concave_hull.reconstruct( mesh_out );
  
  // for visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> Hullviewer(new pcl::visualization::PCLVisualizer ("hull viewer"));
  Hullviewer->setBackgroundColor(0,0,0);
  Hullviewer->addPolygonMesh(mesh_out,"hull");
  
  // save the polygon mesh
  pcl::io::saveOBJFile(object_name+".obj", mesh_out );
  
  // getting the vertices of the polygon mesh generated
  pcl::PointCloud<pcl::PointXYZ> poly_mesh_vertices;
  pcl::fromPCLPointCloud2(mesh_out.cloud, poly_mesh_vertices);
  
  std::cout << "number of vertices: " << poly_mesh_vertices.size() << std::endl;
  */
  
  
  
  // loading mesh to save development time!
  pcl::PolygonMesh object_mesh;
  pcl::io::loadOBJFile(object_name+".obj", object_mesh );
  
  // getting the vertices of the polygon mesh loaded
  pcl::PointCloud<pcl::PointXYZ> object_mesh_vertices;
  pcl::fromPCLPointCloud2(object_mesh.cloud, object_mesh_vertices);
  
  
  // override in case of not using .obj files (.ply file for example)
  //pcl::io::loadPLYFile<pcl::PointXYZ>(object_name+".ply", *object_cloud_xyz);
  //object_mesh_vertices = *object_cloud_xyz;
  
  std::cout << "number of vertices: " << object_mesh_vertices.size() << std::endl << std::endl;
  Hullviewer->addPolygonMesh(object_mesh,"hull");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for(unsigned int i=0;i<object_mesh_vertices.points.size();i++)
    centroid.add( object_mesh_vertices.points[i] );
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  
  
  
  // object major axis estimation (z-axis)
  // generating the object's principal axis (axis of symmetry) we should align the graspable volume around it!
  // get the longest distance accross object vertices in x, y, and z directions, to know along which axis exists the axis of symmetry
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = centroid_point;
  far_point_in_neg_direction = centroid_point;
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
  
  // divide the object verices into two sets on in the positive side of centroid and the other on the negative side
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
  
  // draw the approximate axis line
  std::basic_string<char> name = "z-axis (object's longest dimension)";
  viewer->addArrow<pcl::PointXYZ>(centroid_far_pos_point, centroid_far_neg_point, 0.0, 0.0, 1.0, 0, name, 0); 
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  
  /*
  pcl::ModelCoefficients coeff_z;
  coeff_z.values.push_back(centroid_point.x);   // start point location in x
  coeff_z.values.push_back(centroid_point.y);
  coeff_z.values.push_back(centroid_point.z);
  coeff_z.values.push_back(centroid_far_pos_point.x);
  coeff_z.values.push_back(centroid_far_pos_point.y);
  coeff_z.values.push_back(centroid_far_pos_point.z);
  coeff_z.values.push_back(0.1);   // length
  coeff_z.values.push_back(0.005);  // diameter
  viewer->addCylinder(coeff_z, "z-axis");
  viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "z-axis"); 
  viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "z-axis");
  */
  
  
  // computing object major axis orientation
  // major axis vector
  double alpha;   // angle about (+ve) x-axis
  double beta;    // angle about (+ve) y-axis
  double gamma;   // angle about (+ve) z-axis
  Eigen::Vector3d major_axis_vector;
  Eigen::Vector3d major_axis_vector_normalized;
  major_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  major_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  major_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  
  // normalize the major axis vector:
  major_axis_vector_normalized = major_axis_vector.normalized();
  std::cout << "major_axis_vector (before N) = " << std::endl << major_axis_vector << std::endl << std::endl;
  //major_axis_vector.normalize();
  std::cout << "major_axis_vector (after  N) = " << std::endl << major_axis_vector_normalized << std::endl << std::endl;
  alpha = acos( major_axis_vector_normalized[0] );
  beta  = acos( major_axis_vector_normalized[1] );
  gamma = acos( major_axis_vector_normalized[2] );
  /*
  alpha = atan2( major_axis_vector_normalized[2] , major_axis_vector_normalized[0] );
  beta  = atan2( major_axis_vector_normalized[2] , major_axis_vector_normalized[1] );
  gamma = atan2( major_axis_vector_normalized[1] , major_axis_vector_normalized[2] );
  */
  std::cout << "alpha = " << alpha << std::endl;
  std::cout << "beta  = " << beta << std::endl;
  std::cout << "gamma = " << gamma << std::endl << std::endl;
  
  
  
  
  // computing perpendicular axis to the major one (x-axis)
  // we have to take into account intersection with y-axis to make it more versatile and robust
  double slope = major_axis_vector[1]/major_axis_vector[0];
  std::cout << "slope = " << slope << std::endl;
  double y_intercept = centroid_far_pos_point.y - slope*centroid_far_pos_point.x;
  std::cout << "y_intercept = " << y_intercept << std::endl;
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
  
  // draw the x-axis arrow
  name = "x axis";
  viewer->addArrow<pcl::PointXYZ>(centroid_far_pos_point_x_axis, centroid_far_neg_point_x_axis, 1.0, 0.0, 0.0, 0, name, 0);
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  Eigen::Vector3d x_axis_vector;
  Eigen::Vector3d x_axis_vector_normalized;
  x_axis_vector[0] = centroid_far_pos_point_x_axis.x - centroid_far_neg_point_x_axis.x;
  x_axis_vector[1] = centroid_far_pos_point_x_axis.y - centroid_far_neg_point_x_axis.y;
  x_axis_vector[2] = centroid_far_pos_point_x_axis.z - centroid_far_neg_point_x_axis.z;
  x_axis_vector_normalized = x_axis_vector.normalized();
  std::cout << "x_axis_vector (before N) = " << std::endl << x_axis_vector << std::endl << std::endl;
  std::cout << "x_axis_vector (after  N) = " << std::endl << x_axis_vector_normalized << std::endl << std::endl;
  
  
  
  
  
  // trial : this might not be what I want to do afterall !!!
  // what i think i need is to get the centroid of workspace only and use it as the centroid for HAND-CLOUD+WORKSPACE-CLOUD
  pcl::PointXYZRGB workspace_only_centroid_point;
  pcl::CentroidPoint<pcl::PointXYZRGB> workspace_only_centroid;
  
  // iterating the workspace
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr workspace_only_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
  for(unsigned int i=0; i<allegro_hand_workspace_ellipsoids_xyzrgb->size(); i++){
    if( (allegro_hand_workspace_ellipsoids_xyzrgb->points[i].r != 0) or (allegro_hand_workspace_ellipsoids_xyzrgb->points[i].g != 0) or (allegro_hand_workspace_ellipsoids_xyzrgb->points[i].b != 0) ){
      //workspace_only_cloud_xyzrgb->points.push_back( allegro_hand_workspace_ellipsoids_xyzrgb->points[i] );
      if(allegro_hand_workspace_ellipsoids_xyzrgb->points[i].x == allegro_hand_workspace_ellipsoids_xyzrgb->points[i].x and allegro_hand_workspace_ellipsoids_xyzrgb->points[i].y == allegro_hand_workspace_ellipsoids_xyzrgb->points[i].y and allegro_hand_workspace_ellipsoids_xyzrgb->points[i].z == allegro_hand_workspace_ellipsoids_xyzrgb->points[i].z)
        workspace_only_centroid.add( allegro_hand_workspace_ellipsoids_xyzrgb->points[i] );
    }
  }
  // get the centroid point
  workspace_only_centroid.get(workspace_only_centroid_point);
  //viewer->addCoordinateSystem(0.1,workspace_only_centroid_point.x,workspace_only_centroid_point.y,workspace_only_centroid_point.z+0.3);
  std::cout << "workspace_only_centroid_point = " << workspace_only_centroid_point << std::endl;
  
  // transform to different location to be able to see it
  //translation << 0, 0, 0.3;
  //transform <<  Eigen::Matrix3d::Identity(), translation,  // from khalil's book page 21
  //              0, 0, 0, 1;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr workspace_only_cloud_xyzrgb_transfomed(new pcl::PointCloud<pcl::PointXYZRGB>());
  //pcl::transformPointCloud(*workspace_only_cloud_xyzrgb, *workspace_only_cloud_xyzrgb_transfomed, transform);
  //*augmented_cloud += *workspace_only_cloud_xyzrgb_transfomed;
  
  
  
  
  
  
  
  // loading the graspable volume as a set of ellipsoids
  std::vector<double> ellipsoid_a, ellipsoid_b, ellipsoid_c, ellipsoid_offset_x, ellipsoid_offset_y, ellipsoid_offset_z;
  std::vector<double> data;
  std::string line;
  ifstream graspable_volume_ellipoids_file("graspable_volume_as_a_set_of_ellipsoids.txt");
  if(graspable_volume_ellipoids_file.is_open()){
    while(getline( graspable_volume_ellipoids_file, line ) ){
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
      ellipsoid_a.push_back( data[0] );
      ellipsoid_b.push_back( data[1] );
      ellipsoid_c.push_back( data[2] );
      ellipsoid_offset_x.push_back( data[3] );
      ellipsoid_offset_y.push_back( data[4] );
      ellipsoid_offset_z.push_back( data[5] );
    }
  }
  graspable_volume_ellipoids_file.close();
  
  
  /*
  // just to verify the new ellipsoid equation!!
  ellipsoid_a.clear();
  ellipsoid_b.clear();
  ellipsoid_c.clear();
  ellipsoid_offset_x.clear();
  ellipsoid_offset_y.clear();
  ellipsoid_offset_z.clear();
  
  ellipsoid_a.push_back( 0.05 );
  ellipsoid_b.push_back( 0.03 );
  ellipsoid_c.push_back( 0.02 );
  ellipsoid_offset_x.push_back( 0 );
  ellipsoid_offset_y.push_back( 0 );
  ellipsoid_offset_z.push_back( 0 );
  */
  // generate and view the point cloud of the graspable volume ellipsoids
  int ellipsoid_point_cloud_samples = 50;
  double ellipsoid_x, ellipsoid_y, ellipsoid_z;
  pcl::PointCloud<pcl::PointXYZRGB> ellipsoid_point_cloud;
  pcl::PointXYZRGB ellipsoid_point;
  pcl::PointXYZ ellipsoid_offset;
  ellipsoid_point_cloud.clear();
  for(unsigned int j=0; j<ellipsoid_a.size(); j++){
    ellipsoid_offset.x = ellipsoid_offset_x[j];
    ellipsoid_offset.y = ellipsoid_offset_y[j];
    ellipsoid_offset.z = ellipsoid_offset_z[j];
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-ellipsoid_a[j]+ellipsoid_offset.x) + k*2*ellipsoid_a[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-ellipsoid_b[j]+ellipsoid_offset.y) + l*2*ellipsoid_b[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = ellipsoid_offset.z + ellipsoid_c[j]*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(ellipsoid_a[j], 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(ellipsoid_b[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 250;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = ellipsoid_offset.z - ellipsoid_c[j]*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 2)/pow(ellipsoid_a[j], 2) - pow(ellipsoid_y-ellipsoid_offset.y, 2)/pow(ellipsoid_b[j], 2) );
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
  *augmented_cloud += ellipsoid_point_cloud;
  
  /*
  // check special ellipsoid equation !!
  ellipsoid_point_cloud.clear();
  for(unsigned int j=0; j<ellipsoid_a.size(); j++){
    ellipsoid_offset.x = ellipsoid_offset_x[j];
    ellipsoid_offset.y = ellipsoid_offset_y[j];
    ellipsoid_offset.z = ellipsoid_offset_z[j];
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-ellipsoid_a[j]+ellipsoid_offset.x) + k*2*ellipsoid_a[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-ellipsoid_b[j]+ellipsoid_offset.y) + l*2*ellipsoid_b[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = ellipsoid_offset.z + ellipsoid_c[j]*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 10)/pow(ellipsoid_a[j], 10) - pow(ellipsoid_y-ellipsoid_offset.y, 10)/pow(ellipsoid_b[j], 10) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 250;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = ellipsoid_offset.z - ellipsoid_c[j]*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 10)/pow(ellipsoid_a[j], 10) - pow(ellipsoid_y-ellipsoid_offset.y, 10)/pow(ellipsoid_b[j], 10) );
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
  
  *augmented_cloud += ellipsoid_point_cloud;
  */
  /*
  // check if already known point is inside either ellipsoids!
  double normal_ellipsoid_value;
  double special_ellipsoid_value;
  pcl::PointXYZ trial_point;
  trial_point.x = -0.041;
  trial_point.y = -0.021;
  trial_point.z = -0.011;
  viewer->addCoordinateSystem(0.05,trial_point.x,trial_point.y,trial_point.z);
  // equation of ellipsoid
  normal_ellipsoid_value =   pow(trial_point.x - ellipsoid_offset_x[0], 2)/pow(ellipsoid_a[0], 2) 
                           + pow(trial_point.y - ellipsoid_offset_y[0], 2)/pow(ellipsoid_b[0], 2) 
                           + pow(trial_point.z - ellipsoid_offset_z[0], 2)/pow(ellipsoid_c[0], 2);
  if( normal_ellipsoid_value < 1.0 )
    std::cout << "point is inside normal ellipsoid ... " << std::endl;
  else
    std::cout << "point is OUT normal ellipsoid ... " << std::endl;
  
  special_ellipsoid_value =  pow(trial_point.x - ellipsoid_offset_x[0], 10)/pow(ellipsoid_a[0], 10) 
                           + pow(trial_point.y - ellipsoid_offset_y[0], 10)/pow(ellipsoid_b[0], 10) 
                           + pow(trial_point.z - ellipsoid_offset_z[0], 2) /pow(ellipsoid_c[0], 2);
  if( special_ellipsoid_value < 1.0 )
    std::cout << "point is inside special ellipsoid ... " << std::endl;
  else
    std::cout << "point is OUT special ellipsoid ... " << std::endl;
  */
  
  
  
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  // orienting the hand along the object's major axis
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation, rpy;
  translation << 0.0, 0.0, 0.0;
  
  // generating an orthogornal frame out of the major axis vector and the x-axis vector
  // major axis is selected as object's z-axis
  Eigen::Vector3d y_axis_vector_normalized;
  Eigen::Vector3d unit_x, unit_y, unit_z;
  unit_x << 1,0,0;
  unit_y << 0,1,0;
  unit_z << 0,0,1;
  y_axis_vector_normalized = major_axis_vector_normalized.cross(x_axis_vector_normalized);
  y_axis_vector_normalized.normalize();
  std::cout << "x_axis_vector_normalized = " << std::endl << x_axis_vector_normalized << std::endl;
  std::cout << "y_axis_vector_normalized = " << std::endl << y_axis_vector_normalized << std::endl;
  
  // only issue is: if the object and hand are alligned from beginning !?
  // this is a quick fix, need some time to do properly !
  double epsilon = 0.05;
  if( (major_axis_vector_normalized.dot(unit_z)> 1.0-epsilon) and ( (fabs(x_axis_vector_normalized.dot(unit_x))> 1.0-epsilon) or (fabs(x_axis_vector_normalized.dot(unit_y))> 1.0-epsilon) or (fabs(y_axis_vector_normalized.dot(unit_x))> 1.0-epsilon) or (fabs(y_axis_vector_normalized.dot(unit_y))> 1.0-epsilon) ) )
    rotation_matrix = Eigen::Matrix3d::Identity();
  else
    rotation_matrix <<  x_axis_vector_normalized.dot(unit_x), y_axis_vector_normalized.dot(unit_x), major_axis_vector_normalized.dot(unit_x),
                        x_axis_vector_normalized.dot(unit_y), y_axis_vector_normalized.dot(unit_y), major_axis_vector_normalized.dot(unit_y),
                        x_axis_vector_normalized.dot(unit_z), y_axis_vector_normalized.dot(unit_z), major_axis_vector_normalized.dot(unit_z);
  
  // orient the rotation to be perpendicular to the major axis
  //rotation_matrix = rotation_matrix*Roty(-1.57);    // doing this here will affect the object transform and also its major axes dimension estimation !!!
  
  std::cout << "R = " << std::endl << rotation_matrix << std::endl;
  transform << rotation_matrix, translation,
               0, 0, 0, 1;
  
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*allegro_hand_workspace_ellipsoids_xyzrgb, *hand_transformed_cloud, transform);
  
  Eigen::Vector3d output;
  output = rotation_matrix.transpose()*major_axis_vector_normalized;
  output.normalize();
  std::cout << "inverse transform applied = " << std::endl << output << std::endl;
  
  
  
  //
  // projecting object cloud on the origin frame
  Eigen::Matrix4d inverse_transform = Eigen::Matrix4d::Identity();
  inverse_transform <<  rotation_matrix.transpose(), -rotation_matrix.transpose()*translation,  // from khalil's book page 21
                        0, 0, 0, 1;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_mesh_vertices_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  copyPointCloud(object_mesh_vertices, *object_mesh_vertices_xyzrgb);    // converting to rgb will set values to 0 (object color is black)
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_transformed_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*object_mesh_vertices_xyzrgb, *object_transformed_cloud_xyzrgb, inverse_transform);
  pcl::transformPointCloud(object_mesh_vertices, *object_transformed_cloud, inverse_transform);
  //*augmented_cloud += *object_transformed_cloud_xyzrgb;
  *augmented_cloud += *object_mesh_vertices_xyzrgb;
  
  
  
  
  
  
  // computing the object major dimentions along its x, y, and z-axes
  // get the centroid first
  pcl::PointXYZ centroid_point_transformed_object;
  pcl::CentroidPoint<pcl::PointXYZ> centroid_transformed_object;
  for(unsigned int i=0;i<object_transformed_cloud->size();i++)
    centroid_transformed_object.add( object_transformed_cloud->points[i] );
  centroid_transformed_object.get(centroid_point_transformed_object);
  
  // computing the object major dimentions along its x, y, and z-axes
  pcl::PointXYZ object_major_dimensions;
  pcl::PointXYZ farest_point_pos;
  pcl::PointXYZ farest_point_neg;
  farest_point_pos = centroid_point_transformed_object;
  farest_point_neg = centroid_point_transformed_object;
  for(unsigned int i=0; i<object_transformed_cloud->size(); i++){
    if( object_transformed_cloud->points[i].x > farest_point_pos.x )
      farest_point_pos.x = object_transformed_cloud->points[i].x;
    if( object_transformed_cloud->points[i].x < farest_point_neg.x )
      farest_point_neg.x = object_transformed_cloud->points[i].x;
    
    if( object_transformed_cloud->points[i].y > farest_point_pos.y )
      farest_point_pos.y = object_transformed_cloud->points[i].y;
    if( object_transformed_cloud->points[i].y < farest_point_neg.y )
      farest_point_neg.y = object_transformed_cloud->points[i].y;
    
    if( object_transformed_cloud->points[i].z > farest_point_pos.z )
      farest_point_pos.z = object_transformed_cloud->points[i].z;
    if( object_transformed_cloud->points[i].z < farest_point_neg.z )
      farest_point_neg.z = object_transformed_cloud->points[i].z;
  }
  object_major_dimensions.x = farest_point_pos.x - farest_point_neg.x;
  object_major_dimensions.y = farest_point_pos.y - farest_point_neg.y;
  object_major_dimensions.z = farest_point_pos.z - farest_point_neg.z;
  std::cout << "object_major_dimensions = " << std::endl << object_major_dimensions << std::endl;
  
  
  
  
  
  
  
  
  
  
  
  /*
  // next we want to place the workspace_only centroid on the beginning point of the object's major axis
  // set relative orientation 
  // set relative position based on workspace_only centroid location
  // orient the rotation to be perpendicular to the major axis
  Eigen::Matrix3d initial_search_rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d initial_search_transform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d initial_search_translation;
  int index_of_object_axis_with_smallest_dimension;
  double smallest_dimension = 1000000000.0;
  if(smallest_dimension > object_major_dimensions.x){
    smallest_dimension = object_major_dimensions.x;
    index_of_object_axis_with_smallest_dimension = 0;
  }
  if(smallest_dimension > object_major_dimensions.y){
    smallest_dimension = object_major_dimensions.y;
    index_of_object_axis_with_smallest_dimension = 1;
  }
  if(smallest_dimension > object_major_dimensions.z){
    smallest_dimension = object_major_dimensions.z;
    index_of_object_axis_with_smallest_dimension = 2;
  }
  
  // at this point the hand z-axis (frame) is oriented at the same direction as z-axis (frame) of object
  // apply transform to make the hand z-axis parallel to the object's axis of smallest dimension
  // get angle between z-axis of hand and the objects's axis of smallest dimension
  if(index_of_object_axis_with_smallest_dimension == 0)
    initial_search_rotation_matrix = rotation_matrix*Roty(-1.57);
  if(index_of_object_axis_with_smallest_dimension == 1)
    initial_search_rotation_matrix = rotation_matrix*Rotx(-1.57);
  
  initial_search_translation     = translation;
  
  std::cout << "R = " << std::endl << rotation_matrix << std::endl;
  initial_search_transform << initial_search_rotation_matrix, initial_search_translation,
                              0, 0, 0, 1;
  
  pcl::transformPointCloud(*allegro_hand_workspace_ellipsoids_xyzrgb, *hand_transformed_cloud, initial_search_transform);
  */
  
  
  
  
  viewer->addCoordinateSystem(0.2,0,0,0);
  Hullviewer->addCoordinateSystem(0.2,0,0,0);
  
  
  // view the hand workspace
  //*augmented_cloud += *allegro_hand_workspace_ellipsoids_xyzrgb;
  //*augmented_cloud += *hand_transformed_cloud;
  //*augmented_cloud += *allegro_hand_workspace_xyzrgb;
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  //Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  
  
  
  
  
  // region of interest
  // a cuboid volume of width = maximum hand openning, height = finger thickness, length = arbitrary for the moment
  // we scan the object starting from: 1. centroid location 2. at z-axis of hand parallel to either x-axis or y-axis of object (to be determined later!)
  // the whole subset of object cloud with thickness = height of cuboid, must be completely contained in the region of interest cuboid
  double roi_h = 0.14/2;  // 28mm is finger tip diameter - along object z-axis
  double roi_w = max_hand_opening_approx/2;
  double roi_l = 0.3/2;
    
  // draw the special ellipsoid
  // centered at origin with orientation coincident to that of origin frame
  ellipsoid_point_cloud.clear();
  ellipsoid_offset.x = 0;  ellipsoid_offset.y = 0;  ellipsoid_offset.z = 0;
  ellipsoid_point_cloud_samples = 250;
  for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
    ellipsoid_x = (-roi_l+ellipsoid_offset.x) + k*2*roi_l/ellipsoid_point_cloud_samples;
    for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
      ellipsoid_y = (-roi_w+ellipsoid_offset.y) + l*2*roi_w/ellipsoid_point_cloud_samples;
      ellipsoid_z = ellipsoid_offset.z + roi_h*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 10)/pow(roi_l, 10) - pow(ellipsoid_y-ellipsoid_offset.y, 10)/pow(roi_w, 10) );
      ellipsoid_point.x = ellipsoid_x;
      ellipsoid_point.y = ellipsoid_y;
      ellipsoid_point.z = ellipsoid_z;
      ellipsoid_point.r = 250;
      ellipsoid_point.g = 0;
      ellipsoid_point.b = 0;
      ellipsoid_point_cloud.points.push_back( ellipsoid_point );
      
      ellipsoid_z = ellipsoid_offset.z - roi_h*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset.x, 10)/pow(roi_l, 10) - pow(ellipsoid_y-ellipsoid_offset.y, 10)/pow(roi_w, 10) );
      ellipsoid_point.x = ellipsoid_x;
      ellipsoid_point.y = ellipsoid_y;
      ellipsoid_point.z = ellipsoid_z;
      ellipsoid_point.r = 250;
      ellipsoid_point.g = 0;
      ellipsoid_point.b = 0;
      ellipsoid_point_cloud.points.push_back( ellipsoid_point );    
    }
  }
  
  // orient the region of interest scanner with the object orientation
  // and translate the roi scanner to the centroid of object
  Eigen::Matrix3d roi_rotation;
  Eigen::Vector3d roi_translation;
  Eigen::Matrix4d roi_transform;
  roi_rotation = rotation_matrix;
  roi_translation << centroid_point.x, centroid_point.y, centroid_point.z;
  roi_transform <<  roi_rotation, roi_translation,
                    0, 0, 0, 1;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(ellipsoid_point_cloud, *roi_transformed_cloud, roi_transform);
  *augmented_cloud += *roi_transformed_cloud;
  
  // NOT COMPLETE YET, NO SCANNING IS IMPLEMENTED YET, had to start with optimization first !
  // scanning action
  // STEP#1 : Check if the object subcloud is completely contained in the region of interest special ellipsoid
  // obtain the complete object subcloud with height as that of region of interest starting from object centroid
  // to make it easy we use the object transformed to the origin frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_transformed_sub_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
  Eigen::Vector3d object_transformed_centroid_point;
  object_transformed_centroid_point << centroid_point.x, centroid_point.y, centroid_point.z;
  object_transformed_centroid_point = rotation_matrix.transpose()*object_transformed_centroid_point;      // object transformed to origin frame
  for(unsigned int i=0; i<object_transformed_cloud_xyzrgb->size(); i++){
    if( (object_transformed_cloud_xyzrgb->points[i].z > (object_transformed_centroid_point(2)-0.9*roi_h)) and (object_transformed_cloud_xyzrgb->points[i].z < (object_transformed_centroid_point(2)+0.9*roi_h)) )
      object_transformed_sub_cloud_xyzrgb->push_back( object_transformed_cloud_xyzrgb->points[i] );
  }
  *augmented_cloud += *object_transformed_sub_cloud_xyzrgb;
  
  // check if the object subcloud is completely contained in the region of interest
  // we do it at the origin frame (orientation only is the same!) for simplicity
  bool object_subcloud_fully_contained = false;
  double special_ellipsoid_value;
  for(unsigned int i=0; i<object_transformed_sub_cloud_xyzrgb->size(); i++){
    special_ellipsoid_value =  pow(object_transformed_sub_cloud_xyzrgb->points[i].x - object_transformed_centroid_point(0), 10)/pow(roi_l, 10) 
                             + pow(object_transformed_sub_cloud_xyzrgb->points[i].y - object_transformed_centroid_point(1), 10)/pow(roi_w, 10) 
                             + pow(object_transformed_sub_cloud_xyzrgb->points[i].z - object_transformed_centroid_point(2), 2) /pow(roi_h, 2);
    if( special_ellipsoid_value <= 1.0 )
      object_subcloud_fully_contained = true;
    else{
      object_subcloud_fully_contained = false;
      break;
    }
  }
  std::cout << "object_subcloud_fully_contained ... " << object_subcloud_fully_contained << std::endl;
  
  /*
  // STEP#2 : If NOT completely contained, first scan by rotation about the abject's major axis (z-axis)
  // rotate about the object's major axis (z-axis)
  dummy_translation << 0,0,0;
  dummy_transform << Rotz(0.3), dummy_translation, 0,0,0,1;
  //roi_rotation = roi_rotation*Rotz(0.3);
  //roi_transform <<  roi_rotation, roi_translation + roi_rotation*additional_translation,
  //                  0, 0, 0, 1;
  roi_transform = roi_transform*dummy_transform;
  
  // STEP#3 : If rotation about the object does NOT result in completely contained object subcloud, start moving in +ve z-axis then do it all over again
  // translate along the object's major axis (z-axis)
  dummy_translation << 0, 0, 0.05;
  dummy_transform << Eigen::Matrix3d::Identity(), dummy_translation, 0,0,0,1;
  //roi_transform <<  roi_rotation, roi_translation + roi_rotation*dummy_translation,
  //                  0, 0, 0, 1;
  roi_transform = roi_transform*dummy_transform;
  // apply transform on region of interest special ellipsoid
  pcl::transformPointCloud(ellipsoid_point_cloud, *roi_transformed_cloud, roi_transform);
  */
  
  //
  // show the hand alligned with the region of iterest
  Eigen::Matrix3d hand_rotation;
  Eigen::Vector3d hand_translation;
  Eigen::Matrix4d hand_transform;
  dummy_translation << 0,0,0;
  dummy_transform << Rotx(1.57), dummy_translation,
                     0,0,0,1;
  hand_transform = roi_transform*dummy_transform;
  // apply another translation to finish the allignment
  dummy_translation << -roi_l,0,-5*roi_w/4;
  dummy_transform << Eigen::Matrix3d::Identity(), dummy_translation,
                     0,0,0,1;
  hand_transform = hand_transform*dummy_transform;
  
  pcl::transformPointCloud(*allegro_hand_workspace_ellipsoids_xyzrgb, *hand_transformed_cloud, hand_transform);
  
  
  
  
  
  
  
  
  
  
  /*
  // optimization
  quadprogpp::Matrix<double> G, CE, CI;
  quadprogpp::Vector<double> g0, ce0, ci0, x;
	int n, m, p;
	double sum = 0.0;
	char ch;
  
  n = 2;
  G.resize(n, n);
  {
		std::istringstream is("4, -2,"
													"-2, 4 ");
		for (int i = 0; i < n; i++)	
			for (int j = 0; j < n; j++)
				is >> G[i][j] >> ch;
	}
	
  g0.resize(n);
  {
		std::istringstream is("6.0, 0.0 ");
		for (int i = 0; i < n; i++)
			is >> g0[i] >> ch;
	}
  
  m = 1;
  CE.resize(n, m);
	{
		std::istringstream is("1.0, "
													"1.0 ");
		for (int i = 0; i < n; i++)
			for (int j = 0; j < m; j++)
				is >> CE[i][j] >> ch;
	} 
  
  ce0.resize(m);
	{
		std::istringstream is("-3.0 ");
		for (int j = 0; j < m; j++)
			is >> ce0[j] >> ch;
  }
	
	p = 3;
  CI.resize(n, p);
  {
		std::istringstream is("1.0, 0.0, 1.0, "
													"0.0, 1.0, 1.0 ");
		for (int i = 0; i < n; i++)
			for (int j = 0; j < p; j++)
				is >> CI[i][j] >> ch;
	}
  
  ci0.resize(p);
  {
		std::istringstream is("0.0, 0.0, -2.0 ");
		for (int j = 0; j < p; j++)
			is >> ci0[j] >> ch;
	}
  x.resize(n);

  std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
	std::cout << "x: " << x << std::endl;
	
  

	// FOR DOUBLE CHECKING COST since in the solve_quadprog routine the matrix G is modified
	
	{
    std::istringstream is("4, -2,"
													"-2, 4 ");
	
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				is >> G[i][j] >> ch;
	}
	
  std::cout << "Double checking cost: ";
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			sum += x[i] * G[i][j] * x[j];
	sum *= 0.5;	
	
	for (int i = 0; i < n; i++)
		sum += g0[i] * x[i];
	std::cout << sum << std::endl;
  */
  
  
  
  
  
  
  /*
  // get the surface normals of the object
  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(object_mesh_vertices, *input_cloud);    // converting to rgb will set values to 0 (object color is black)
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(input_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute(*cloud_normals);
  
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(input_cloud, cloud_normals, 10, 0.05, "object normals");
  */
  
  
  
  
  //
  // get the nearest object points to the 3 points on palm
  // point#1 palm center point
  Eigen::Vector4d palm_frame_origin_point;
  Eigen::Vector4d palm_center_point, palm_right_point, palm_left_point;
  palm_frame_origin_point << 19.6/2000, 0.0, 0.0475, 1.0;
  palm_center_point << 19.6/2000, 0.0, 0.0475, 1.0;
  palm_right_point  << 19.6/2000, 0.03, 0.0775, 1.0;
  palm_left_point   << 19.6/2000, -0.03, 0.0775, 1.0;
  palm_center_point = hand_transform*palm_center_point;
  palm_right_point  = hand_transform*palm_right_point;
  palm_left_point   = hand_transform*palm_left_point;
  viewer->addCoordinateSystem( 0.05, palm_center_point(0), palm_center_point(1), palm_center_point(2) );
  viewer->addCoordinateSystem( 0.05, palm_right_point(0), palm_right_point(1), palm_right_point(2) );
  viewer->addCoordinateSystem( 0.05, palm_left_point(0), palm_left_point(1), palm_left_point(2) );
  
  // find the nearest 3 points on object to these 3 palm points
  double distance_to_palm_center_point = 1000000;
  double distance_to_palm_right_point = 1000000;
  double distance_to_palm_left_point = 1000000;
  double euclidean_distance;
  pcl::PointXYZRGB nearest_object_point_to_palm_center_point;
  pcl::PointXYZRGB nearest_object_point_to_palm_right_point;
  pcl::PointXYZRGB nearest_object_point_to_palm_left_point;
  for(unsigned int i=0; i<object_mesh_vertices_xyzrgb->size(); i++){
    euclidean_distance = pow( palm_center_point(0)-object_mesh_vertices_xyzrgb->points[i].x ,2) + 
                         pow( palm_center_point(1)-object_mesh_vertices_xyzrgb->points[i].y ,2) + 
                         pow( palm_center_point(2)-object_mesh_vertices_xyzrgb->points[i].z ,2);
    if( euclidean_distance < distance_to_palm_center_point ){
      nearest_object_point_to_palm_center_point = object_mesh_vertices_xyzrgb->points[i];
      distance_to_palm_center_point = euclidean_distance;
    }
    
    euclidean_distance = pow( palm_right_point(0) -object_mesh_vertices_xyzrgb->points[i].x ,2) + 
                         pow( palm_right_point(1) -object_mesh_vertices_xyzrgb->points[i].y ,2) + 
                         pow( palm_right_point(2) -object_mesh_vertices_xyzrgb->points[i].z ,2);
    if( euclidean_distance < distance_to_palm_right_point ){
      nearest_object_point_to_palm_right_point = object_mesh_vertices_xyzrgb->points[i];
      distance_to_palm_right_point = euclidean_distance;
    }
    
    euclidean_distance = pow( palm_left_point(0)  -object_mesh_vertices_xyzrgb->points[i].x ,2) + 
                         pow( palm_left_point(1)  -object_mesh_vertices_xyzrgb->points[i].y ,2) + 
                         pow( palm_left_point(2)  -object_mesh_vertices_xyzrgb->points[i].z ,2);
    if( euclidean_distance < distance_to_palm_left_point ){
      nearest_object_point_to_palm_left_point = object_mesh_vertices_xyzrgb->points[i];
      distance_to_palm_left_point = euclidean_distance;
    }
  }
  
  // visualize nearest object point
  viewer->addCoordinateSystem( 0.05, nearest_object_point_to_palm_center_point.x, nearest_object_point_to_palm_center_point.y, nearest_object_point_to_palm_center_point.z );
  viewer->addCoordinateSystem( 0.05, nearest_object_point_to_palm_right_point.x, nearest_object_point_to_palm_right_point.y, nearest_object_point_to_palm_right_point.z );
  viewer->addCoordinateSystem( 0.05, nearest_object_point_to_palm_left_point.x, nearest_object_point_to_palm_left_point.y, nearest_object_point_to_palm_left_point.z );
  
  // use the palm center point to get the distance to object, the translate the hand in +ve x-axis to touch the object
  // convert nearest point found in previous step to the transformed hand frame
  Eigen::Vector3d dummy_vector;
  dummy_vector << nearest_object_point_to_palm_center_point.x-palm_center_point(0), 
                  nearest_object_point_to_palm_center_point.y-palm_center_point(1),
                  nearest_object_point_to_palm_center_point.z-palm_center_point(2);
  
  // compute euclidean distance then project onto hand x-axis vector
  // construct the hand x-axis vector
  Eigen::Vector4d hand_x_axis_vector, origin_point;
  hand_x_axis_vector << 1,0,0,1;
  origin_point << 0,0,0,1;
  hand_x_axis_vector = hand_transform*hand_x_axis_vector;
  origin_point = hand_transform*origin_point;
  
  pcl::PointXYZ hand_origin_point;
  pcl::PointXYZ hand_x_axis;
  hand_origin_point.x = origin_point(0);
  hand_origin_point.y = origin_point(1);
  hand_origin_point.z = origin_point(2);
  hand_x_axis.x = hand_x_axis_vector(0);
  hand_x_axis.y = hand_x_axis_vector(1);
  hand_x_axis.z = hand_x_axis_vector(2);
  name = "hand x axis";
  viewer->addArrow<pcl::PointXYZ>(hand_x_axis, hand_origin_point, 1.0, 0.0, 0.0, 0, name, 0); 
  
  // project the distance vector onto the hand x-axis vector
  Eigen::Vector3d hand_x_axis_vector_3d;
  hand_x_axis_vector_3d << hand_x_axis_vector(0)-origin_point(0), hand_x_axis_vector(1)-origin_point(1), hand_x_axis_vector(2)-origin_point(2);
  dummy_translation << dummy_vector.dot(hand_x_axis_vector_3d),0,0;
  
  
  // translate the hand to touch the object at nearest point
  //dummy_translation << abs(dummy_vector(0)),0,0;
  //dummy_translation << sqrt(distance_to_palm_center_point),0,0;
  dummy_transform << Eigen::Matrix3d::Identity(), dummy_translation,
                     0,0,0,1;
  hand_transform = hand_transform*dummy_transform;
  pcl::transformPointCloud(*allegro_hand_workspace_ellipsoids_xyzrgb, *hand_transformed_cloud, hand_transform);
  
  
  
  
  // add to visualized point cloud
  *augmented_cloud += *hand_transformed_cloud;
  *augmented_cloud += *roi_transformed_cloud;
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
	
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce();
    //triangulation_viewer->spinOnce();
  } 
  return 0;
}

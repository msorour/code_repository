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
#include <time.h>

int main(int argc, char **argv){
  std::string object_name;
  object_name = argv[1];
  
  clock_t begin, end;
	double time_spent;
  
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
  viewer->addPointCloud(augmented_cloud,rgb,"object cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object cloud");
  
  //
  
  
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
  
  // load allegro hand only
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_right_hand_model_cloud.pcd", *allegro_hand_xyzrgb);
  
  
  
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
  
  begin = clock();
  
  
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
  
  
  end = clock();
	
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	
	std::cout << "time spent to load mesh file = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  begin = clock();
  
  ///////////////
  // SECTION 2 //
  ///////////////
  //
  // object frame
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
  alpha = acos( major_axis_vector_normalized[0] );
  beta  = acos( major_axis_vector_normalized[1] );
  gamma = acos( major_axis_vector_normalized[2] );
  /*std::cout << "alpha = " << alpha << std::endl;
  std::cout << "beta  = " << beta << std::endl;
  std::cout << "gamma = " << gamma << std::endl << std::endl;*/
  
  
  
  
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
  
  
  
  
  /*
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
  */
  
  
  // generate and view the point cloud of the graspable volume ellipsoids
  pcl::PointCloud<pcl::PointXYZRGB> ellipsoid_point_cloud;
  int ellipsoid_point_cloud_samples = 50;
  double ellipsoid_x, ellipsoid_y, ellipsoid_z;
  pcl::PointXYZRGB ellipsoid_point;
  pcl::PointXYZ ellipsoid_offset;
  ellipsoid_point_cloud.clear();
  /*
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
  */
  
  
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  viewer->spinOnce();
  Hullviewer->spinOnce();
  
  end = clock();
	
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	
	std::cout << "time spent to estimate object orientation = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  begin = clock();
  ///////////////
  // SECTION 3 //
  ///////////////
  //
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
  
  
  
  
  viewer->addCoordinateSystem(0.2,0,0,0);
  Hullviewer->addCoordinateSystem(0.2,0,0,0);
  
  
  // view the hand workspace
  //*augmented_cloud += *allegro_hand_workspace_ellipsoids_xyzrgb;
  //*augmented_cloud += *hand_transformed_cloud;
  //*augmented_cloud += *allegro_hand_workspace_xyzrgb;
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  //Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  end = clock();
	
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	
	std::cout << "time spent to orient hand = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  begin = clock();
  ///////////////
  // SECTION 4 //
  ///////////////
  //
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
  //*augmented_cloud += *object_transformed_sub_cloud_xyzrgb;
  
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
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to find region of interest = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  begin = clock();
  ///////////////
  // SECTION 5 //
  ///////////////
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
  //viewer->addCoordinateSystem( 0.05, palm_center_point(0), palm_center_point(1), palm_center_point(2) );
  //viewer->addCoordinateSystem( 0.05, palm_right_point(0), palm_right_point(1), palm_right_point(2) );
  //viewer->addCoordinateSystem( 0.05, palm_left_point(0), palm_left_point(1), palm_left_point(2) );
  
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
  //viewer->addCoordinateSystem( 0.05, nearest_object_point_to_palm_center_point.x, nearest_object_point_to_palm_center_point.y, nearest_object_point_to_palm_center_point.z );
  //viewer->addCoordinateSystem( 0.05, nearest_object_point_to_palm_right_point.x, nearest_object_point_to_palm_right_point.y, nearest_object_point_to_palm_right_point.z );
  //viewer->addCoordinateSystem( 0.05, nearest_object_point_to_palm_left_point.x, nearest_object_point_to_palm_left_point.y, nearest_object_point_to_palm_left_point.z );
  
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
  //viewer->addArrow<pcl::PointXYZ>(hand_x_axis, hand_origin_point, 1.0, 0.0, 0.0, 0, name, 0); 
  
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
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to get the hand to touch object = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  
  begin = clock();
  ///////////////
  // SECTION 6 //
  ///////////////
  // loading hand workspace ellipsoids
  // thumb
  std::vector<double> thumb_ellipsoid_a, thumb_ellipsoid_b, thumb_ellipsoid_c, thumb_ellipsoid_offset_x, thumb_ellipsoid_offset_y, thumb_ellipsoid_offset_z;
  std::vector<double> data;
  std::string line;
  ifstream graspable_volume_ellipoids_file("thumb_workspace_as_ellipsoids.txt");
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
      thumb_ellipsoid_a.push_back( data[0] );
      thumb_ellipsoid_b.push_back( data[1] );
      thumb_ellipsoid_c.push_back( data[2] );
      thumb_ellipsoid_offset_x.push_back( data[3] );
      thumb_ellipsoid_offset_y.push_back( data[4] );
      thumb_ellipsoid_offset_z.push_back( data[5] );
    }
  }
  graspable_volume_ellipoids_file.close();
  
  // index
  std::vector<double> index_ellipsoid_a, index_ellipsoid_b, index_ellipsoid_c, index_ellipsoid_offset_x, index_ellipsoid_offset_y, index_ellipsoid_offset_z;
  graspable_volume_ellipoids_file.open("index_workspace_as_ellipsoids.txt");
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
      index_ellipsoid_a.push_back( data[0] );
      index_ellipsoid_b.push_back( data[1] );
      index_ellipsoid_c.push_back( data[2] );
      index_ellipsoid_offset_x.push_back( data[3] );
      index_ellipsoid_offset_y.push_back( data[4] );
      index_ellipsoid_offset_z.push_back( data[5] );
    }
  }
  graspable_volume_ellipoids_file.close();
  
  // middle
  std::vector<double> middle_ellipsoid_a, middle_ellipsoid_b, middle_ellipsoid_c, middle_ellipsoid_offset_x, middle_ellipsoid_offset_y, middle_ellipsoid_offset_z;
  graspable_volume_ellipoids_file.open("middle_workspace_as_ellipsoids.txt");
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
      middle_ellipsoid_a.push_back( data[0] );
      middle_ellipsoid_b.push_back( data[1] );
      middle_ellipsoid_c.push_back( data[2] );
      middle_ellipsoid_offset_x.push_back( data[3] );
      middle_ellipsoid_offset_y.push_back( data[4] );
      middle_ellipsoid_offset_z.push_back( data[5] );
    }
  }
  graspable_volume_ellipoids_file.close();
  
  // pinky
  std::vector<double> pinky_ellipsoid_a, pinky_ellipsoid_b, pinky_ellipsoid_c, pinky_ellipsoid_offset_x, pinky_ellipsoid_offset_y, pinky_ellipsoid_offset_z;
  graspable_volume_ellipoids_file.open("pinky_workspace_as_ellipsoids.txt");
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
      pinky_ellipsoid_a.push_back( data[0] );
      pinky_ellipsoid_b.push_back( data[1] );
      pinky_ellipsoid_c.push_back( data[2] );
      pinky_ellipsoid_offset_x.push_back( data[3] );
      pinky_ellipsoid_offset_y.push_back( data[4] );
      pinky_ellipsoid_offset_z.push_back( data[5] );
    }
  }
  graspable_volume_ellipoids_file.close();
  
  
  
  
  
  // using only hand model then add workspace ellipsoids
  // below command overrides previous hand clouds 
  pcl::transformPointCloud(*allegro_hand_xyzrgb, *hand_transformed_cloud, hand_transform);
  
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to load hand workspace ellipsoids = " << time_spent << std::endl << std::endl;
  
  
  
  
  /*
  // transform and draw the thumb workspace ellipsoids
  pcl::PointCloud<pcl::PointXYZRGB> thumb_ellipsoid_point_cloud;
  ellipsoid_point_cloud_samples = 30;
  Eigen::Vector4d ellipsoid_offset_vector;
  for(unsigned int j=0; j<thumb_ellipsoid_a.size(); j++){
    ellipsoid_offset_vector << thumb_ellipsoid_offset_x[j], thumb_ellipsoid_offset_y[j], thumb_ellipsoid_offset_z[j], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-thumb_ellipsoid_a[j]+ellipsoid_offset_vector(0)) + k*2*thumb_ellipsoid_a[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-thumb_ellipsoid_b[j]+ellipsoid_offset_vector(1)) + l*2*thumb_ellipsoid_b[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = ellipsoid_offset_vector(2) + thumb_ellipsoid_c[j]*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset_vector(0), 2)/pow(thumb_ellipsoid_a[j], 2) - pow(ellipsoid_y-ellipsoid_offset_vector(1), 2)/pow(thumb_ellipsoid_b[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 250;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        thumb_ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = ellipsoid_offset_vector(2) - thumb_ellipsoid_c[j]*sqrt( 1 - pow(ellipsoid_x-ellipsoid_offset_vector(0), 2)/pow(thumb_ellipsoid_a[j], 2) - pow(ellipsoid_y-ellipsoid_offset_vector(1), 2)/pow(thumb_ellipsoid_b[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 250;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        thumb_ellipsoid_point_cloud.points.push_back( ellipsoid_point );    
      }
    }
  }
  *augmented_cloud += thumb_ellipsoid_point_cloud;
  */
  
  
  
  /*
  ///////////////
  // SECTION 7 //
  ///////////////
  // next filter the ellipsoids that are merging with object vertices
  // register the object vertex nearest to ellipsoid surface in which it is located
  // thumb
  Eigen::Vector4d ellipsoid_offset_vector;
  std::vector<double> thumb_ellipsoid_a_filtered, thumb_ellipsoid_b_filtered, thumb_ellipsoid_c_filtered, thumb_ellipsoid_offset_x_filtered, thumb_ellipsoid_offset_y_filtered, thumb_ellipsoid_offset_z_filtered;
  std::vector<double> vertex_point_filtered_x, vertex_point_filtered_y, vertex_point_filtered_z;
  double ellipsoid_value, largest_ellipsoid_value=0.0;
  double a_to_save, b_to_save, c_to_save, x_offset_to_save, y_offset_to_save, z_offset_to_save;
  Eigen::Vector3d vertex_point_to_save; 
  for( unsigned int i=0; i<thumb_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << thumb_ellipsoid_offset_x[i], thumb_ellipsoid_offset_y[i], thumb_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(thumb_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(thumb_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(thumb_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if( largest_ellipsoid_value < ellipsoid_value ){
          largest_ellipsoid_value = ellipsoid_value;
          a_to_save = thumb_ellipsoid_a[i];
          b_to_save = thumb_ellipsoid_b[i];
          c_to_save = thumb_ellipsoid_c[i];
          x_offset_to_save = ellipsoid_offset_vector(0);
          y_offset_to_save = ellipsoid_offset_vector(1);
          z_offset_to_save = ellipsoid_offset_vector(2);
          vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
          vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
          vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      thumb_ellipsoid_a_filtered.push_back( a_to_save );
      thumb_ellipsoid_b_filtered.push_back( b_to_save );
      thumb_ellipsoid_c_filtered.push_back( c_to_save );
      thumb_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      thumb_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      thumb_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered thumb ellipsoids: " << thumb_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the thumb workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB> thumb_ellipsoid_point_cloud;
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<thumb_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-thumb_ellipsoid_a_filtered[j]+thumb_ellipsoid_offset_x_filtered[j]) + k*2*thumb_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-thumb_ellipsoid_b_filtered[j]+thumb_ellipsoid_offset_y_filtered[j]) + l*2*thumb_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = thumb_ellipsoid_offset_z_filtered[j] + thumb_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-thumb_ellipsoid_offset_x_filtered[j], 2)/pow(thumb_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-thumb_ellipsoid_offset_y_filtered[j], 2)/pow(thumb_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 250;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        thumb_ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = thumb_ellipsoid_offset_z_filtered[j] - thumb_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-thumb_ellipsoid_offset_x_filtered[j], 2)/pow(thumb_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-thumb_ellipsoid_offset_y_filtered[j], 2)/pow(thumb_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 250;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        thumb_ellipsoid_point_cloud.points.push_back( ellipsoid_point );    
      }
    }
  }
  *augmented_cloud += thumb_ellipsoid_point_cloud;
  
  
    
  // index
  std::vector<double> index_ellipsoid_a_filtered, index_ellipsoid_b_filtered, index_ellipsoid_c_filtered, index_ellipsoid_offset_x_filtered, index_ellipsoid_offset_y_filtered, index_ellipsoid_offset_z_filtered;
  for( unsigned int i=0; i<index_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << index_ellipsoid_offset_x[i], index_ellipsoid_offset_y[i], index_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(index_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(index_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(index_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if( largest_ellipsoid_value < ellipsoid_value ){
          largest_ellipsoid_value = ellipsoid_value;
          a_to_save = index_ellipsoid_a[i];
          b_to_save = index_ellipsoid_b[i];
          c_to_save = index_ellipsoid_c[i];
          x_offset_to_save = ellipsoid_offset_vector(0);
          y_offset_to_save = ellipsoid_offset_vector(1);
          z_offset_to_save = ellipsoid_offset_vector(2);
          vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
          vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
          vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      index_ellipsoid_a_filtered.push_back( a_to_save );
      index_ellipsoid_b_filtered.push_back( b_to_save );
      index_ellipsoid_c_filtered.push_back( c_to_save );
      index_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      index_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      index_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered index ellipsoids: " << index_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the index workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB> index_ellipsoid_point_cloud;
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<index_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-index_ellipsoid_a_filtered[j]+index_ellipsoid_offset_x_filtered[j]) + k*2*index_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-index_ellipsoid_b_filtered[j]+index_ellipsoid_offset_y_filtered[j]) + l*2*index_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = index_ellipsoid_offset_z_filtered[j] + index_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-index_ellipsoid_offset_x_filtered[j], 2)/pow(index_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-index_ellipsoid_offset_y_filtered[j], 2)/pow(index_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 250;
        ellipsoid_point.b = 0;
        index_ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = index_ellipsoid_offset_z_filtered[j] - index_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-index_ellipsoid_offset_x_filtered[j], 2)/pow(index_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-index_ellipsoid_offset_y_filtered[j], 2)/pow(index_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 250;
        ellipsoid_point.b = 0;
        index_ellipsoid_point_cloud.points.push_back( ellipsoid_point );    
      }
    }
  }
  *augmented_cloud += index_ellipsoid_point_cloud;
  
  
  
  
  // middle
  std::vector<double> middle_ellipsoid_a_filtered, middle_ellipsoid_b_filtered, middle_ellipsoid_c_filtered, middle_ellipsoid_offset_x_filtered, middle_ellipsoid_offset_y_filtered, middle_ellipsoid_offset_z_filtered;
  for( unsigned int i=0; i<middle_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << middle_ellipsoid_offset_x[i], middle_ellipsoid_offset_y[i], middle_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(middle_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(middle_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(middle_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if( largest_ellipsoid_value < ellipsoid_value ){
          largest_ellipsoid_value = ellipsoid_value;
          a_to_save = middle_ellipsoid_a[i];
          b_to_save = middle_ellipsoid_b[i];
          c_to_save = middle_ellipsoid_c[i];
          x_offset_to_save = ellipsoid_offset_vector(0);
          y_offset_to_save = ellipsoid_offset_vector(1);
          z_offset_to_save = ellipsoid_offset_vector(2);
          vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
          vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
          vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      middle_ellipsoid_a_filtered.push_back( a_to_save );
      middle_ellipsoid_b_filtered.push_back( b_to_save );
      middle_ellipsoid_c_filtered.push_back( c_to_save );
      middle_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      middle_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      middle_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered middle ellipsoids: " << middle_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the middle workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB> middle_ellipsoid_point_cloud;
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<middle_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-middle_ellipsoid_a_filtered[j]+middle_ellipsoid_offset_x_filtered[j]) + k*2*middle_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-middle_ellipsoid_b_filtered[j]+middle_ellipsoid_offset_y_filtered[j]) + l*2*middle_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = middle_ellipsoid_offset_z_filtered[j] + middle_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-middle_ellipsoid_offset_x_filtered[j], 2)/pow(middle_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-middle_ellipsoid_offset_y_filtered[j], 2)/pow(middle_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 250;
        middle_ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = middle_ellipsoid_offset_z_filtered[j] - middle_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-middle_ellipsoid_offset_x_filtered[j], 2)/pow(middle_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-middle_ellipsoid_offset_y_filtered[j], 2)/pow(middle_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 250;
        middle_ellipsoid_point_cloud.points.push_back( ellipsoid_point );    
      }
    }
  }
  *augmented_cloud += middle_ellipsoid_point_cloud;
  
  
  
  
  // pinky
  std::vector<double> pinky_ellipsoid_a_filtered, pinky_ellipsoid_b_filtered, pinky_ellipsoid_c_filtered, pinky_ellipsoid_offset_x_filtered, pinky_ellipsoid_offset_y_filtered, pinky_ellipsoid_offset_z_filtered;
  for( unsigned int i=0; i<pinky_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << pinky_ellipsoid_offset_x[i], pinky_ellipsoid_offset_y[i], pinky_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(pinky_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(pinky_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(pinky_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if( largest_ellipsoid_value < ellipsoid_value ){
          largest_ellipsoid_value = ellipsoid_value;
          a_to_save = pinky_ellipsoid_a[i];
          b_to_save = pinky_ellipsoid_b[i];
          c_to_save = pinky_ellipsoid_c[i];
          x_offset_to_save = ellipsoid_offset_vector(0);
          y_offset_to_save = ellipsoid_offset_vector(1);
          z_offset_to_save = ellipsoid_offset_vector(2);
          vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
          vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
          vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      pinky_ellipsoid_a_filtered.push_back( a_to_save );
      pinky_ellipsoid_b_filtered.push_back( b_to_save );
      pinky_ellipsoid_c_filtered.push_back( c_to_save );
      pinky_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      pinky_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      pinky_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered pinky ellipsoids: " << pinky_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the pinky workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB> pinky_ellipsoid_point_cloud;
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<pinky_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-pinky_ellipsoid_a_filtered[j]+pinky_ellipsoid_offset_x_filtered[j]) + k*2*pinky_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-pinky_ellipsoid_b_filtered[j]+pinky_ellipsoid_offset_y_filtered[j]) + l*2*pinky_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = pinky_ellipsoid_offset_z_filtered[j] + pinky_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-pinky_ellipsoid_offset_x_filtered[j], 2)/pow(pinky_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-pinky_ellipsoid_offset_y_filtered[j], 2)/pow(pinky_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 100;
        ellipsoid_point.g = 100;
        ellipsoid_point.b = 100;
        pinky_ellipsoid_point_cloud.points.push_back( ellipsoid_point );
        
        ellipsoid_z = pinky_ellipsoid_offset_z_filtered[j] - pinky_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-pinky_ellipsoid_offset_x_filtered[j], 2)/pow(pinky_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-pinky_ellipsoid_offset_y_filtered[j], 2)/pow(pinky_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 100;
        ellipsoid_point.g = 100;
        ellipsoid_point.b = 100;
        pinky_ellipsoid_point_cloud.points.push_back( ellipsoid_point );    
      }
    }
  }
  *augmented_cloud += pinky_ellipsoid_point_cloud;
  */
  
  
  
  
  
  
  
  
  
  
  begin = clock();
  ///////////////
  // SECTION 8 //
  ///////////////
  //
  // apply some constraints to decrease the amount of ellipsoids/vertices points to choose from
  // examples of functions needs to be formulated:
  // 1. force balance constraint
  // 2. maximize distance between grasping points
  // 3. main contact points (thumb tip + middle tip + palm center) must be on opposite sides of the object coordinate system locate at centroid
  // in other words, keep main contact points at opposite quadrants regarding the object's grasping axis (either x-axis or y-axis)
  // 4. pick first ellipsoids that are nearer to the neutral hand (tip) configuration (open hand)
  
  // OR : go in reverse, assume a successful closed hand configuration, try to get the object vertices nearest to this configuration
  // by configuration we mean end tips + palm center points positions
  
  // OR : we can formulate optimization problem, with each tip point being the optimization variable over the convex feasible region being each ellipsoid (we iterate over all ellipsoids per finger)
  // formulate optimization function to maximize force balance + maximize gasping volume
  // formulate inequality constraints to take care of distances between fingers
  // IMP: xy area formed by main contact points must be greater than half og object's estimated xy area
  
  // BETTER IMPLEMENT THIS ONE !!!
  // OR : use nominal object location, a location where we would like to have it as the centroid of the graspable portion of the object
  // such location should be near more or less the centroid of the all fingers workspace
  
  
  // make unit vector along hand's x, y, z-axes and center these vectors at the hand workspace centroid
  Eigen::Vector4d hand_centroid_frame_x_axis, hand_centroid_frame_y_axis, hand_centroid_frame_z_axis;
  Eigen::Vector4d hand_centroid_frame_origin, hand_centroid_frame_origin_in_hand_frame;
  Eigen::Vector3d hand_centroid_frame_x_axis_vector, hand_centroid_frame_y_axis_vector, hand_centroid_frame_z_axis_vector;
  Eigen::Vector3d hand_centroid_frame_origin_point;
  
  hand_centroid_frame_origin << 0,0,0,1;
  hand_centroid_frame_x_axis << 1,0,0,1;
  hand_centroid_frame_y_axis << 0,1,0,1;
  hand_centroid_frame_z_axis << 0,0,1,1;
  
  dummy_translation << hand_workspace_centroid_point.x, hand_workspace_centroid_point.y, hand_workspace_centroid_point.z;
  dummy_transform << Eigen::Matrix3d::Identity(), dummy_translation,
                     0,0,0,1;
  dummy_transform = hand_transform*dummy_transform;
  
  hand_centroid_frame_origin = dummy_transform*hand_centroid_frame_origin;
  hand_centroid_frame_x_axis = dummy_transform*hand_centroid_frame_x_axis;
  hand_centroid_frame_y_axis = dummy_transform*hand_centroid_frame_y_axis;
  hand_centroid_frame_z_axis = dummy_transform*hand_centroid_frame_z_axis;
  
  //viewer->addCoordinateSystem(0.1,hand_centroid_frame_origin(0),hand_centroid_frame_origin(1),hand_centroid_frame_origin(2));
  
  pcl::PointXYZ axis_point, frame_origin_point;
  frame_origin_point.x = hand_centroid_frame_origin(0);
  frame_origin_point.y = hand_centroid_frame_origin(1);
  frame_origin_point.z = hand_centroid_frame_origin(2);
  axis_point.x = hand_centroid_frame_x_axis(0);
  axis_point.y = hand_centroid_frame_x_axis(1);
  axis_point.z = hand_centroid_frame_x_axis(2);
  name = "hand centroid x axis";
  viewer->addArrow<pcl::PointXYZ>(axis_point, frame_origin_point, 1.0, 0.0, 0.0, 0, name, 0);
  
  axis_point.x = hand_centroid_frame_y_axis(0);
  axis_point.y = hand_centroid_frame_y_axis(1);
  axis_point.z = hand_centroid_frame_y_axis(2);
  name = "hand centroid y axis";
  viewer->addArrow<pcl::PointXYZ>(axis_point, frame_origin_point, 0.0, 1.0, 0.0, 0, name, 0);
  
  axis_point.x = hand_centroid_frame_z_axis(0);
  axis_point.y = hand_centroid_frame_z_axis(1);
  axis_point.z = hand_centroid_frame_z_axis(2);
  name = "hand centroid z axis";
  viewer->addArrow<pcl::PointXYZ>(axis_point, frame_origin_point, 0.0, 0.0, 1.0, 0, name, 0);
  
  // hand workspace frame axes vectors
  hand_centroid_frame_x_axis_vector << hand_centroid_frame_x_axis(0)-hand_centroid_frame_origin(0), hand_centroid_frame_x_axis(1)-hand_centroid_frame_origin(1), hand_centroid_frame_x_axis(2)-hand_centroid_frame_origin(2);
  hand_centroid_frame_y_axis_vector << hand_centroid_frame_y_axis(0)-hand_centroid_frame_origin(0), hand_centroid_frame_y_axis(1)-hand_centroid_frame_origin(1), hand_centroid_frame_y_axis(2)-hand_centroid_frame_origin(2);
  hand_centroid_frame_z_axis_vector << hand_centroid_frame_z_axis(0)-hand_centroid_frame_origin(0), hand_centroid_frame_z_axis(1)-hand_centroid_frame_origin(1), hand_centroid_frame_z_axis(2)-hand_centroid_frame_origin(2);
  
  
  /*
  // project the distance vector onto the hand x-axis vector
  Eigen::Vector3d hand_x_axis_vector_3d;
  hand_x_axis_vector_3d << hand_x_axis_vector(0)-origin_point(0), hand_x_axis_vector(1)-origin_point(1), hand_x_axis_vector(2)-origin_point(2);
  dummy_translation << dummy_vector.dot(hand_x_axis_vector_3d),0,0;
  */
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to make hand workspace centroid frame = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  
  
  
  begin = clock();
  ///////////////
  // SECTION 9 //
  ///////////////
  // AGAIN filter the ellipsoids that are merging with object vertices using the workspace centroid frame
  // thumb
  Eigen::Vector4d ellipsoid_offset_vector;
  Eigen::Vector3d ellipsoid_offset_vector_3d;
  std::vector<double> thumb_ellipsoid_a_filtered, thumb_ellipsoid_b_filtered, thumb_ellipsoid_c_filtered, thumb_ellipsoid_offset_x_filtered, thumb_ellipsoid_offset_y_filtered, thumb_ellipsoid_offset_z_filtered;
  std::vector<double> vertex_point_filtered_x, vertex_point_filtered_y, vertex_point_filtered_z;
  double ellipsoid_value, largest_ellipsoid_value=0.0;
  double a_to_save, b_to_save, c_to_save, x_offset_to_save, y_offset_to_save, z_offset_to_save;
  Eigen::Vector3d vertex_point_to_save; 
  
  
  for( unsigned int i=0; i<thumb_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << thumb_ellipsoid_offset_x[i], thumb_ellipsoid_offset_y[i], thumb_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    ellipsoid_offset_vector_3d << ellipsoid_offset_vector(0)-hand_centroid_frame_origin(0), ellipsoid_offset_vector(1)-hand_centroid_frame_origin(1), ellipsoid_offset_vector(2)-hand_centroid_frame_origin(2);
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(thumb_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(thumb_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(thumb_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if(ellipsoid_offset_vector_3d.dot(hand_centroid_frame_z_axis_vector) < 0){   // select only thumb workspace ellipsoids on the negative z-axis
          if( largest_ellipsoid_value < ellipsoid_value ){
            largest_ellipsoid_value = ellipsoid_value;
            a_to_save = thumb_ellipsoid_a[i];
            b_to_save = thumb_ellipsoid_b[i];
            c_to_save = thumb_ellipsoid_c[i];
            x_offset_to_save = ellipsoid_offset_vector(0);
            y_offset_to_save = ellipsoid_offset_vector(1);
            z_offset_to_save = ellipsoid_offset_vector(2);
            vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
            vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
            vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
          }
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      thumb_ellipsoid_a_filtered.push_back( a_to_save );
      thumb_ellipsoid_b_filtered.push_back( b_to_save );
      thumb_ellipsoid_c_filtered.push_back( c_to_save );
      thumb_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      thumb_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      thumb_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered thumb ellipsoids: " << thumb_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the thumb workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr thumb_ellipsoid_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<thumb_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-thumb_ellipsoid_a_filtered[j]+thumb_ellipsoid_offset_x_filtered[j]) + k*2*thumb_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-thumb_ellipsoid_b_filtered[j]+thumb_ellipsoid_offset_y_filtered[j]) + l*2*thumb_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = thumb_ellipsoid_offset_z_filtered[j] + thumb_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-thumb_ellipsoid_offset_x_filtered[j], 2)/pow(thumb_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-thumb_ellipsoid_offset_y_filtered[j], 2)/pow(thumb_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 255;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        thumb_ellipsoid_point_cloud->points.push_back( ellipsoid_point );
        
        ellipsoid_z = thumb_ellipsoid_offset_z_filtered[j] - thumb_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-thumb_ellipsoid_offset_x_filtered[j], 2)/pow(thumb_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-thumb_ellipsoid_offset_y_filtered[j], 2)/pow(thumb_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 255;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 0;
        thumb_ellipsoid_point_cloud->points.push_back( ellipsoid_point );
      }
    }
  }
  //*augmented_cloud += *thumb_ellipsoid_point_cloud;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(thumb_ellipsoid_point_cloud, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(thumb_ellipsoid_point_cloud, red_color, "red cloud");
  
  
  // index
  std::vector<double> index_ellipsoid_a_filtered, index_ellipsoid_b_filtered, index_ellipsoid_c_filtered, index_ellipsoid_offset_x_filtered, index_ellipsoid_offset_y_filtered, index_ellipsoid_offset_z_filtered;
  for( unsigned int i=0; i<index_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << index_ellipsoid_offset_x[i], index_ellipsoid_offset_y[i], index_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    ellipsoid_offset_vector_3d << ellipsoid_offset_vector(0)-hand_centroid_frame_origin(0), ellipsoid_offset_vector(1)-hand_centroid_frame_origin(1), ellipsoid_offset_vector(2)-hand_centroid_frame_origin(2);
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(index_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(index_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(index_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if(ellipsoid_offset_vector_3d.dot(hand_centroid_frame_z_axis_vector) > 0){   // select only index workspace ellipsoids on the positive z-axis
          if( largest_ellipsoid_value < ellipsoid_value ){
            largest_ellipsoid_value = ellipsoid_value;
            a_to_save = index_ellipsoid_a[i];
            b_to_save = index_ellipsoid_b[i];
            c_to_save = index_ellipsoid_c[i];
            x_offset_to_save = ellipsoid_offset_vector(0);
            y_offset_to_save = ellipsoid_offset_vector(1);
            z_offset_to_save = ellipsoid_offset_vector(2);
            vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
            vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
            vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
          }
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      index_ellipsoid_a_filtered.push_back( a_to_save );
      index_ellipsoid_b_filtered.push_back( b_to_save );
      index_ellipsoid_c_filtered.push_back( c_to_save );
      index_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      index_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      index_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered index ellipsoids: " << index_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the index workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr index_ellipsoid_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<index_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-index_ellipsoid_a_filtered[j]+index_ellipsoid_offset_x_filtered[j]) + k*2*index_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-index_ellipsoid_b_filtered[j]+index_ellipsoid_offset_y_filtered[j]) + l*2*index_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = index_ellipsoid_offset_z_filtered[j] + index_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-index_ellipsoid_offset_x_filtered[j], 2)/pow(index_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-index_ellipsoid_offset_y_filtered[j], 2)/pow(index_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 255;
        ellipsoid_point.b = 0;
        index_ellipsoid_point_cloud->points.push_back( ellipsoid_point );
        
        ellipsoid_z = index_ellipsoid_offset_z_filtered[j] - index_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-index_ellipsoid_offset_x_filtered[j], 2)/pow(index_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-index_ellipsoid_offset_y_filtered[j], 2)/pow(index_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 255;
        ellipsoid_point.b = 0;
        index_ellipsoid_point_cloud->points.push_back( ellipsoid_point );    
      }
    }
  }
  //*augmented_cloud += *index_ellipsoid_point_cloud;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(index_ellipsoid_point_cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB>(index_ellipsoid_point_cloud, green_color, "green cloud");
  
  
  
  // middle
  std::vector<double> middle_ellipsoid_a_filtered, middle_ellipsoid_b_filtered, middle_ellipsoid_c_filtered, middle_ellipsoid_offset_x_filtered, middle_ellipsoid_offset_y_filtered, middle_ellipsoid_offset_z_filtered;
  for( unsigned int i=0; i<middle_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << middle_ellipsoid_offset_x[i], middle_ellipsoid_offset_y[i], middle_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    ellipsoid_offset_vector_3d << ellipsoid_offset_vector(0)-hand_centroid_frame_origin(0), ellipsoid_offset_vector(1)-hand_centroid_frame_origin(1), ellipsoid_offset_vector(2)-hand_centroid_frame_origin(2);
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(middle_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(middle_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(middle_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if(ellipsoid_offset_vector_3d.dot(hand_centroid_frame_z_axis_vector) > 0){   // select only middle workspace ellipsoids on the positive z-axis
          if( largest_ellipsoid_value < ellipsoid_value ){
            largest_ellipsoid_value = ellipsoid_value;
            a_to_save = middle_ellipsoid_a[i];
            b_to_save = middle_ellipsoid_b[i];
            c_to_save = middle_ellipsoid_c[i];
            x_offset_to_save = ellipsoid_offset_vector(0);
            y_offset_to_save = ellipsoid_offset_vector(1);
            z_offset_to_save = ellipsoid_offset_vector(2);
            vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
            vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
            vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
          }
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      middle_ellipsoid_a_filtered.push_back( a_to_save );
      middle_ellipsoid_b_filtered.push_back( b_to_save );
      middle_ellipsoid_c_filtered.push_back( c_to_save );
      middle_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      middle_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      middle_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered middle ellipsoids: " << middle_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the middle workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_ellipsoid_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<middle_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-middle_ellipsoid_a_filtered[j]+middle_ellipsoid_offset_x_filtered[j]) + k*2*middle_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-middle_ellipsoid_b_filtered[j]+middle_ellipsoid_offset_y_filtered[j]) + l*2*middle_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = middle_ellipsoid_offset_z_filtered[j] + middle_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-middle_ellipsoid_offset_x_filtered[j], 2)/pow(middle_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-middle_ellipsoid_offset_y_filtered[j], 2)/pow(middle_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 255;
        middle_ellipsoid_point_cloud->points.push_back( ellipsoid_point );
        
        ellipsoid_z = middle_ellipsoid_offset_z_filtered[j] - middle_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-middle_ellipsoid_offset_x_filtered[j], 2)/pow(middle_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-middle_ellipsoid_offset_y_filtered[j], 2)/pow(middle_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 0;
        ellipsoid_point.g = 0;
        ellipsoid_point.b = 255;
        middle_ellipsoid_point_cloud->points.push_back( ellipsoid_point );    
      }
    }
  }
  //*augmented_cloud += *middle_ellipsoid_point_cloud;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(middle_ellipsoid_point_cloud, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZRGB>(middle_ellipsoid_point_cloud, blue_color, "blue cloud");
  
  
  
  // pinky
  std::vector<double> pinky_ellipsoid_a_filtered, pinky_ellipsoid_b_filtered, pinky_ellipsoid_c_filtered, pinky_ellipsoid_offset_x_filtered, pinky_ellipsoid_offset_y_filtered, pinky_ellipsoid_offset_z_filtered;
  for( unsigned int i=0; i<pinky_ellipsoid_a.size(); i++ ){
    ellipsoid_offset_vector << pinky_ellipsoid_offset_x[i], pinky_ellipsoid_offset_y[i], pinky_ellipsoid_offset_z[i], 1;
    ellipsoid_offset_vector = hand_transform*ellipsoid_offset_vector;   // transforming the ellipsoid offset point
    ellipsoid_offset_vector_3d << ellipsoid_offset_vector(0)-hand_centroid_frame_origin(0), ellipsoid_offset_vector(1)-hand_centroid_frame_origin(1), ellipsoid_offset_vector(2)-hand_centroid_frame_origin(2);
    a_to_save=0.0;
    largest_ellipsoid_value=0.0;
    for( unsigned int j=0; j<object_mesh_vertices.size(); j++ ){
      ellipsoid_value =  pow(object_mesh_vertices.points[j].x - ellipsoid_offset_vector(0), 2)/pow(pinky_ellipsoid_a[i], 2) 
                       + pow(object_mesh_vertices.points[j].y - ellipsoid_offset_vector(1), 2)/pow(pinky_ellipsoid_b[i], 2) 
                       + pow(object_mesh_vertices.points[j].z - ellipsoid_offset_vector(2), 2)/pow(pinky_ellipsoid_c[i], 2);
      
      // compute the value of ellipsoid using the vertex point
      // save ellipsoids that contain vertex points
      if( ellipsoid_value < 1.0 ){
        if(ellipsoid_offset_vector_3d.dot(hand_centroid_frame_z_axis_vector) > 0){   // select only pinky workspace ellipsoids on the positive z-axis
          if( largest_ellipsoid_value < ellipsoid_value ){
            largest_ellipsoid_value = ellipsoid_value;
            a_to_save = pinky_ellipsoid_a[i];
            b_to_save = pinky_ellipsoid_b[i];
            c_to_save = pinky_ellipsoid_c[i];
            x_offset_to_save = ellipsoid_offset_vector(0);
            y_offset_to_save = ellipsoid_offset_vector(1);
            z_offset_to_save = ellipsoid_offset_vector(2);
            vertex_point_to_save(0) = object_mesh_vertices.points[j].x;
            vertex_point_to_save(1) = object_mesh_vertices.points[j].y;
            vertex_point_to_save(2) = object_mesh_vertices.points[j].z;
          }
        }
      }
    }
    
    if( a_to_save!=0.0 ){
      // save the ellipsoid if we found one!
      pinky_ellipsoid_a_filtered.push_back( a_to_save );
      pinky_ellipsoid_b_filtered.push_back( b_to_save );
      pinky_ellipsoid_c_filtered.push_back( c_to_save );
      pinky_ellipsoid_offset_x_filtered.push_back( x_offset_to_save );
      pinky_ellipsoid_offset_y_filtered.push_back( y_offset_to_save );
      pinky_ellipsoid_offset_z_filtered.push_back( z_offset_to_save );
      
      // save the vertex point
      vertex_point_filtered_x.push_back( vertex_point_to_save(0) );
      vertex_point_filtered_y.push_back( vertex_point_to_save(1) );
      vertex_point_filtered_z.push_back( vertex_point_to_save(2) );
    }
    
  }
  std::cout << "filtered pinky ellipsoids: " << pinky_ellipsoid_a_filtered.size() << std::endl;
  
  // draw the pinky workspace ellipsoids [filtered]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pinky_ellipsoid_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ellipsoid_point_cloud_samples = 30;
  for(unsigned int j=0; j<pinky_ellipsoid_a_filtered.size(); j++){
    for(unsigned int k=0; k<ellipsoid_point_cloud_samples; k++){
      ellipsoid_x = (-pinky_ellipsoid_a_filtered[j]+pinky_ellipsoid_offset_x_filtered[j]) + k*2*pinky_ellipsoid_a_filtered[j]/ellipsoid_point_cloud_samples;
      for(unsigned int l=0; l<ellipsoid_point_cloud_samples; l++){
        ellipsoid_y = (-pinky_ellipsoid_b_filtered[j]+pinky_ellipsoid_offset_y_filtered[j]) + l*2*pinky_ellipsoid_b_filtered[j]/ellipsoid_point_cloud_samples;
        ellipsoid_z = pinky_ellipsoid_offset_z_filtered[j] + pinky_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-pinky_ellipsoid_offset_x_filtered[j], 2)/pow(pinky_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-pinky_ellipsoid_offset_y_filtered[j], 2)/pow(pinky_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 100;
        ellipsoid_point.g = 100;
        ellipsoid_point.b = 100;
        pinky_ellipsoid_point_cloud->points.push_back( ellipsoid_point );
        
        ellipsoid_z = pinky_ellipsoid_offset_z_filtered[j] - pinky_ellipsoid_c_filtered[j]*sqrt( 1 - pow(ellipsoid_x-pinky_ellipsoid_offset_x_filtered[j], 2)/pow(pinky_ellipsoid_a_filtered[j], 2) - pow(ellipsoid_y-pinky_ellipsoid_offset_y_filtered[j], 2)/pow(pinky_ellipsoid_b_filtered[j], 2) );
        ellipsoid_point.x = ellipsoid_x;
        ellipsoid_point.y = ellipsoid_y;
        ellipsoid_point.z = ellipsoid_z;
        ellipsoid_point.r = 100;
        ellipsoid_point.g = 100;
        ellipsoid_point.b = 100;
        pinky_ellipsoid_point_cloud->points.push_back( ellipsoid_point );    
      }
    }
  }
  //*augmented_cloud += *pinky_ellipsoid_point_cloud;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> grey_color(pinky_ellipsoid_point_cloud, 100, 100, 100);
  viewer->addPointCloud<pcl::PointXYZRGB>(pinky_ellipsoid_point_cloud, grey_color, "grey cloud");
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "time spent to filter hand workspace ellipsoids = " << time_spent << std::endl << std::endl;
  
  
  
  
  
  
  /*
  
  // optimization
  // optimization example using QuadProg++
  
	MatrixXd A(12,2);
	VectorXd B(12);
	
	qp::Matrix<double> G2, CE, CI;
  qp::Vector<double> g0, ce0, ci0, x2;
	int n, m, p;
	double cost = 0.0;
	char ch;
  
  
	
	
	
	clock_t begin, end;
	double time_spent;
  
  
  
  // A*x + B >= 0
	/////////////////////
	
	A(0,0) = pow( -1 , quad_max(0)+1 )*( -tan( beta_ref_max(0) + PI/2 ) );
	A(0,1) = pow( -1 , quad_max(0)+1 )*1;
	A(1,0) = pow( -1 , quad_min(0) )*( -tan( beta_ref_min(0) + PI/2 ) );
	A(1,1) = pow( -1 , quad_min(0) )*1;
	
	A(2,0) = pow( -1 , quad_max(1)+1 )*( -tan( beta_ref_max(1) + PI/2 ) );
	A(2,1) = pow( -1 , quad_max(1)+1 )*1;
	A(3,0) = pow( -1 , quad_min(1) )*( -tan( beta_ref_min(1) + PI/2 ) );
	A(3,1) = pow( -1 , quad_min(1) )*1;
	
	A(4,0) = pow( -1 , quad_max(2)+1 )*( -tan( beta_ref_max(2) + PI/2 ) );
	A(4,1) = pow( -1 , quad_max(2)+1 )*1;
	A(5,0) = pow( -1 , quad_min(2) )*( -tan( beta_ref_min(2) + PI/2 ) );
	A(5,1) = pow( -1 , quad_min(2) )*1;
	
	A(6,0) = pow( -1 , quad_max(3)+1 )*( -tan( beta_ref_max(3) + PI/2 ) );
	A(6,1) = pow( -1 , quad_max(3)+1 )*1;
	A(7,0) = pow( -1 , quad_min(3) )*( -tan( beta_ref_min(3) + PI/2 ) );
	A(7,1) = pow( -1 , quad_min(3) )*1;
	
	
	A(8,0) = -1;
	A(8,1) = 0;
	A(9,0) = 1;
	A(9,1) = 0;
	
	A(10,0) = 0;
	A(10,1) = -1;
	A(11,0) = 0;
	A(11,1) = 1;
	
	
	
	B(0) = pow( -1 , quad_max(0)+1 )*( - hy1 + hx1*tan( beta_ref_max(0) + PI/2 ) );
	B(1) = pow( -1 , quad_min(0) )*( - hy1 + hx1*tan( beta_ref_min(0) + PI/2 ) );
	
	B(2) = pow( -1 , quad_max(1)+1 )*( - hy2 + hx2*tan( beta_ref_max(1) + PI/2 ) );
	B(3) = pow( -1 , quad_min(1) )*( - hy2 + hx2*tan( beta_ref_min(1) + PI/2 ) );
	
	B(4) = pow( -1 , quad_max(2)+1 )*( - hy3 + hx3*tan( beta_ref_max(2) + PI/2 ) );
	B(5) = pow( -1 , quad_min(2) )*( - hy3 + hx3*tan( beta_ref_min(2) + PI/2 ) );
	
	B(6) = pow( -1 , quad_max(3)+1 )*( - hy4 + hx4*tan( beta_ref_max(3) + PI/2 ) );
	B(7) = pow( -1 , quad_min(3) )*( - hy4 + hx4*tan( beta_ref_min(3) + PI/2 ) );
	
	
	B(8) = ICR_past_step_opt(0) + ICR_dot_max(0)*SAMPLE_TIME;
	B(9) = -ICR_past_step_opt(0) + ICR_dot_max(0)*SAMPLE_TIME;
	
	B(10) = ICR_past_step_opt(1) + ICR_dot_max(1)*SAMPLE_TIME;
	B(11) = -ICR_past_step_opt(1) + ICR_dot_max(1)*SAMPLE_TIME;
	
	
	cout << "A = " << endl << A << endl << endl;
	cout << "B = " << endl << B << endl << endl;
	
	
	
	
	begin = clock();
	
	
	// OPTIMIZATION
	n = 2;
	G2.resize(n, n);
	{
		std::istringstream is("2, 0,"
													"0, 2 ");

		for (int i = 0; i < n; i++)	
			for (int j = 0; j < n; j++)
				is >> G2[i][j] >> ch;
	}
	std::cout << G2 << endl;

	g0.resize(n);
	{
		for (int i = 0; i < n; i++)
		{	
			g0[i] = -2*ICR_ref(i);
			//g0[i] = -2*ICR_des(i);
		}
	}
	std::cout << g0 << endl;
	
	
	m = 0;
	CE.resize(n, m);
	{
		std::istringstream is("0.0, "
													"0.0 ");

		for (int i = 0; i < n; i++)
			for (int j = 0; j < m; j++)
				is >> CE[i][j] >> ch;
	}
	//std::cout << CE << endl;

	ce0.resize(m);
	{
		std::istringstream is("0.0 ");

		for (int j = 0; j < m; j++)
			is >> ce0[j] >> ch;
	}
	//std::cout << ce0 << endl;
	
	
	
	
	p = 12;
	CI.resize(n, p);
	{
		for (int i = 0; i < n; i++)
			for (int j = 0; j < p; j++)
				CI[i][j] = A(j,i);
	}
	//std::cout << CI << endl;

	ci0.resize(p);
	{
		for (int j = 0; j < p; j++)
			ci0[j] = B(j);
	}
	//std::cout << ci0 << endl;
	
	
	
	x2.resize(n);

	std::cout << "f: " << solve_quadprog(G2, g0, CE, ce0, CI, ci0, x2) << std::endl;
	std::cout << "x2: " << x2 << std::endl;
	
	cout << "x2 = " << x2[0] << " , " << x2[1] << endl << endl;
	
	
	{
		std::istringstream is("1, 0,"
													"0, 1 ");

		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				is >> G2[i][j] >> ch;
	}

	std::cout << "Double checking cost: ";
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			cost += x2[i] * G2[i][j] * x2[j];
	cost *= 0.5;	

	for (int i = 0; i < n; i++)
		cost += g0[i] * x2[i];
	std::cout << cost << std::endl;
	
	
	
	end = clock();
	
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	
	cout << "time spent by optimization algorithm = " << time_spent << endl << endl;
  
  */
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  // add to visualized point cloud
  *augmented_cloud += *hand_transformed_cloud;
  //*augmented_cloud += *roi_transformed_cloud;
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
	
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce();
    //triangulation_viewer->spinOnce();
  } 
  return 0;
}

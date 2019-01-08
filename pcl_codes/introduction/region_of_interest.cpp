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


int main(){
  //std::string object_name = "cocacola_bottle";
  std::string object_name = "object_with_major_axis_in_x";
  //std::string object_name = "cocacola_bottle_in_x";
  //std::string object_name = "cocacola_bottle_in_y";
  //std::string object_name = "cocacola_bottle_in_xyz";
  
  
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
  
  std::cout << "1... " << std::endl;
  // load allegro hand workspace as ellipsoids
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr allegro_hand_workspace_ellipsoids_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("allegro_hand_workspace_as_ellipsoids.pcd", *allegro_hand_workspace_ellipsoids_xyzrgb);
  //Hullviewer->addPointCloud<pcl::PointXYZRGB>(allegro_hand_workspace_ellipsoids_xyzrgb, rgb,"hull viewer");
  
  std::cout << "2... " << std::endl;
  
  
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
  std::cout << "3... " << std::endl;
  
  // getting the vertices of the polygon mesh loaded
  pcl::PointCloud<pcl::PointXYZ> object_mesh_vertices;
  std::cout << "4... " << std::endl;
  pcl::fromPCLPointCloud2(object_mesh.cloud, object_mesh_vertices);
  
  std::cout << "number of vertices: " << object_mesh_vertices.size() << std::endl;
  Hullviewer->addPolygonMesh(object_mesh,"hull");
  
  
  
  // get object centroid location
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for(unsigned int i=0;i<object_mesh_vertices.points.size();i++)
    centroid.add( object_mesh_vertices.points[i] );
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  std::cout<< centroid_point << std::endl;  
  viewer->addCoordinateSystem(0.05,centroid_point.x, centroid_point.y, centroid_point.z);
  Hullviewer->addCoordinateSystem(0.1,centroid_point.x, centroid_point.y, centroid_point.z);
  
  
  
  // object main axis estimation
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
  // draw the approximate axis line
  viewer->addLine(far_point_in_pos_direction, far_point_in_neg_direction, "approximate major axis (object's longest dimension)", 0); 
  
  
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
    if( distance_to_far_point_pos > distance_to_far_point_neg )
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
  std::cout<< centroid_far_pos_point << std::endl; 
  
  for(unsigned int i=0;i<cloud_far_neg.points.size();i++)
    centroid_far_neg.add( cloud_far_neg.points[i] );
  centroid_far_neg.get(centroid_far_neg_point);
  std::cout<< centroid_far_neg_point << std::endl;  
  
  viewer->addCoordinateSystem(0.05,centroid_far_pos_point.x, centroid_far_pos_point.y, centroid_far_pos_point.z);
  viewer->addCoordinateSystem(0.05,centroid_far_neg_point.x, centroid_far_neg_point.y, centroid_far_neg_point.z);
  Hullviewer->addCoordinateSystem(0.1,centroid_far_pos_point.x, centroid_far_pos_point.y, centroid_far_pos_point.z);
  Hullviewer->addCoordinateSystem(0.1,centroid_far_neg_point.x, centroid_far_neg_point.y, centroid_far_neg_point.z);
  
  
  
  
  // to be continued !
  // computing object major axis orientation
  // major axis vector
  /*
  Eigen::Vector3d major_axis_vector, projection;
  double roll, pitch, yaw;
  major_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  major_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  major_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  projection = major_axis_vector;
  projection[2] = 0.0;
  yaw = acos( major_axis_vector.dot(projection)/(major_axis_vector.array().abs()*projection.array().abs()) );
  std::cout << major_axis_vector.dot(projection) << std::endl;
  std::cout << (major_axis_vector.array().abs()*projection.array().abs()) << std::endl;
  */
  
  double alpha;   // angle about (+ve) x-axis
  double beta;    // angle about (+ve) y-axis
  double gamma;   // angle about (+ve) z-axis
  double vector_magnitude;
  Eigen::Vector3d major_axis_vector;
  major_axis_vector[0] = centroid_far_pos_point.x - centroid_far_neg_point.x;
  major_axis_vector[1] = centroid_far_pos_point.y - centroid_far_neg_point.y;
  major_axis_vector[2] = centroid_far_pos_point.z - centroid_far_neg_point.z;
  vector_magnitude = sqrt( pow(major_axis_vector[0],2) + pow(major_axis_vector[1],2) + pow(major_axis_vector[2],2) );
  alpha = acos( major_axis_vector[0]/vector_magnitude );
  beta = acos( major_axis_vector[1]/vector_magnitude );
  gamma = acos( major_axis_vector[2]/vector_magnitude );
  std::cout << "alpha = " << alpha << std::endl;
  std::cout << "beta  = " << beta << std::endl;
  std::cout << "gamma = " << gamma << std::endl;
  
  
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
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  // orienting the hand along the object's major axis
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0, 0;
  transform.rotate( Eigen::AngleAxisf( alpha, Eigen::Vector3f::UnitX() ) );
  transform.rotate( Eigen::AngleAxisf( beta , Eigen::Vector3f::UnitY() ) );
  transform.rotate( Eigen::AngleAxisf( gamma, Eigen::Vector3f::UnitZ() ) );
  
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::transformPointCloud(*allegro_hand_workspace_ellipsoids_xyzrgb, *hand_transformed_cloud, transform);
  
  
  // view the hand workspace
  *augmented_cloud += *hand_transformed_cloud;
  viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  //Hullviewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"object cloud");
  
  
	
  while ( !viewer->wasStopped() ){
    viewer->spinOnce();
    Hullviewer->spinOnce();
    //triangulation_viewer->spinOnce();
  } 
  return 0;
}

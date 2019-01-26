#include <iostream>
#include <string>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>

#include "../../ros_packages/include/Eigen/Dense"
#include "../../ros_packages/include/useful_implementations.h"
#include "../../ros_packages/include/AllegroRightHandModel.h"
#include "../../ros_packages/include/allegro_hand_parameters.h"

int main(int argc, char **argv){
  
  int number_of_sample_points = std::stoi( argv[1] );
  std::string role = argv[2];
  
  // saving the finger workspace as a point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height   = 1;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("allegro hand sampled workspace"));
  // to handle RGB
  viewer->setBackgroundColor(255, 255, 255);
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  /*
  // make coordinate system and orient it
  // we will need this later!
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0, 0;
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(45.0), Eigen::Vector3f::UnitX() ) );
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(45.0), Eigen::Vector3f::UnitY() ) );
  transform.rotate( Eigen::AngleAxisf( pcl::deg2rad(45.0), Eigen::Vector3f::UnitX() ) );
  viewer->addCoordinateSystem(0.1, transform, "view", 0);
  */
  viewer->addCoordinateSystem(0.1);
  
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  
  
  
  // loading allegro hand model point cloud
  pcl::PointCloud<pcl::PointXYZ> allegro_hand_cloud_xyz;
  pcl::PointCloud<pcl::PointXYZRGB> allegro_hand_cloud_xyzrgb;
  pcl::io::loadPCDFile<pcl::PointXYZ>("allegro_right_hand_model_cloud.pcd", allegro_hand_cloud_xyz);
  
  // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
  copyPointCloud(allegro_hand_cloud_xyz, allegro_hand_cloud_xyzrgb);
  
  cloud = allegro_hand_cloud_xyzrgb;
  *augmented_cloud = allegro_hand_cloud_xyzrgb;
  
  viewer->addPointCloud(augmented_cloud,"allegro hand sampled workspace");
  viewer->setCameraPosition(0.569223, 0.312599 , 0.486299, -0.0307768, -0.017401, 0.0562987, -0.38203, -0.231229, 0.894755, 0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "allegro hand sampled workspace");
	
	
	
	std::vector<std::string> finger_list;
  finger_list.push_back("thumb");
  finger_list.push_back("index");
  finger_list.push_back("middle");
  finger_list.push_back("pinky");
	
	// safe maximum and minimum joint values
  Eigen::Vector4d thumb_joint_safe_min, index_joint_safe_min, middle_joint_safe_min, pinky_joint_safe_min;
  Eigen::Vector4d thumb_joint_safe_max, index_joint_safe_max, middle_joint_safe_max, pinky_joint_safe_max;
  double joint_margin;
	if(role == "convex_shape_generation")
	  joint_margin = 0.4;
	else
    joint_margin = 0.1;
  
  thumb_joint_safe_min  << thumb_joint_0_min+joint_margin,  thumb_joint_1_min+joint_margin,  thumb_joint_2_min+joint_margin,  thumb_joint_3_min+joint_margin;
  index_joint_safe_min  << index_joint_0_min+joint_margin,  index_joint_1_min+joint_margin,  index_joint_2_min+joint_margin,  index_joint_3_min+joint_margin;
  middle_joint_safe_min << middle_joint_0_min+joint_margin, middle_joint_1_min+joint_margin, middle_joint_2_min+joint_margin, middle_joint_3_min+joint_margin;
  pinky_joint_safe_min  << pinky_joint_0_min+joint_margin,  pinky_joint_1_min+joint_margin,  pinky_joint_2_min+joint_margin,  pinky_joint_3_min+joint_margin;

  thumb_joint_safe_max  << thumb_joint_0_max-joint_margin,  thumb_joint_1_max-joint_margin,  thumb_joint_2_max-joint_margin,  thumb_joint_3_max-joint_margin;
  index_joint_safe_max  << index_joint_0_max-joint_margin,  index_joint_1_max-joint_margin,  index_joint_2_max-joint_margin,  index_joint_3_max-joint_margin;
  middle_joint_safe_max << middle_joint_0_max-joint_margin, middle_joint_1_max-joint_margin, middle_joint_2_max-joint_margin, middle_joint_3_max-joint_margin;
  pinky_joint_safe_max  << pinky_joint_0_max-joint_margin,  pinky_joint_1_max-joint_margin,  pinky_joint_2_max-joint_margin,  pinky_joint_3_max-joint_margin;
	  
	
	// safe joint range (stroke)
	Eigen::Vector4d thumb_joint_safe_range(thumb_joint_safe_max-thumb_joint_safe_min);
	Eigen::Vector4d index_joint_safe_range(index_joint_safe_max-index_joint_safe_min);
	Eigen::Vector4d middle_joint_safe_range(middle_joint_safe_max-middle_joint_safe_min);
	Eigen::Vector4d pinky_joint_safe_range(pinky_joint_safe_max-pinky_joint_safe_min);
	
	
	Eigen::Matrix4d DGM;
  Eigen::VectorXd pose_rpy(6);
  Eigen::Vector4d joint_position_desired;
	
	pcl::PointXYZRGB point;
	int i =0;
  
  pcl::PointCloud<pcl::PointXYZRGB> thumb_workspace_cloud, index_workspace_cloud, middle_workspace_cloud, pinky_workspace_cloud;
  thumb_workspace_cloud.height = 1;
  index_workspace_cloud.height = 1;
  middle_workspace_cloud.height = 1;
  pinky_workspace_cloud.height = 1;
  
	for(int finger_index=0; finger_index<finger_list.size(); finger_index++){
	  for(int loop1=0;loop1<number_of_sample_points;loop1++){
	    for(int loop2=0;loop2<number_of_sample_points;loop2++){
	      for(int loop3=0;loop3<number_of_sample_points;loop3++){
	        for(int loop4=0;loop4<number_of_sample_points;loop4++){
	          if(finger_list[finger_index]=="thumb"){
          	  joint_position_desired << (thumb_joint_safe_min(0)+thumb_joint_safe_range(0)*loop1/number_of_sample_points),
          	                            (thumb_joint_safe_min(1)+thumb_joint_safe_range(1)*loop2/number_of_sample_points),
          	                            (thumb_joint_safe_min(2)+thumb_joint_safe_range(2)*loop3/number_of_sample_points),
          	                            (thumb_joint_safe_min(3)+thumb_joint_safe_range(3)*loop4/number_of_sample_points);
          	  DGM  = finger_direct_geometric_model("thumb",joint_position_desired);
          	  pose_rpy = transformation_matrix_to_pose_rpy(DGM);
          	  point.x = pose_rpy(0);  point.y = pose_rpy(1);  point.z = pose_rpy(2);
          	  point.r = 255;  point.g = 0;  point.b = 0;
          	  thumb_workspace_cloud.points.push_back(point);
          	  *augmented_cloud = thumb_workspace_cloud;
          	  *augmented_cloud += allegro_hand_cloud_xyzrgb;
        	  }
        	  else if(finger_list[finger_index]=="index"){
          	  joint_position_desired << (index_joint_safe_min(0)+index_joint_safe_range(0)*loop1/number_of_sample_points),
          	                            (index_joint_safe_min(1)+index_joint_safe_range(1)*loop2/number_of_sample_points),
          	                            (index_joint_safe_min(2)+index_joint_safe_range(2)*loop3/number_of_sample_points),
          	                            (index_joint_safe_min(3)+index_joint_safe_range(3)*loop4/number_of_sample_points);
          	  DGM  = finger_direct_geometric_model("index",joint_position_desired);
          	  pose_rpy = transformation_matrix_to_pose_rpy(DGM);
          	  point.x = pose_rpy(0);  point.y = pose_rpy(1);  point.z = pose_rpy(2);
          	  point.r = 0;  point.g = 255;  point.b = 0;
          	  index_workspace_cloud.points.push_back(point);
          	  *augmented_cloud = index_workspace_cloud;
          	  *augmented_cloud += allegro_hand_cloud_xyzrgb;
          	  *augmented_cloud += thumb_workspace_cloud;
        	  }
        	  else if(finger_list[finger_index]=="middle"){
          	  joint_position_desired << (middle_joint_safe_min(0)+middle_joint_safe_range(0)*loop1/number_of_sample_points),
          	                            (middle_joint_safe_min(1)+middle_joint_safe_range(1)*loop2/number_of_sample_points),
          	                            (middle_joint_safe_min(2)+middle_joint_safe_range(2)*loop3/number_of_sample_points),
          	                            (middle_joint_safe_min(3)+middle_joint_safe_range(3)*loop4/number_of_sample_points);
          	  DGM  = finger_direct_geometric_model("middle",joint_position_desired);
          	  pose_rpy = transformation_matrix_to_pose_rpy(DGM);
          	  point.x = pose_rpy(0);  point.y = pose_rpy(1);  point.z = pose_rpy(2);
          	  point.r = 0;  point.g = 0;  point.b = 255;
          	  middle_workspace_cloud.points.push_back(point);
          	  *augmented_cloud = middle_workspace_cloud;
          	  *augmented_cloud += allegro_hand_cloud_xyzrgb;
          	  *augmented_cloud += thumb_workspace_cloud;
          	  *augmented_cloud += index_workspace_cloud;
        	  }
        	  else if(finger_list[finger_index]=="pinky"){
          	  joint_position_desired << (pinky_joint_safe_min(0)+pinky_joint_safe_range(0)*loop1/number_of_sample_points),
          	                            (pinky_joint_safe_min(1)+pinky_joint_safe_range(1)*loop2/number_of_sample_points),
          	                            (pinky_joint_safe_min(2)+pinky_joint_safe_range(2)*loop3/number_of_sample_points),
          	                            (pinky_joint_safe_min(3)+pinky_joint_safe_range(3)*loop4/number_of_sample_points);
          	  DGM  = finger_direct_geometric_model("pinky",joint_position_desired);
          	  pose_rpy = transformation_matrix_to_pose_rpy(DGM);
          	  point.x = pose_rpy(0);  point.y = pose_rpy(1);  point.z = pose_rpy(2);
          	  point.r = 100;  point.g = 100;  point.b = 100;
          	  pinky_workspace_cloud.points.push_back(point);
          	  *augmented_cloud = pinky_workspace_cloud;
          	  *augmented_cloud += allegro_hand_cloud_xyzrgb;
          	  *augmented_cloud += thumb_workspace_cloud;
          	  *augmented_cloud += index_workspace_cloud;
          	  *augmented_cloud += middle_workspace_cloud;
        	  }
          	
          	i++;
            std::cout<< "i:"<< i << ", loop1:" << loop1 << " loop2:" << loop2 << " loop3:" << loop3 << " loop4:" << loop4<<std::endl;
            
          	viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"allegro hand sampled workspace");
            viewer->spinOnce();
          }
        }
      }
    }
  }
  
  

	std::cout<< "Finished iterations!" <<std::endl;
	std::cout<< thumb_workspace_cloud.points.size() <<std::endl;
	
	thumb_workspace_cloud.width    = thumb_workspace_cloud.points.size();
	thumb_workspace_cloud.is_dense = false;
	thumb_workspace_cloud.points.resize(thumb_workspace_cloud.width * thumb_workspace_cloud.height);
	
	index_workspace_cloud.width    = index_workspace_cloud.points.size();
	index_workspace_cloud.is_dense = false;
	index_workspace_cloud.points.resize(index_workspace_cloud.width * index_workspace_cloud.height);
	
	middle_workspace_cloud.width    = middle_workspace_cloud.points.size();
	middle_workspace_cloud.is_dense = false;
	middle_workspace_cloud.points.resize(middle_workspace_cloud.width * middle_workspace_cloud.height);
	
	pinky_workspace_cloud.width    = pinky_workspace_cloud.points.size();
	pinky_workspace_cloud.is_dense = false;
	pinky_workspace_cloud.points.resize(pinky_workspace_cloud.width * pinky_workspace_cloud.height);
	
	cloud += thumb_workspace_cloud;
	cloud += index_workspace_cloud;
	cloud += middle_workspace_cloud;
	cloud += pinky_workspace_cloud;
	cloud.width    = cloud.points.size();
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
  
	// save the cloud data
  pcl::io::savePCDFileASCII("thumb_workspace_"+std::to_string(number_of_sample_points)+"_"+role+".pcd" , thumb_workspace_cloud);
	pcl::io::savePCDFileASCII("index_workspace_"+std::to_string(number_of_sample_points)+"_"+role+".pcd" , index_workspace_cloud);
	pcl::io::savePCDFileASCII("middle_workspace_"+std::to_string(number_of_sample_points)+"_"+role+".pcd", middle_workspace_cloud);
	pcl::io::savePCDFileASCII("pinky_workspace_"+std::to_string(number_of_sample_points)+"_"+role+".pcd" , pinky_workspace_cloud);
	pcl::io::savePCDFileASCII("allegro_right_hand_workspace_"+std::to_string(number_of_sample_points)+"_"+role+".pcd", cloud);
	
	viewer->spinOnce();
	// save point cloud as image
	//pcl::visualization::PCLVisualizer::saveScreenshot("allegrohand_sampled_workspace_point_cloud");
	viewer->saveScreenshot("allegro_hand_sampled_workspace_point_cloud_"+std::to_string(number_of_sample_points)+"_"+role+".png");
	
	std::vector<pcl::visualization::Camera> cam; 
  
	
	while (!viewer->wasStopped()){ 
	  viewer->spinOnce();
	  //transform = viewer->getViewerPose();
	  //std::cout << transform.translation() << endl;
	  //std::cout << transform.rotation() << endl;
	  
	  // use the code below to get the camera settings required to adjust orientation of view
	  // this works perfectly !
	  //Save the position of the camera           
    //viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
    //             << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
    //             << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
	}
  
  return 0;
}

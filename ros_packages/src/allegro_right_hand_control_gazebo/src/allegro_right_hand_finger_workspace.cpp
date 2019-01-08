#include <iostream>
#include <fstream>
#include <time.h>
#include "ros/ros.h"

//#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_lib_io.h>

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/io.h>

//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/console/parse.h>

#include <pcl/common/transforms.h>

//#include <pcl/io/openni_grabber.h>
//#include <pcl/common/time.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/AllegroRightHandModel.h"
#include "../include/allegro_right_hand.h"


using namespace allegro_right_hand;


void GetJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_position_sim_time = _msg->data[0];
	for(int k=0; k<4; k++)
		index_joint_position(k)  = _msg->data[k+1];
	for(int k=0; k<4; k++)
		middle_joint_position(k) = _msg->data[k+4+1];
	for(int k=0; k<4; k++)
		pinky_joint_position(k)  = _msg->data[k+8+1];
	for(int k=0; k<4; k++)
		thumb_joint_position(k)  = _msg->data[k+12+1];
}

void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	joint_velocity_sim_time = _msg->data[0];
	for(int k=0; k<4; k++)
		index_joint_velocity(k)  = _msg->data[k+1];
	for(int k=0; k<4; k++)
		middle_joint_velocity(k) = _msg->data[k+4+1];
	for(int k=0; k<4; k++)
		pinky_joint_velocity(k)  = _msg->data[k+8+1];
	for(int k=0; k<4; k++)
		thumb_joint_velocity(k)  = _msg->data[k+12+1];
}


int main(int argc, char **argv){
    
  // saving the finger workspace as a point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height   = 1;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // to handle RGB
  viewer->setBackgroundColor(255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(augmented_cloud);
  
  
  
  // loading allegro hand model point cloud
  pcl::PointCloud<pcl::PointXYZ> allegro_hand_cloud_xyz;
  pcl::PointCloud<pcl::PointXYZRGB> allegro_hand_cloud_xyzrgb;
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/allegro_right_hand_model_cloud.pcd", allegro_hand_cloud_xyz);
  
  // this will convert pcl::PointXYZ to pcl::PointXYZRGB but sets the RGB value to "0", so the color is black, make background white to be able to visualize
  copyPointCloud(allegro_hand_cloud_xyz, allegro_hand_cloud_xyzrgb);
  
  // modify the hand cloud : shift the zero point upwards by 95mm to be inline with the kinematic model we use
  for(int i=0;i<allegro_hand_cloud_xyz.points.size();i++){
    allegro_hand_cloud_xyzrgb.points[i].z += 0.095;
  }
  
  cloud = allegro_hand_cloud_xyzrgb;
  *augmented_cloud = allegro_hand_cloud_xyzrgb;
  viewer->addPointCloud(augmented_cloud,"workspace cloud");
	
	
	
  
  std::string model_name;
  model_name = argv[2];
  
  ros::init(argc, argv, "allegro_right_hand_controller");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub   = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/torque", 10);
	ros::Publisher  JointVelocityCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/velocity", 10);
	ros::Publisher  JointPositionCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/position", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+model_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+model_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(1000);
	
	//thumb_joint_position_desired  << pi, pi/2, pi/2, pi/2;
	//index_joint_position_desired  << pi/3, pi/3, 0.0, 0.0;
	thumb_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	index_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	middle_joint_position_desired << 0.0, 0.0, 0.0, 0.0;
	pinky_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	
	int number_of_sample_points = 3;
	
	
	
	pcl::PointXYZRGB point;
	int i =0;
  
  // thumb point cloud
  pcl::PointCloud<pcl::PointXYZRGB> thumb_workspace_cloud;
  thumb_workspace_cloud.height = 1;
	for(int loop1=0;loop1<=number_of_sample_points;loop1++){
	  for(int loop2=0;loop2<=number_of_sample_points;loop2++){
	    for(int loop3=0;loop3<=number_of_sample_points;loop3++){
	      for(int loop4=0;loop4<=number_of_sample_points;loop4++){
      	  thumb_joint_position_desired << (thumb_joint_safe_min(0)+thumb_joint_safe_range(0)*loop1/number_of_sample_points),
      	                                  (thumb_joint_safe_min(1)+thumb_joint_safe_range(1)*loop2/number_of_sample_points),
      	                                  (thumb_joint_safe_min(2)+thumb_joint_safe_range(2)*loop3/number_of_sample_points),
      	                                  (thumb_joint_safe_min(3)+thumb_joint_safe_range(3)*loop4/number_of_sample_points);
      	  
      	  Kp = 3.7;
        	//thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
        	thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position_desired);
        	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
        	thumb_joint_velocity_command  = Kp*(thumb_joint_position_desired-thumb_joint_position);
        	
        	
        	// Sending Joint VElocity Command
        	std_msgs::Float32MultiArray velocity_command;
        	velocity_command.data.clear();
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(index_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(middle_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(pinky_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(thumb_joint_velocity_command(k));
        	JointVelocityCommandPub.publish(velocity_command);
        	
          i++;
          std::cout<< "i:"<< i << ", loop1:" << loop1 << " loop2:" << loop2 << " loop3:" << loop3 << " loop4:" << loop4<<std::endl;
        	point.x = pose_rpy_Ptt(0);
          point.y = pose_rpy_Ptt(1);
          point.z = pose_rpy_Ptt(2);
          point.r = 255;
          point.g = 0;
          point.b = 0;
          thumb_workspace_cloud.points.push_back(point);
          *augmented_cloud = thumb_workspace_cloud;
          
          
        	if(!viewer->addPointCloud(augmented_cloud,"thumb workspace cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"thumb workspace cloud");
          
          // to visualize RGB
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "thumb workspace cloud");
          //viewer->addCoordinateSystem (1.0);
          //viewer->initCameraParameters ();
          
        	viewer->spinOnce();
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
  }
  
  
  // index point cloud
  pcl::PointCloud<pcl::PointXYZRGB> index_workspace_cloud;
  index_workspace_cloud.height = 1;
	for(int loop1=0;loop1<=number_of_sample_points;loop1++){
	  for(int loop2=0;loop2<=number_of_sample_points;loop2++){
	    for(int loop3=0;loop3<=number_of_sample_points;loop3++){
	      for(int loop4=0;loop4<=number_of_sample_points;loop4++){
      	  index_joint_position_desired << (index_joint_safe_min(0)+index_joint_safe_range(0)*loop1/number_of_sample_points),
      	                                  (index_joint_safe_min(1)+index_joint_safe_range(1)*loop2/number_of_sample_points),
      	                                  (index_joint_safe_min(2)+index_joint_safe_range(2)*loop3/number_of_sample_points),
      	                                  (index_joint_safe_min(3)+index_joint_safe_range(3)*loop4/number_of_sample_points);
      	  
      	  Kp = 3.7;
        	//index_DGM  = finger_direct_geometric_model("index",index_joint_position);
        	index_DGM  = finger_direct_geometric_model("index",index_joint_position_desired);
        	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(index_DGM);
        	index_joint_velocity_command  = Kp*(index_joint_position_desired-index_joint_position);
        	
        	
        	// Sending Joint VElocity Command
        	std_msgs::Float32MultiArray velocity_command;
        	velocity_command.data.clear();
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(index_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(middle_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(pinky_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(thumb_joint_velocity_command(k));
        	JointVelocityCommandPub.publish(velocity_command);
        	
          i++;
          std::cout<< "i:"<< i << ", loop1:" << loop1 << " loop2:" << loop2 << " loop3:" << loop3 << " loop4:" << loop4<<std::endl;
        	point.x = pose_rpy_Ptt(0);
          point.y = pose_rpy_Ptt(1);
          point.z = pose_rpy_Ptt(2);
          point.r = 0;
          point.g = 255;
          point.b = 0;
          index_workspace_cloud.points.push_back(point);
          *augmented_cloud = index_workspace_cloud;
          
          
        	if(!viewer->addPointCloud(augmented_cloud,"index workspace cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"index workspace cloud");
          
          // to visualize RGB
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "index workspace cloud");
          //viewer->addCoordinateSystem (1.0);
          //viewer->initCameraParameters ();
          
        	viewer->spinOnce();
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
  }
  
  
  // middle point cloud
  pcl::PointCloud<pcl::PointXYZRGB> middle_workspace_cloud;
  middle_workspace_cloud.height = 1;
	for(int loop1=0;loop1<=number_of_sample_points;loop1++){
	  for(int loop2=0;loop2<=number_of_sample_points;loop2++){
	    for(int loop3=0;loop3<=number_of_sample_points;loop3++){
	      for(int loop4=0;loop4<=number_of_sample_points;loop4++){
      	  middle_joint_position_desired << (middle_joint_safe_min(0)+middle_joint_safe_range(0)*loop1/number_of_sample_points),
      	                                  (middle_joint_safe_min(1)+middle_joint_safe_range(1)*loop2/number_of_sample_points),
      	                                  (middle_joint_safe_min(2)+middle_joint_safe_range(2)*loop3/number_of_sample_points),
      	                                  (middle_joint_safe_min(3)+middle_joint_safe_range(3)*loop4/number_of_sample_points);
      	  
      	  Kp = 3.7;
        	//middle_DGM  = finger_direct_geometric_model("middle",middle_joint_position);
        	middle_DGM  = finger_direct_geometric_model("middle",middle_joint_position_desired);
        	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(middle_DGM);
        	middle_joint_velocity_command  = Kp*(middle_joint_position_desired-middle_joint_position);
        	
        	
        	// Sending Joint VElocity Command
        	std_msgs::Float32MultiArray velocity_command;
        	velocity_command.data.clear();
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(index_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(middle_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(pinky_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(thumb_joint_velocity_command(k));
        	JointVelocityCommandPub.publish(velocity_command);
        	
          i++;
          std::cout<< "i:"<< i << ", loop1:" << loop1 << " loop2:" << loop2 << " loop3:" << loop3 << " loop4:" << loop4<<std::endl;
        	point.x = pose_rpy_Ptt(0);
          point.y = pose_rpy_Ptt(1);
          point.z = pose_rpy_Ptt(2);
          point.r = 0;
          point.g = 0;
          point.b = 255;
          middle_workspace_cloud.points.push_back(point);
          *augmented_cloud = middle_workspace_cloud;
          
          
        	if(!viewer->addPointCloud(augmented_cloud,"middle workspace cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"middle workspace cloud");
          
          // to visualize RGB
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "middle workspace cloud");
          //viewer->addCoordinateSystem (1.0);
          //viewer->initCameraParameters ();
          
        	viewer->spinOnce();
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
  }
  
  
  // pinky point cloud
  pcl::PointCloud<pcl::PointXYZRGB> pinky_workspace_cloud;
  pinky_workspace_cloud.height = 1;
	for(int loop1=0;loop1<=number_of_sample_points;loop1++){
	  for(int loop2=0;loop2<=number_of_sample_points;loop2++){
	    for(int loop3=0;loop3<=number_of_sample_points;loop3++){
	      for(int loop4=0;loop4<=number_of_sample_points;loop4++){
      	  pinky_joint_position_desired << (pinky_joint_safe_min(0)+pinky_joint_safe_range(0)*loop1/number_of_sample_points),
      	                                  (pinky_joint_safe_min(1)+pinky_joint_safe_range(1)*loop2/number_of_sample_points),
      	                                  (pinky_joint_safe_min(2)+pinky_joint_safe_range(2)*loop3/number_of_sample_points),
      	                                  (pinky_joint_safe_min(3)+pinky_joint_safe_range(3)*loop4/number_of_sample_points);
      	  
      	  Kp = 3.7;
        	//pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);
        	pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position_desired);
        	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(pinky_DGM);
        	pinky_joint_velocity_command  = Kp*(pinky_joint_position_desired-pinky_joint_position);
        	
        	
        	// Sending Joint VElocity Command
        	std_msgs::Float32MultiArray velocity_command;
        	velocity_command.data.clear();
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(index_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(middle_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(pinky_joint_velocity_command(k));
        	for(int k=0; k<4; k++)
        		velocity_command.data.push_back(thumb_joint_velocity_command(k));
        	JointVelocityCommandPub.publish(velocity_command);
        	
          i++;
          std::cout<< "i:"<< i << ", loop1:" << loop1 << " loop2:" << loop2 << " loop3:" << loop3 << " loop4:" << loop4<<std::endl;
        	point.x = pose_rpy_Ptt(0);
          point.y = pose_rpy_Ptt(1);
          point.z = pose_rpy_Ptt(2);
          point.r = 100;
          point.g = 100;
          point.b = 100;
          pinky_workspace_cloud.points.push_back(point);
          *augmented_cloud = pinky_workspace_cloud;
          
          
        	if(!viewer->addPointCloud(augmented_cloud,"pinky workspace cloud"))
            viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb,"pinky workspace cloud");
          
          // to visualize RGB
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pinky workspace cloud");
          //viewer->addCoordinateSystem (1.0);
          //viewer->initCameraParameters ();
          
        	viewer->spinOnce();
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
  }

	std::cout<< "Finished iterations!" <<std::endl;
	
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
	
	// Fill in the cloud data
  /*
  pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/thumb_workspace.pcd" , thumb_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/index_workspace.pcd" , index_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/middle_workspace.pcd", middle_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/pinky_workspace.pcd" , pinky_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/allegro_right_hand_workspace.pcd", cloud);
	*/
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/thumb_workspace_light.pcd" , thumb_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/index_workspace_light.pcd" , index_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/middle_workspace_light.pcd", middle_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/pinky_workspace_light.pcd" , pinky_workspace_cloud);
	pcl::io::savePCDFileASCII("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/allegro_right_hand_workspace_light.pcd", cloud);
	
	// load thumb ellipsoid workspace subset
	pcl::PointCloud<pcl::PointXYZ> thumb_ellipsoid_workspace_cloud_xyz;
  pcl::PointCloud<pcl::PointXYZRGB> thumb_ellipsoid_workspace_cloud_xyzrgb;
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/thumb_workspace_as_ellipsoids.pcd", thumb_ellipsoid_workspace_cloud_xyz);
	copyPointCloud(thumb_ellipsoid_workspace_cloud_xyz, thumb_ellipsoid_workspace_cloud_xyzrgb);
	
	// Visualize to check
	//pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/allegro_right_hand_workspace.pcd", *augmented_cloud);
	augmented_cloud->clear();
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/work/code_repository/ros_packages/src/allegro_right_hand_control_gazebo/src/allegro_right_hand_workspace_light.pcd", *augmented_cloud);
	*augmented_cloud += thumb_ellipsoid_workspace_cloud_xyzrgb;
  viewer->addPointCloud(augmented_cloud,"allegro hand workspace cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "allegro hand workspace cloud");
	
	while (!viewer->wasStopped()){ viewer->spinOnce(); }
	return 0;
}

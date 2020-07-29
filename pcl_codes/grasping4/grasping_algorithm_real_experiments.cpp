
/*
before starting this program, start first the realsense ROS nodelet using:
roslaunch realsense2_camera rs_rgbd.launch

then run using:
reset && cmake .. && make && ./grasping_algorithm_real_experiments storage_bin2 ../include/gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera.pcd
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int8.h>

#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
//#include "../include/Eigen/Dense"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <unistd.h>

#include "include/declarations.h"
#include "include/useful_implementations.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "include/grasping_algorithm.h"

//boost::shared_ptr<pcl::visualization::PCLVisualizer> realsense_viewer  (new pcl::visualization::PCLVisualizer("realsense cloud"));

pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud                            (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_xyz                        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    augmented_cloud_filtered_xyz               (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_xyzrgb                 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    current_scene_cloud_xyz                    (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scene_cloud_transformed_xyzrgb     (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    current_scene_cloud_transformed_xyz        (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr    dummy_xyz                                  (new pcl::PointCloud<pcl::PointXYZ>);

std::string file_name;

void view_point_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  *current_scene_cloud_xyzrgb = *temp_cloud;
  std::cout << "point cloud arrived ... " <<std::endl;
}


int main(int argc, char **argv){
  std::string point_cloud_name, point_cloud_id;
  point_cloud_name   = argv[1];
  
  // ROS
  ros::init(argc, argv, "grasping_algorithm_real_experiments");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);  // once per 10 seconds
  
  //realsense_viewer->addPointCloud(current_scene_cloud_xyzrgb, rgb,"realsense cloud");
  //realsense_viewer->addCoordinateSystem(0.2);
  
  ros::Publisher pose_pub           = n.advertise<geometry_msgs::PoseStamped> ("/panda_arm/command/pose", 1);
  ros::Publisher joint_position_pub = n.advertise<std_msgs::Int8>             ("/panda_arm/command/joint_position", 10);
  
  ros::Subscriber realsense_sub     = n.subscribe<sensor_msgs::PointCloud2>   ("/camera/depth/color/points", 1, view_point_cloud);
  
  
  
  tf::TransformListener listener;
  tf::StampedTransform stamped_transform;
  
  //realsense_viewer->setBackgroundColor(255,255,255);
  


  
  // arm pose
  geometry_msgs::PoseStamped poseCommand;
  geometry_msgs::PoseStamped poseResponse;
  poseCommand.header.frame_id="/panda_link0";
  
  Eigen::Vector3f camera_depth_frame_new_wrt_link0_translation;
  Eigen::Matrix3f camera_depth_frame_new_wrt_link0_rotation;
  Eigen::Matrix4f camera_depth_frame_new_wrt_link0_transform;
  
  Eigen::Vector3f camera_depth_frame_old_wrt_link0_translation;
  Eigen::Matrix3f camera_depth_frame_old_wrt_link0_rotation;
  Eigen::Matrix4f camera_depth_frame_old_wrt_link0_transform;
  
  Eigen::Matrix4f camera_depth_frame_new_wrt_camera_depth_frame_old_transform;
  Eigen::Quaternionf q;
  
  
  // This transformation matrix will be used later on to go to desired gripper pose
	// get current panda_hand with respect to panda_link0 transformation matrix
	//Eigen::Matrix3f arm_hand_wrt_arm_link0_frame_rotation = Eigen::Quaternionf{Eigen::AngleAxisf{0.3659975, Eigen::Vector3f{-0.01241378, 0.01459942, -0.9304185}}}.toRotationMatrix();
	Eigen::Vector3f arm_hand_wrt_arm_link0_frame_translation_old;
	Eigen::Matrix3f arm_hand_wrt_arm_link0_frame_rotation_old;
	Eigen::Matrix4f arm_hand_wrt_arm_link0_frame_transform_old;
  
  
  std_msgs::Int8 config_index;
  config_index.data = 0;
  
  clock_t begin7, end;
	double time_spent;
	
	ofstream transformation_matrix_file;
  transformation_matrix_file.open("../raw_object_pcd_files/"+ point_cloud_name + "_tf.txt");
  std::string tf_string;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // STEP#1 : Getting the object + table point clouds
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if ( ros::ok() ){
  	// get the current camera_depth_frame transform with respect to link0
		ros::Duration(1).sleep();
		try{listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), stamped_transform);}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
		poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
		poseResponse.pose.orientation.x = stamped_transform.getRotation().x();
	
		camera_depth_frame_old_wrt_link0_translation << stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z();
		q.x() = stamped_transform.getRotation().x();
		q.y() = stamped_transform.getRotation().y();
		q.z() = stamped_transform.getRotation().z();
		q.w() = stamped_transform.getRotation().w();
		camera_depth_frame_old_wrt_link0_rotation = q.normalized().toRotationMatrix();
		camera_depth_frame_old_wrt_link0_transform << camera_depth_frame_old_wrt_link0_rotation, camera_depth_frame_old_wrt_link0_translation,
					                          0,0,0,1;
		
		std::cout << "camera_depth_frame_old_wrt_link0_transform = " <<std::endl << camera_depth_frame_old_wrt_link0_transform <<std::endl;
  	
  	
  	// This transformation matrix will be used later on to go to desired gripper pose
  	// get current panda_hand with respect to panda_link0 transformation matrix
		try{listener.lookupTransform("/panda_link0", "/panda_hand", ros::Time(0), stamped_transform);}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
		poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
		poseResponse.pose.orientation.x = stamped_transform.getRotation().x();

		arm_hand_wrt_arm_link0_frame_translation_old << stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z();
		q.x() = stamped_transform.getRotation().x();
		q.y() = stamped_transform.getRotation().y();
		q.z() = stamped_transform.getRotation().z();
		q.w() = stamped_transform.getRotation().w();
		arm_hand_wrt_arm_link0_frame_rotation_old = q.normalized().toRotationMatrix();
		arm_hand_wrt_arm_link0_frame_transform_old << arm_hand_wrt_arm_link0_frame_rotation_old, arm_hand_wrt_arm_link0_frame_translation_old,
						                                      0,0,0,1;
  	
  	// capturing the 3 view point clouds
    for(int i=0;i<3;i++){
      config_index.data = i;
      joint_position_pub.publish( config_index );
			ros::Duration(8).sleep();
		  ros::spinOnce();
			
      // get the current camera_depth_frame transform with respect to link0
			try{listener.lookupTransform("/panda_link0", "/camera_depth_optical_frame", ros::Time(0), stamped_transform);}
			catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
			poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
			poseResponse.pose.orientation.x = stamped_transform.getRotation().x();
			
			camera_depth_frame_new_wrt_link0_translation << stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z();
			q.x() = stamped_transform.getRotation().x();
			q.y() = stamped_transform.getRotation().y();
			q.z() = stamped_transform.getRotation().z();
			q.w() = stamped_transform.getRotation().w();
			camera_depth_frame_new_wrt_link0_rotation = q.normalized().toRotationMatrix();
			camera_depth_frame_new_wrt_link0_transform << camera_depth_frame_new_wrt_link0_rotation, camera_depth_frame_new_wrt_link0_translation,
			                                0,0,0,1;
			
			dummy_xyz->clear();
			// remove far and near points
			for(int j=0; j<current_scene_cloud_xyzrgb->size(); j++){
				if(current_scene_cloud_xyzrgb->points[j].z < 0.7 and current_scene_cloud_xyzrgb->points[j].z > 0.3){
					point_xyz.x = current_scene_cloud_xyzrgb->points[j].x;
					point_xyz.y = current_scene_cloud_xyzrgb->points[j].y;
					point_xyz.z = current_scene_cloud_xyzrgb->points[j].z;
					dummy_xyz->points.push_back(point_xyz);
				}
			}
			*current_scene_cloud_xyz = *dummy_xyz;
			current_scene_cloud_xyz->width = current_scene_cloud_xyz->points.size();
			current_scene_cloud_xyz->height = 1;
			current_scene_cloud_xyz->is_dense = true;
			
			
			// save individual point cloud
  		file_name = "../raw_object_pcd_files/"+ point_cloud_name + "_" + std::to_string(i) + ".pcd";
			pcl::io::savePCDFileASCII(file_name, *current_scene_cloud_xyz);
			
      camera_depth_frame_new_wrt_camera_depth_frame_old_transform = camera_depth_frame_old_wrt_link0_transform.inverse()*camera_depth_frame_new_wrt_link0_transform;
      pcl::transformPointCloud (*current_scene_cloud_xyzrgb, *current_scene_cloud_transformed_xyzrgb, camera_depth_frame_new_wrt_camera_depth_frame_old_transform);
      *augmented_cloud += *current_scene_cloud_transformed_xyzrgb;
      
      std::stringstream ss;
		  ss << camera_depth_frame_new_wrt_camera_depth_frame_old_transform;
		  tf_string += ss.str() + "\n";
      
      
      if(i==0){
        *scene_cloud_xyz_1 = *current_scene_cloud_xyz;
        tm1 = camera_depth_frame_new_wrt_camera_depth_frame_old_transform;}
      if(i==1){
        *scene_cloud_xyz_2 = *current_scene_cloud_xyz;
        tm2 = camera_depth_frame_new_wrt_camera_depth_frame_old_transform;}
      if(i==2){
        *scene_cloud_xyz_3 = *current_scene_cloud_xyz;
        tm3 = camera_depth_frame_new_wrt_camera_depth_frame_old_transform;}
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  
  // save point cloud data
  file_name = "../raw_object_pcd_files/"+ point_cloud_name + ".pcd";
  pcl::io::savePCDFileASCII(file_name, *augmented_cloud);
  
  transformation_matrix_file << tf_string;
  transformation_matrix_file.close();
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  gripper_file_name      = argv[2];
  
  if(gripper_file_name.find("allegro_right_hand" )!=std::string::npos){gripper_model = "allegro_right_hand";}
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){gripper_model = "franka_gripper";}
  
  
  // visualization of point cloud
  scene_cloud_viewer->addCoordinateSystem(0.2);   // this is arm hand frame (the origin)
  scene_cloud_viewer->setCameraPosition(-1.53884, 0.506528, -0.636167, -0.171077, 0.068023, 0.333948, 0.522106, -0.203709, -0.828196, 0);
  scene_cloud_viewer->setBackgroundColor(255,255,255);
  
  scene_cloud_viewer->addPointCloud(object_cloud_downsampled_in_arm_hand_frame_xyz, magenta_color,                    "object cloud");
  scene_cloud_viewer->addPointCloud(object_sampling_in_arm_hand_frame_xyz, blue_color_again,                          "object sampling cloud");
  
  scene_cloud_viewer->addPointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyz, brown_color,                "table cloud");
  scene_cloud_viewer->addPointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, orange_color,       "table special ellipsoid");
  
  scene_cloud_viewer->addPointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyz, cyan_color_again,                "gripper cloud in arm hand frame");
  scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyz, cyan_color,                      "gripper cloud transformed in arm hand frame");
  //scene_cloud_viewer->addPointCloud(gripper_cloud_transformed_in_object_plane_frame_xyz, black_color_again,           "gripper cloud in object plane frame");
  scene_cloud_viewer->addPointCloud(gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, black_color,  "gripper special ellipsoids transformed");
  scene_cloud_viewer->addPointCloud(gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, black_color_again,        "gripper special ellipsoids");
  
  
  
  
  cout << "scene_cloud_xyz_1->points.size() = " << scene_cloud_xyz_1->points.size() << endl;
  cout << "scene_cloud_xyz_2->points.size() = " << scene_cloud_xyz_2->points.size() << endl;
  cout << "scene_cloud_xyz_3->points.size() = " << scene_cloud_xyz_3->points.size() << endl;
  
  cout << "tm1 = " << endl << tm1 << endl;
  cout << "tm2 = " << endl << tm2 << endl;
  cout << "tm3 = " << endl << tm3 << endl;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Downsampling, Regestering, segmenting object's 3 view point clouds
  reader.read(gripper_file_name, *gripper_cloud_downsampled_in_gripper_frame_xyz);
  /*
  if(mode=="simulation")
    load_object_3_view_point_clouds_and_corresponding_transforms();
  else if(mode=="experiment"){
  }
  */
  initial_overhead_begin = clock();
	registering_downsampling_segmenting_3_view_point_clouds(scene_cloud_xyz_1, tm1,   scene_cloud_xyz_2, tm2,   scene_cloud_xyz_3, tm3,
                                                          scene_cloud_xyz_1_transformed, scene_cloud_xyz_2_transformed, scene_cloud_xyz_3_transformed,
                                                          scene_cloud_xyz_1_transformed_downsampled, scene_cloud_xyz_2_transformed_downsampled, scene_cloud_xyz_3_transformed_downsampled,
                                                          object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz, object_cloud_downsampled_in_camera_depth_optical_frame_xyz );
  
  std::cout << "Object (downsampled) point cloud has:  " << object_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()        << " data points." << std::endl;
  std::cout << "Table (downsampled) point cloud has:   " << object_plane_cloud_downsampled_in_camera_depth_optical_frame_xyz->points.size()  << " data points." << std::endl;
  std::cout << "Gripper (downsampled) point cloud has: " << gripper_cloud_downsampled_in_gripper_frame_xyz->points.size()                    << " data points." << std::endl;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  load_transformations();
  object_pose_approximation_and_sampling();
  object_plane_pose_approximation();
  constructing_special_ellipsoids();
  object_plane_pose_check();
  load_gripper_workspace_spheres_and_compute_centroid();
  
  
  end = clock();
	time_spent_for_initial_overhead = (double)( end - initial_overhead_begin )/ CLOCKS_PER_SEC;
	std::cout << "total initial overhead (to add to the execution time) = " << time_spent_for_initial_overhead << std::endl;
  
  end = clock();
	time_spent = (double)( end - time_to_load_clouds_begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent to load stuff = " << time_spent << std::endl;
  
  //char ch;
  //std::cout << "maximuize screen please ..." << std::endl;
  //std::cin >> ch;
  
  time_to_run_algorithm_begin = clock();
  begin2 = clock();
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // scanning for best grasp pose
  evaluate_grasp_pose_candidates();
  
  end = clock();
	time_spent = (double)( end - begin2 )/ CLOCKS_PER_SEC;
	std::cout << "time spent in checking gripper collision with table        = " << time_elapsed_checking_gripper_collision_with_table        << std::endl;
	std::cout << "time spent in checking gripper collision with object       = " << time_elapsed_checking_gripper_collision_with_object       << std::endl;
  std::cout << "time spent in checking object contact with gripper support = " << time_elapsed_checking_object_contact_with_gripper_support << std::endl;
  std::cout << "time spent in scanning for best gripper pose               = " << time_spent << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyz->points.size() << " iterations, gripper collide with table  : " << gripper_collide_with_table   << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyz->points.size() << " iterations, gripper collide with object : " << gripper_collide_with_object  << " times." << std::endl;
  std::cout << "out of " << orientation_samples*object_sampling_in_arm_hand_frame_xyz->points.size() << " iterations, gripper contacts with object: " << gripper_contacts_with_object << " times." << std::endl;
  
  end = clock();
	time_spent = (double)( end - time_to_run_algorithm_begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by the ALGORITHM = " << time_spent << std::endl;
  std::cout << "total spent (ALGORITHM + initial overhead) = " << time_spent+time_spent_for_initial_overhead << std::endl;
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  visualize_workspace_spheres_and_object_points_for_best_gripper_pose();
  
  
  
  
  
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // object cloud
  scene_cloud_viewer->updatePointCloud(object_cloud_downsampled_in_arm_hand_frame_xyz, magenta_color,                      "object cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "object cloud");
  
  // object plane cloud
  scene_cloud_viewer->updatePointCloud(object_plane_cloud_downsampled_in_arm_hand_frame_xyz, brown_color,                  "table cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,                   "table cloud");
  
  // object plane special ellipsoid
  scene_cloud_viewer->updatePointCloud(object_plane_special_ellipsoid_point_cloud_in_arm_hand_frame, orange_color,         "table special ellipsoid");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,                   "table special ellipsoid");
  
  // object sampling cloud
  scene_cloud_viewer->updatePointCloud(object_sampling_in_arm_hand_frame_xyz, blue_color_again,                            "object sampling cloud");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "object sampling cloud");
  
  // gripper cloud transformed in arm hand frame
  pcl::transformPointCloud(*gripper_cloud_downsampled_in_arm_hand_frame_xyz, *gripper_cloud_transformed_in_arm_hand_frame_xyz, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(gripper_cloud_transformed_in_arm_hand_frame_xyz, cyan_color,                        "gripper cloud transformed in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "gripper cloud transformed in arm hand frame");
  
  // gripper cloud in arm hand frame
  scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyz, cyan_color_again,                  "gripper cloud in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "gripper cloud in arm hand frame");
  
  // gripper special ellipsoids transformed
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, *gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, black_color,    "gripper special ellipsoids transformed");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper special ellipsoids transformed");
  
  // gripper special ellipsoids
  scene_cloud_viewer->updatePointCloud(gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, black_color_again,          "gripper special ellipsoids");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper special ellipsoids");
  
  scene_cloud_viewer->spinOnce();
  
  
  
  // output
  std::cout << "best_gripper_transform = " << std::endl << best_gripper_transform << std::endl;
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by this program = " << time_spent << std::endl;
  
  //std::vector<pcl::visualization::Camera> cam;
  while ( !scene_cloud_viewer->wasStopped() ){
    scene_cloud_viewer->spinOnce();
    //if(gripper_model == "allegro_right_hand")
    //  save_file_name = "allegro_video/allegro_best.png";
    //else if(gripper_model == "franka_gripper")
    //  save_file_name = "franka_video/franka_best.png";
    //scene_cloud_viewer->saveScreenshot(save_file_name);
    //scene_cloud_viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  char execute;

  std::cout << "Press [y] to execute or [n] to abort ... " << std::endl;
  std::cin.get(execute);
  
  if(execute == 'y'){
  	std::cout << "Execute grasping in progress ... " << std::endl;
  	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		// STEP#3 : move the gripper to desired grasping pose
		//
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// desired gripper transformation matrix with respect to panda_hand frame
		Eigen::Matrix4f desired_hand_transform_in_arm_hand_frame;
		desired_hand_transform_in_arm_hand_frame = best_gripper_transform;
	
		// desired hand transform in link0 frame
		Eigen::Matrix4f desired_hand_transform_in_arm_link0_frame;
		desired_hand_transform_in_arm_link0_frame = arm_hand_wrt_arm_link0_frame_transform_old*desired_hand_transform_in_arm_hand_frame;
		std::cout << "desired_hand_transform_in_arm_link0_frame = " <<std::endl << desired_hand_transform_in_arm_link0_frame <<std::endl;
		
		Eigen::Matrix3f desired_hand_rotation_in_arm_link0_frame;
		desired_hand_rotation_in_arm_link0_frame << desired_hand_transform_in_arm_link0_frame(0,0), desired_hand_transform_in_arm_link0_frame(0,1), desired_hand_transform_in_arm_link0_frame(0,2),
		                                            desired_hand_transform_in_arm_link0_frame(1,0), desired_hand_transform_in_arm_link0_frame(1,1), desired_hand_transform_in_arm_link0_frame(1,2),
		                                            desired_hand_transform_in_arm_link0_frame(2,0), desired_hand_transform_in_arm_link0_frame(2,1), desired_hand_transform_in_arm_link0_frame(2,2);
	
		Eigen::Quaternionf desired_hand_rotation_in_arm_link0_frame_quat(desired_hand_rotation_in_arm_link0_frame);
		std::cout << "best_hand_rotation = " << desired_hand_rotation_in_arm_link0_frame_quat.x() << ", " << desired_hand_rotation_in_arm_link0_frame_quat.y() << ", " << desired_hand_rotation_in_arm_link0_frame_quat.z() << ", " << desired_hand_rotation_in_arm_link0_frame_quat.w() << std::endl;
		
		Eigen::Vector3f desired_hand_translation_in_arm_link0_frame;
		desired_hand_translation_in_arm_link0_frame << desired_hand_transform_in_arm_link0_frame(0,3), desired_hand_transform_in_arm_link0_frame(1,3), desired_hand_transform_in_arm_link0_frame(2,3);
		
		poseCommand.pose.position.x = desired_hand_translation_in_arm_link0_frame(0);
		poseCommand.pose.position.y = desired_hand_translation_in_arm_link0_frame(1);
		poseCommand.pose.position.z = desired_hand_translation_in_arm_link0_frame(2) + 0.1;
		//poseCommand.pose.position.z = 0.17;
		poseCommand.pose.orientation.x = desired_hand_rotation_in_arm_link0_frame_quat.x();
		poseCommand.pose.orientation.y = desired_hand_rotation_in_arm_link0_frame_quat.y();
		poseCommand.pose.orientation.z = desired_hand_rotation_in_arm_link0_frame_quat.z();
		poseCommand.pose.orientation.w = desired_hand_rotation_in_arm_link0_frame_quat.w();
		
		for(int i=0;i<1;i++){
		  
		  // send pose command
			pose_pub.publish( poseCommand );
		
			// wait for 10 seconds for motion execution
			ros::Duration(10).sleep();
			ros::spinOnce();
		  loop_rate.sleep();
		}
		
		
		// move in z-axis only
		poseCommand.pose.position.x = desired_hand_translation_in_arm_link0_frame(0);
		poseCommand.pose.position.y = desired_hand_translation_in_arm_link0_frame(1);
		poseCommand.pose.position.z = desired_hand_translation_in_arm_link0_frame(2);
		//poseCommand.pose.position.z = 0.17;
		poseCommand.pose.orientation.x = desired_hand_rotation_in_arm_link0_frame_quat.x();
		poseCommand.pose.orientation.y = desired_hand_rotation_in_arm_link0_frame_quat.y();
		poseCommand.pose.orientation.z = desired_hand_rotation_in_arm_link0_frame_quat.z();
		poseCommand.pose.orientation.w = desired_hand_rotation_in_arm_link0_frame_quat.w();
		
		for(int i=0;i<1;i++){
		  
		  // send pose command
			pose_pub.publish( poseCommand );
		
			// wait for 10 seconds for motion execution
			ros::Duration(5).sleep();
			ros::spinOnce();
		  loop_rate.sleep();
		}
		
		
		
  
  }
  else if(execute == 'n')
  	std::cout << "Abort ... " << std::endl;
  
  
  
  
  
  
  
  
  std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  
  
  char lift_object;

  std::cout << "Press [y] to lift object or [n] to abort ... " << std::endl;
  std::cin.get(lift_object);
  
  if(lift_object == 'y'){
  	std::cout << "lifting grasped object upwards in progress ... " << std::endl;
  	  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		// STEP#4 : lifting the grasped object upwards
		//
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		
		// get current panda_hand with respect to panda_link0 transformation matrix
		try{listener.lookupTransform("/panda_link0", "/panda_hand", ros::Time(0), stamped_transform);}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());	ros::Duration(1.0).sleep();}
		poseResponse.pose.position.x    = stamped_transform.getOrigin().x();
		poseResponse.pose.orientation.x = stamped_transform.getRotation().x();
		
		poseCommand.pose.position.x    = stamped_transform.getOrigin().x();
		poseCommand.pose.position.y    = stamped_transform.getOrigin().y();
		poseCommand.pose.position.z    = stamped_transform.getOrigin().z() + 0.2;
		poseCommand.pose.orientation.x = stamped_transform.getRotation().x();
		poseCommand.pose.orientation.y = stamped_transform.getRotation().y();
		poseCommand.pose.orientation.z = stamped_transform.getRotation().z();
		poseCommand.pose.orientation.w = stamped_transform.getRotation().w();
		
		for(int i=0;i<1;i++){
		  
		  // send pose command
			pose_pub.publish( poseCommand );
		
			// wait for 10 seconds for motion execution
			ros::Duration(10).sleep();
			ros::spinOnce();
		  loop_rate.sleep();
		}
  
  }
  else if(lift_object == 'n')
  	std::cout << "Abort ... " << std::endl;
  
  
  
  
  
  
  
  
  return 0;
}




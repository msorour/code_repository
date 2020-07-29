/*
run this program using:

reset && cmake .. && make && ./grasping_algorithm simulation ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/storage_bin_0.pcd ../raw_object_pcd_files/storage_bin_1.pcd ../raw_object_pcd_files/storage_bin_2.pcd ../raw_object_pcd_files/storage_bin_tf.txt

reset && cmake .. && make && ./grasping_algorithm simulation ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/dish_brush_0.pcd ../raw_object_pcd_files/dish_brush_1.pcd ../raw_object_pcd_files/dish_brush_2.pcd ../raw_object_pcd_files/dish_brush_tf.txt

reset && cmake .. && make && ./grasping_algorithm simulation ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled.pcd ../raw_object_pcd_files/sprayer_0.pcd ../raw_object_pcd_files/sprayer_1.pcd ../raw_object_pcd_files/sprayer_2.pcd ../raw_object_pcd_files/sprayer_tf.txt

reset && cmake .. && make -j7 && ./grasping_algorithm simulation ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled_100.pcd ../raw_object_pcd_files/cup_without_handle_0.pcd ../raw_object_pcd_files/cup_without_handle_1.pcd ../raw_object_pcd_files/cup_without_handle_2.pcd ../raw_object_pcd_files/cup_without_handle_tf.txt

reset && cmake .. && make -j7 && ./grasping_algorithm simulation ../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled_100.pcd ../raw_object_pcd_files/cup_without_handle

*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include "include/declarations.h"
#include "include/useful_implementations.h"
#include "include/grasping_algorithm.h"
#include <math.h>
#include "QuadProg++.hh"


int main (int argc, char** argv){
  begin = clock();
	time_to_load_clouds_begin = clock();
	
	mode = argv[1];
  gripper_file_name = argv[2];
  
  if(mode=="simulation"){
    object_file_name = argv[3];
		file_name1 = object_file_name+"_0.pcd";
		file_name2 = object_file_name+"_1.pcd";
		file_name3 = object_file_name+"_2.pcd";
		transformation_matrix_file_name = object_file_name+"_tf.txt";
  }
  else if(mode=="experiment"){
    object_file_name = argv[3];
  }
  
  if(gripper_file_name.find("allegro_right_hand" )!=std::string::npos){gripper_model = "allegro_right_hand";}
  else if(gripper_file_name.find("franka_gripper")!=std::string::npos){gripper_model = "franka_gripper";}
  
  
  // visualization of point cloud
  //scene_cloud_viewer->addCoordinateSystem(0.2);   // this is arm hand frame (the origin)
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
  
	/*
  scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame, orange_color_again,                 "gripper support ellipsoid");
  scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame, orange_color_again2,    "gripper support transformed ellipsoid");
  */
	
	if(gripper_model == "allegro_right_hand"){
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame_1, orange_color_again_1,                 "gripper support1 ellipsoid");
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_1, orange_color_again2_1,    "gripper support1 transformed ellipsoid");
		
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame_2, orange_color_again_2,                 "gripper support2 ellipsoid");
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_2, orange_color_again2_2,    "gripper support2 transformed ellipsoid");
		
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame_3, orange_color_again_3,                 "gripper support3 ellipsoid");
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_3, orange_color_again2_3,    "gripper support3 transformed ellipsoid");
		
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame_4, orange_color_again_4,                 "gripper support4 ellipsoid");
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_4, orange_color_again2_4,    "gripper support4 transformed ellipsoid");
	}
	else if(gripper_model == "franka_gripper"){
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame_1, orange_color_again_1,                 "gripper support1 ellipsoid");
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_1, orange_color_again2_1,    "gripper support1 transformed ellipsoid");
		
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_in_arm_hand_frame_2, orange_color_again_2,                 "gripper support2 ellipsoid");
		scene_cloud_viewer->addPointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_2, orange_color_again2_2,    "gripper support2 transformed ellipsoid");
	}
  
  //downsample_gripper_pointcloud(100);
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Downsampling, Regestering, segmenting object's 3 view point clouds
  reader.read(gripper_file_name, *gripper_cloud_downsampled_in_gripper_frame_xyz);
  if(mode=="simulation")
    load_object_3_view_point_clouds_and_corresponding_transforms();
  else if(mode=="experiment"){
  }
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
  std::cout << "out of " << sampling_iteration << " iterations, gripper collide with table  : " << gripper_collide_with_table   << " times." << std::endl;
  std::cout << "out of " << sampling_iteration << " iterations, gripper collide with object : " << gripper_collide_with_object  << " times." << std::endl;
  std::cout << "out of " << sampling_iteration << " iterations, gripper contacts with object: " << gripper_contacts_with_object << " times." << std::endl;
  
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
  /*
  // gripper cloud in arm hand frame
  scene_cloud_viewer->updatePointCloud(gripper_cloud_downsampled_in_arm_hand_frame_xyz, cyan_color_again,                  "gripper cloud in arm hand frame");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,                  "gripper cloud in arm hand frame");
  */
  // gripper special ellipsoids transformed
  pcl::transformPointCloud(*gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, *gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, best_gripper_transform);
  scene_cloud_viewer->updatePointCloud(gripper_as_set_of_special_ellipsoids_transformed_in_arm_hand_frame, black_color,    "gripper special ellipsoids transformed");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper special ellipsoids transformed");
  /*
  // gripper special ellipsoids
  scene_cloud_viewer->updatePointCloud(gripper_as_set_of_special_ellipsoids_in_arm_hand_frame, black_color_again,          "gripper special ellipsoids");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper special ellipsoids");
  */
	/*
	// gripper support ellipsoid
	scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame, orange_color_again,                   "gripper support ellipsoid");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support ellipsoid");
	
	// gripper support transformed ellipsoid
	pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame, *gripper_support_point_cloud_transformed_in_arm_hand_frame, best_gripper_transform);
	scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame, orange_color_again2,      "gripper support transformed ellipsoid");
  scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support transformed ellipsoid");
	*/
	
	if(gripper_model == "allegro_right_hand"){
		// gripper support1 ellipsoid
		//scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame_1, orange_color_again_1,                   "gripper support1 ellipsoid");
		//scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support1 ellipsoid");
		// gripper support1 transformed ellipsoid
		pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame_1, *gripper_support_point_cloud_transformed_in_arm_hand_frame_1, best_gripper_transform);
		scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_1, orange_color_again2_1,      "gripper support1 transformed ellipsoid");
		scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support1 transformed ellipsoid");
		
		// gripper support2 ellipsoid
		//scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame_2, orange_color_again_2,                   "gripper support2 ellipsoid");
		//scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support2 ellipsoid");
		// gripper support2 transformed ellipsoid
		pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame_2, *gripper_support_point_cloud_transformed_in_arm_hand_frame_2, best_gripper_transform);
		scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_2, orange_color_again2_2,      "gripper support2 transformed ellipsoid");
		scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support2 transformed ellipsoid");
		
		// gripper support3 ellipsoid
		//scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame_3, orange_color_again_3,                   "gripper support3 ellipsoid");
		//scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support3 ellipsoid");
		// gripper support3 transformed ellipsoid
		pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame_3, *gripper_support_point_cloud_transformed_in_arm_hand_frame_3, best_gripper_transform);
		scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_3, orange_color_again2_3,      "gripper support3 transformed ellipsoid");
		scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support3 transformed ellipsoid");
		
		// gripper support4 ellipsoid
		//scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame_4, orange_color_again_4,                   "gripper support4 ellipsoid");
		//scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support4 ellipsoid");
		// gripper support4 transformed ellipsoid
		pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame_4, *gripper_support_point_cloud_transformed_in_arm_hand_frame_4, best_gripper_transform);
		scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_4, orange_color_again2_4,      "gripper support4 transformed ellipsoid");
		scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support4 transformed ellipsoid");
	}
	else if(gripper_model == "franka_gripper"){
		// gripper support1 ellipsoid
		//scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame_1, orange_color_again_1,                   "gripper support1 ellipsoid");
		//scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support1 ellipsoid");
		// gripper support1 transformed ellipsoid
		pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame_1, *gripper_support_point_cloud_transformed_in_arm_hand_frame_1, best_gripper_transform);
		scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_1, orange_color_again2_1,      "gripper support1 transformed ellipsoid");
		scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support1 transformed ellipsoid");
		
		// gripper support2 ellipsoid
		//scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_in_arm_hand_frame_2, orange_color_again_2,                   "gripper support2 ellipsoid");
		//scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support2 ellipsoid");
		// gripper support2 transformed ellipsoid
		pcl::transformPointCloud(*gripper_support_point_cloud_in_arm_hand_frame_2, *gripper_support_point_cloud_transformed_in_arm_hand_frame_2, best_gripper_transform);
		scene_cloud_viewer->updatePointCloud(gripper_support_point_cloud_transformed_in_arm_hand_frame_2, orange_color_again2_2,      "gripper support2 transformed ellipsoid");
		scene_cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,                   "gripper support2 transformed ellipsoid");
	}

  scene_cloud_viewer->spinOnce();
  
  
  
  // output
  std::cout << "best_gripper_transform = " << std::endl << best_gripper_transform << std::endl;
  
  end = clock();
	time_spent = (double)( end - begin )/ CLOCKS_PER_SEC;
	std::cout << "total time spent by this program = " << time_spent << std::endl;
  
	//scene_cloud_viewer->setCameraPosition(-0.666557, 0.0468138, -0.399861, -0.0378051, 0.0823057, 0.348896, 0.765311, 0.0166941, -0.643444, 0);
	scene_cloud_viewer->setCameraPosition(-0.909655, -0.134472, -0.0385448, -0.0378051, 0.0823057, 0.348896, 0.410664, -0.0225198, -0.911509, 0);
	
  //std::vector<pcl::visualization::Camera> cam;
  while ( !scene_cloud_viewer->wasStopped() ){
    scene_cloud_viewer->spinOnce();
    //if(gripper_model == "allegro_right_hand")
    //  save_file_name = "allegro_video/allegro_best.png";
    //else if(gripper_model == "franka_gripper")
    //save_file_name = "allegro_plant_pot.png";
    //save_file_name = "franka_plant_pot_best.png";
    //save_file_name = "allegro_plant_pot_best.png";
    //scene_cloud_viewer->saveScreenshot(save_file_name);
    //scene_cloud_viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  }
  return (0);
}

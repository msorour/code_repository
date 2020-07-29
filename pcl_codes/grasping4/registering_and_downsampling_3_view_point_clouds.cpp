/*
run this program using:
./step_1_downsampling_object_cloud tool_box.pcd 0.003

you can view all output segmented clusters using:
pcl_viewer tool_box_downsampled.pcd


reset && cmake .. && make && ./registering_and_downsampling_3_view_point_clouds ../raw_object_pcd_files/storage_bin_0.pcd ../raw_object_pcd_files/storage_bin_1.pcd ../raw_object_pcd_files/storage_bin_2.pcd ../raw_object_pcd_files/storage_bin_tf.txt
reset && cmake .. && make && ./registering_and_downsampling_3_view_point_clouds ../raw_object_pcd_files/dish_brush_0.pcd ../raw_object_pcd_files/dish_brush_1.pcd ../raw_object_pcd_files/dish_brush_2.pcd ../raw_object_pcd_files/dish_brush_tf.txt
reset && cmake .. && make && ./registering_and_downsampling_3_view_point_clouds ../raw_object_pcd_files/sprayer_0.pcd ../raw_object_pcd_files/sprayer_1.pcd ../raw_object_pcd_files/sprayer_2.pcd ../raw_object_pcd_files/sprayer_tf.txt
reset && cmake .. && make && ./registering_and_downsampling_3_view_point_clouds ../raw_object_pcd_files/cup_without_handle_0.pcd ../raw_object_pcd_files/cup_without_handle_1.pcd ../raw_object_pcd_files/cup_without_handle_2.pcd ../raw_object_pcd_files/cup_without_handle_tf.txt
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <fstream>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/centroid.h>
#include <pcl/Vertices.h>
#include <math.h>
#include <unistd.h>   // for sleep

#include "include/useful_implementations.h"
#include "include/grasping_algorithm.h"

int main (int argc, char** argv){
  std::string file_name1 = argv[1];
  std::string file_name2 = argv[2];
  std::string file_name3 = argv[3];
  std::string tf_matrix_file_name = argv[4];
  
  double leaf_size = 0.01;
  double distance_threshold = 0.01;
  
  clock_t begin1, begin2, begin3, begin4, end;
	double time_spent;
	
	begin1 = clock();
  
  Eigen::MatrixXd tf = readMatrix(tf_matrix_file_name.c_str());
  Eigen::Matrix4d tf1, tf2, tf3;
  tf1 << tf(0,0),  tf(0,1),  tf(0,2),  tf(0,3),
         tf(1,0),  tf(1,1),  tf(1,2),  tf(1,3),
         tf(2,0),  tf(2,1),  tf(2,2),  tf(2,3),
         tf(3,0),  tf(3,1),  tf(3,2),  tf(3,3);
  tf2 << tf(4,0),  tf(4,1),  tf(4,2),  tf(4,3),
         tf(5,0),  tf(5,1),  tf(5,2),  tf(5,3),
         tf(6,0),  tf(6,1),  tf(6,2),  tf(6,3),
         tf(7,0),  tf(7,1),  tf(7,2),  tf(7,3);
  tf3 << tf(8,0),  tf(8,1),  tf(8,2),  tf(8,3),
         tf(9,0),  tf(9,1),  tf(9,2),  tf(9,3),
         tf(10,0), tf(10,1), tf(10,2), tf(10,3),
         tf(11,0), tf(11,1), tf(11,2), tf(11,3);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3                             (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3_transformed                 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_1_transformed_downsampled     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_2_transformed_downsampled     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_3_transformed_downsampled     (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       scene_cloud_xyz_transformed_downsampled       (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       object_cloud_xyz_downsampled                  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       table_cloud_xyz_downsampled                   (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       allegro_hand_point_cloud                      (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       allegro_hand_point_cloud_downsampled          (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr       franka_gripper_point_cloud                    (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr       franka_gripper_point_cloud_downsampled        (new pcl::PointCloud<pcl::PointXYZ>);
  
  double i;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PCDWriter writer;
  
  // load the 3 view point clouds
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name1, *scene_cloud_xyz_1);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name2, *scene_cloud_xyz_2);
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name3, *scene_cloud_xyz_3);
  
  pcl::io::loadPCDFile<pcl::PointXYZ>("../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera.pcd", *allegro_hand_point_cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>("../gripper_pcd_model/franka_gripper_model_cloud_plus_camera.pcd", *franka_gripper_point_cloud);
  copyPointCloud(*allegro_hand_point_cloud, *allegro_hand_point_cloud_downsampled);
  copyPointCloud(*franka_gripper_point_cloud, *franka_gripper_point_cloud_downsampled);
  
  // downsampling gripper point cloud
  // Allegro hand
  i = 0.0;
  while(allegro_hand_point_cloud_downsampled->points.size() > 100){
    i += 0.001;
    vg.setInputCloud(allegro_hand_point_cloud);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*allegro_hand_point_cloud_downsampled);
  }
  std::cout << "allegro hand cloud after downsampling has: " << allegro_hand_point_cloud_downsampled->points.size() << " data points." << std::endl;
  // save downsampled cloud
  writer.write<pcl::PointXYZ>("../gripper_pcd_model/allegro_right_hand_model_cloud_plus_camera_downsampled_100.pcd", *allegro_hand_point_cloud_downsampled, false);
  
  // Franka gripper
  i = 0.0;
  while(franka_gripper_point_cloud_downsampled->points.size() > 100){
    i += 0.001;
    vg.setInputCloud(franka_gripper_point_cloud);
    vg.setLeafSize(leaf_size+i, leaf_size+i, leaf_size+i);
    vg.filter(*franka_gripper_point_cloud_downsampled);
  }
  std::cout << "franka gripper cloud after downsampling has: " << franka_gripper_point_cloud_downsampled->points.size() << " data points." << std::endl;
  // save downsampled cloud
  writer.write<pcl::PointXYZ>("../gripper_pcd_model/franka_gripper_model_cloud_plus_camera_downsampled_100.pcd", *franka_gripper_point_cloud_downsampled, false);
  
  end = clock();
	time_spent = (double)( end - begin1 )/ CLOCKS_PER_SEC;
	std::cout << "time spent in loading point clouds from disk = " << time_spent << std::endl;
  
  
  
  // for visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("point cloud visulaizer"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> downsampled_viewer(new pcl::visualization::PCLVisualizer ("downsampled point cloud visulaizer"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> segmentation_viewer(new pcl::visualization::PCLVisualizer ("segmented object/table point cloud visulaizer"));
  
  viewer->setCameraPosition(-0.582661, 0.49555, 0.175435, -0.129036, 0.102673, 0.469419, 0.468416, -0.114679, -0.876034, 0);
  viewer->addCoordinateSystem(0.15);
  viewer->setBackgroundColor(255,255,255);
  
  downsampled_viewer->setCameraPosition(-0.582661, 0.49555, 0.175435, -0.129036, 0.102673, 0.469419, 0.468416, -0.114679, -0.876034, 0);
  downsampled_viewer->addCoordinateSystem(0.15);
  downsampled_viewer->setBackgroundColor(255,255,255);
  
  segmentation_viewer->setCameraPosition(-0.582661, 0.49555, 0.175435, -0.129036, 0.102673, 0.469419, 0.468416, -0.114679, -0.876034, 0);
  segmentation_viewer->addCoordinateSystem(0.15);
  segmentation_viewer->setBackgroundColor(255,255,255);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> black  (scene_cloud_xyz_1_transformed, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_1_transformed, black, "scene cloud 1");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud 1");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red  (scene_cloud_xyz_2_transformed, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_2_transformed, red, "scene cloud 2");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud 2");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue  (scene_cloud_xyz_3_transformed, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_3_transformed, blue, "scene cloud 3");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene cloud 3");
  
  //
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> black2  (scene_cloud_xyz_1_transformed_downsampled, 0, 0, 0);
  downsampled_viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_1_transformed_downsampled, black2, "scene cloud 1 downsampled");
  downsampled_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "scene cloud 1 downsampled");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red2  (scene_cloud_xyz_2_transformed_downsampled, 255, 0, 0);
  downsampled_viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_2_transformed_downsampled, red2, "scene cloud 2 downsampled");
  downsampled_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "scene cloud 2 downsampled");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue2  (scene_cloud_xyz_3_transformed_downsampled, 0, 0, 255);
  downsampled_viewer->addPointCloud<pcl::PointXYZ>(scene_cloud_xyz_3_transformed_downsampled, blue2, "scene cloud 3 downsampled");
  downsampled_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "scene cloud 3 downsampled");
  
  //
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red3  (object_cloud_xyz_downsampled, 255, 0, 0);
  segmentation_viewer->addPointCloud<pcl::PointXYZ>(object_cloud_xyz_downsampled, red3, "object cloud downsampled");
  segmentation_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "object cloud downsampled");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue3  (table_cloud_xyz_downsampled, 0, 0, 255);
  segmentation_viewer->addPointCloud<pcl::PointXYZ>(table_cloud_xyz_downsampled, blue3, "table cloud downsampled");
  segmentation_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "table cloud downsampled");
  
  
  
  
  
  
  begin4 = clock();
  //registering_downsampling_segmenting_3_view_point_clouds(scene_cloud_xyz_1, tf1,   scene_cloud_xyz_2, tf2,   scene_cloud_xyz_3, tf3,
  //                                                        scene_cloud_xyz_1_transformed, scene_cloud_xyz_2_transformed, scene_cloud_xyz_3_transformed,
  //                                                        scene_cloud_xyz_1_transformed_downsampled, scene_cloud_xyz_2_transformed_downsampled, scene_cloud_xyz_3_transformed_downsampled,
  //                                                        table_cloud_xyz_downsampled, object_cloud_xyz_downsampled );
  registering_downsampling_segmenting_3_view_point_clouds(scene_cloud_xyz_1, tf1,   scene_cloud_xyz_2, tf2,   scene_cloud_xyz_3, tf3,
                                                          table_cloud_xyz_downsampled, object_cloud_xyz_downsampled );
  end = clock();
	time_spent = (double)( end - begin4 )/ CLOCKS_PER_SEC;
	std::cout << "total time spent in downsampling/segmentation = " << time_spent << std::endl;
	
	viewer->updatePointCloud(scene_cloud_xyz_1_transformed, black, "scene cloud 1");
  viewer->updatePointCloud(scene_cloud_xyz_2_transformed, red,   "scene cloud 2");
  viewer->updatePointCloud(scene_cloud_xyz_3_transformed, blue,  "scene cloud 3");
  
  downsampled_viewer->updatePointCloud(scene_cloud_xyz_1_transformed_downsampled, black2, "scene cloud 1 downsampled");
  downsampled_viewer->updatePointCloud(scene_cloud_xyz_2_transformed_downsampled, red2,   "scene cloud 2 downsampled");
  downsampled_viewer->updatePointCloud(scene_cloud_xyz_3_transformed_downsampled, blue2,  "scene cloud 3 downsampled");
  
  segmentation_viewer->updatePointCloud(table_cloud_xyz_downsampled, blue3, "table cloud downsampled");
  segmentation_viewer->updatePointCloud(object_cloud_xyz_downsampled, red3, "object cloud downsampled");
  
  
  
  
  std::vector<pcl::visualization::Camera> cam;
  while( !viewer->wasStopped() ){
    viewer->spinOnce();
    downsampled_viewer->spinOnce();
    //viewer->saveScreenshot(save_file_name);
    
    //viewer->getCameras(cam);
    //Print recorded points on the screen: 
    //cout << "Cam: " << endl 
    //             << " - pos:   (" << cam[0].pos[0]   << ", " << cam[0].pos[1] <<   ", " << cam[0].pos[2] <<   ")" << endl
    //             << " - view:  (" << cam[0].view[0]  << ", " << cam[0].view[1] <<  ", " << cam[0].view[2] <<  ")" << endl
    //             << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;
  } 
  
  return (0);
}

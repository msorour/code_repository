#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/Image.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <pcl/common/transforms.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/AllegroRightHandModel.h"
#include "../../../include/FrankaPandaArmModel.h"

#include <iostream>
#include <fstream>
#include <time.h>


typedef pcl::PointCloud<pcl::PointXYZ>    PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
boost::shared_ptr<pcl::visualization::PCLVisualizer> realsense_viewer (new pcl::visualization::PCLVisualizer("realsense cloud"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(augmented_cloud);


void view_point_cloud( const PointCloudXYZRGB::ConstPtr& msg ){
  *augmented_cloud = *msg;
  realsense_viewer->updatePointCloud<pcl::PointXYZRGB>(augmented_cloud, rgb, "realsense cloud");
}


int main(int argc, char **argv){
  std::string model_name;
  model_name = argv[2];
  
  ros::init(argc, argv, "point_cloud_processing");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
  
  realsense_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "realsense cloud");
  realsense_viewer->setBackgroundColor(255,255,255);
  realsense_viewer->addCoordinateSystem(0.2);
  
  realsense_viewer->addPointCloud(augmented_cloud, rgb,"realsense cloud");
  ros::Subscriber sub = n.subscribe<PointCloudXYZRGB>("/camera/depth/color/points", 1, view_point_cloud);
  
  
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < 2 ){
		ros::spinOnce();
    loop_rate.sleep();
  }
  
 	double time_now = ros::Time::now().toSec();
 	
  while (ros::ok() and !realsense_viewer->wasStopped()){
  	time_now = ros::Time::now().toSec();
  	//cout << "time_now = " << time_now  << endl;
    realsense_viewer->spinOnce();
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


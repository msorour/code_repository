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
#include "../include/franka_panda_arm_plus_allegro_right_hand.h"

#include <iostream>
#include <fstream>
#include <time.h>

using namespace franka_panda_gazebo_controller;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud viewer"));


class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  public:
    std::string window_name="";
    ImageConverter(std::string image_topic_name) : it_(nh_){
      // Subscrive to input video feed
      image_sub_ = it_.subscribe(image_topic_name, 1, &ImageConverter::imageCb, this);
      cv::namedWindow(window_name);
    }
    
    ~ImageConverter(){cv::destroyWindow(window_name);}

    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      try{cv_ptr = cv_bridge::toCvCopy(msg);}
      catch(cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what()); return;}
      
      // Update GUI Window
      cv::imshow(window_name, cv_ptr->image);
      cv::waitKey(3);
    }
};


void callback(const PointCloud::ConstPtr& msg){  
  float gama = M_PI; // The angle of rotation in radians
  
  // Using a Affine3f
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 3.0 meters on the z axis.
  transform.translation() << 0.0, 0.0, 0.0;

  // Define rotation matrix
  transform.rotate(Eigen::AngleAxisf(gama, Eigen::Vector3f::UnitX()));

  // Print the transformation
  //printf ("\nUsing an Affine3f\n");
  //std::cout << transform.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*msg, *transformed_cloud, transform);
  
  //if(!viewer->addPointCloud(msg,"sample cloud"))
  //  viewer->updatePointCloud<pcl::PointXYZ>(msg, "sample cloud");
  if(!viewer->addPointCloud(transformed_cloud,"sample cloud"))
    viewer->updatePointCloud<pcl::PointXYZ>(transformed_cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
}



/*
class PointCloudToImage{
  public:
    void
    cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud){
      if((cloud->width * cloud->height) == 0)
        return; //return if the cloud is not dense!
      try{pcl::toROSMsg(*cloud, image_);} //convert the cloud
      catch(std::runtime_error e){ROS_ERROR_STREAM("Error in converting cloud to image message: "<< e.what());}
      image_pub_.publish (image_); //publish our cloud image
    }
    
    PointCloudToImage(std::string cloud_topic_, std::string image_topic_){
      sub_ = nh_.subscribe(cloud_topic_, 30, &PointCloudToImage::cloud_cb, this);
      image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 30);
    }
  private:
    ros::NodeHandle nh_;
    sensor_msgs::Image image_;  //cache the image message
    ros::Subscriber sub_;       //cloud subscriber
    ros::Publisher image_pub_;  //image message publisher
};
*/


















int main(int argc, char **argv){
  std::string model_name;
  model_name = argv[2];
  
  ros::init(argc, argv, "workcell_high_level_control");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
  
  ros::Subscriber sub = n.subscribe<PointCloud>("/master_cell/kinect2/sd/points", 1, callback);
  
  
  // Image !!
 	ImageConverter real_sense_depth_image("/master_cell/realsense/depth/image_raw");
  real_sense_depth_image.window_name = "realsense depth image";
  /*
  ImageConverter real_sense_ir1_image("/master_cell/realsense/ir1/image_raw");
  real_sense_ir1_image.window_name = "realsense ir1 image";
  
  ImageConverter real_sense_ir2_image("/master_cell/realsense/ir2/image_raw");
  real_sense_ir2_image.window_name = "realsense ir2 image";
  */
  ImageConverter kinect2_depth_image("/master_cell/kinect2/sd/image_depth_rect");
  kinect2_depth_image.window_name = "kinect2 depth image";
  
  /*
  ImageConverter kinect2_depth_image_comp("/master_cell/kinect2/sd/image_depth_rect/compressed");
  kinect2_depth_image_comp.window_name = "kinect2 depth image comp";
  
  ImageConverter kinect2_ir_image("/master_cell/kinect2/sd/image_ir_rect");
  kinect2_ir_image.window_name = "kinect2 ir image comp";
  */
  
  
  
  
  
  
  
  ImageConverter kinect2_point_cloud_image("/master_cell/point_cloud_image");
  kinect2_point_cloud_image.window_name = "kinect2 point cloud image";
  
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay*10 ){
		ros::spinOnce();
    loop_rate.sleep();
  }
  
  
 	
 	
 	
 	
 	
 	time_now = ros::Time::now().toSec();
 	time_past = time_now;
	
	//while (ros::ok() and ros::Time::now().toSec()<20 ){
  while (ros::ok() and !viewer->wasStopped()){
  	time_now = ros::Time::now().toSec();
  	//cout << "time_now = " << time_now  << endl;
  	
  	
  	
    viewer->spinOnce();
  	
  	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}








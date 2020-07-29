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
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud viewer"));
Eigen::Matrix3f intrinsic_parameters;

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string window_name="dummy";
	
  public:
    ImageConverter(std::string image_topic_name, std::string name="") : it_(nh_){
      // Subscrive to input video feed
      image_sub_ = it_.subscribe(image_topic_name, 1, &ImageConverter::imageCb, this);
      window_name = name;
      cv::namedWindow(window_name);
    }
    
    ~ImageConverter(){cv::destroyWindow(window_name);}

    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      try{cv_ptr = cv_bridge::toCvCopy(msg);}
      catch(cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what()); return;}
      
      // depth image to point cloud
      intrinsic_parameters << 0.0, 0.0, 0.0,
			                        0.0, 0.0, 0.0,
			                        0.0, 0.0, 0.0;

      //point_cloud = RGBDtoPCL(cv_ptr->image, intrinsic_parameters);
      
      // Update GUI Window
      cv::imshow(window_name, cv_ptr->image);
      cv::waitKey(3);
    }
};


/*
PointCloud::Ptr RGBDtoPCL(cv::Mat depth_image, Eigen::Matrix3f& _intrinsics){
	PointCloud::Ptr pointcloud(new PointCloud);
	float fx = _intrinsics(0, 0);
	float fy = _intrinsics(1, 1);
	float cx = _intrinsics(0, 2);
	float cy = _intrinsics(1, 2);
	float factor = 1;
	
	depth_image.convertTo(depth_image, CV_32F); // convert the image data to float type 
	
	if(!depth_image.data){std::cerr << "No depth data!!!" << std::endl; exit(EXIT_FAILURE);}

	pointcloud->width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing 
	pointcloud->height = depth_image.rows;
	pointcloud->resize(pointcloud->width*pointcloud->height);

#pragma omp parallel for
	for (int v = 0; v < depth_image.rows; v += 4){
		for (int u = 0; u < depth_image.cols; u += 4){
			float Z = depth_image.at<float>(v, u) / factor;
			pcl::PointXYZ p;
			p.z = Z;
			p.x = (u - cx) * Z / fx;
			p.y = (v - cy) * Z / fy;
			p.z = p.z / 1000;
			p.x = p.x / 1000;
			p.y = p.y / 1000;
			pointcloud->points.push_back(p);
		}
	}
	return pointcloud;
}
*/


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
  
    
  //ros::Subscriber sub = n.subscribe<PointCloud>("/master_cell/kinect2/sd/points", 1, callback);
  //ros::Subscriber sub = n.subscribe<PointCloud>("/master_cell/realsense/depth/image_raw", 1, callback);
  //ros::Subscriber sub = n.subscribe<PointCloud>("/master_cell/panda1_link7/panda1_realsense_depth/image", 1, callback);
  ros::Subscriber sub = n.subscribe<PointCloud>("/panda1/depth/points", 1, callback);
  
  //ImageConverter real_sense_depth_image("/master_cell/realsense/depth/image_raw");
  //real_sense_depth_image.window_name = "realsense depth image";
  ImageConverter real_sense_depth_image("/master_cell/realsense/depth/image_raw", "realsense depth image");
  //ImageConverter real_sense_depth_image("/master_cell/panda1_link7/panda1_realsense_depth/image", "panda1 realsense depth image");
  
  
  /*
  ImageConverter real_sense_ir1_image("/master_cell/realsense/ir1/image_raw");
  real_sense_ir1_image.window_name = "realsense ir1 image";
  
  ImageConverter real_sense_ir2_image("/master_cell/realsense/ir2/image_raw");
  real_sense_ir2_image.window_name = "realsense ir2 image";
  
  ImageConverter kinect2_depth_image("/master_cell/kinect2/sd/image_depth_rect");
  kinect2_depth_image.window_name = "kinect2 depth image";
  */
  
  /*
  ImageConverter kinect2_depth_image_comp("/master_cell/kinect2/sd/image_depth_rect/compressed");
  kinect2_depth_image_comp.window_name = "kinect2 depth image comp";
  
  ImageConverter kinect2_ir_image("/master_cell/kinect2/sd/image_ir_rect");
  kinect2_ir_image.window_name = "kinect2 ir image comp";
  */
  
  
  
  
  
  
  /*
  ImageConverter kinect2_point_cloud_image("/master_cell/point_cloud_image");
  kinect2_point_cloud_image.window_name = "kinect2 point cloud image";
  */
	
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
  	
  	if(!viewer->addPointCloud(point_cloud,"sample cloud"))
			viewer->updatePointCloud<pcl::PointXYZ>(point_cloud, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  	
    viewer->spinOnce();
  	
  	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}








#include "RealSenseRosPlugin.hh"
#include <sensor_msgs/fill_image.h>

#define DEPTH_PUB_FREQ_HZ 30
#define COLOR_PUB_FREQ_HZ 30
#define IRED1_PUB_FREQ_HZ 30
#define IRED2_PUB_FREQ_HZ 30

#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "stereo_right_imager"
#define IRED2_CAMERA_NAME "stereo_left_imager"

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "stereo_right"
#define IRED2_CAMERA_TOPIC "stereo_left"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

namespace{sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image);}

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RealSenseCamRosPlugin)

/////////////////////////////////////////////////
RealSenseCamRosPlugin::RealSenseCamRosPlugin(){
  this->depthCam = nullptr;
  this->ired1Cam = nullptr;
  this->ired2Cam = nullptr;
  this->colorCam = nullptr;
}

/////////////////////////////////////////////////
RealSenseCamRosPlugin::~RealSenseCamRosPlugin(){}

/////////////////////////////////////////////////
void RealSenseCamRosPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Output the name of the model
  std::cout << std::endl << "RealSenseCamRosPlugin: The realsense_camera plugin is attached to model " << _model->GetName() << std::endl;
  
  std::string model_name = _model->GetName();
  int sensor_count = _model->GetSensorCount();
  std::cerr << "RealSenseCamRosPlugin: model_name: "<< model_name << "\n";
  std::cerr << "RealSenseCamRosPlugin: sensor_count: "<< sensor_count << "\n";
  
  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();
  
  this->sensor = smanager->GetSensors();
  
  // Get sensor names
  for(int k=0; k<sensor_count; k++){
  	std::cerr << "RealSenseCamRosPlugin: Sensor"<<k<< ": " << this->sensor[k]->Name() << "\n";
  	if(this->sensor[k]->Name().find("realsense") != std::string::npos)
  	  this->realsense_sensor.push_back(this->sensor[k]->Name());
  }
  std::cerr << "\n";
  for(int k=0; k<sensor_count; k++)
  	std::cerr << "RealSenseCamRosPlugin: Sensor"<<k<< ": " << this->realsense_sensor[k] << "\n";
  std::cerr << "\n";
  // Get Cameras Renderers
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(smanager->GetSensor(this->realsense_sensor[3]))->DepthCamera();
  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(this->realsense_sensor[2]))->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(this->realsense_sensor[1]))->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(this->realsense_sensor[0]))->Camera();
  /*
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(smanager->GetSensor(DEPTH_CAMERA_NAME))->DepthCamera();
  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(IRED1_CAMERA_NAME))->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(IRED2_CAMERA_NAME))->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(COLOR_CAMERA_NAME))->Camera();
  */
  // Check if camera renderers have been found successfuly
  if (!this->depthCam){std::cerr << "RealSenseCamRosPlugin: Depth Camera has not been found"      << std::endl; return;}
  if (!this->ired1Cam){std::cerr << "RealSenseCamRosPlugin: InfraRed Camera 1 has not been found" << std::endl; return;}
  if (!this->ired2Cam){std::cerr << "RealSenseCamRosPlugin: InfraRed Camera 2 has not been found" << std::endl; return;}
  if (!this->colorCam){std::cerr << "RealSenseCamRosPlugin: Color Camera has not been found"      << std::endl; return;}

  // Resize Depth Map dimensions
  try{this->depthMap.resize(this->depthCam->ImageWidth() * this->depthCam->ImageHeight());}
  catch(std::bad_alloc &e){std::cerr << "RealSenseCamRosPlugin: depthMap allocation failed: " << e.what() << std::endl; return;}

  // Setup Transport Node
  this->transportNode = transport::NodePtr(new transport::Node());
  //this->transportNode->Init(this->world->GetName());    //Gazebo7
  this->transportNode->Init(this->world->Name());         //Gazebo8

  // Setup Publishers
  std::string rsTopicRoot = "~/" + model_name + "/realsense/stream/";

  this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + IRED1_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + IRED2_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + COLOR_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);

  // Listen to depth camera new frame event
  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(std::bind(&RealSenseCamRosPlugin::OnNewDepthFrame, this));
  this->newIred1FrameConn = this->ired1Cam->ConnectNewImageFrame(std::bind(&RealSenseCamRosPlugin::OnNewFrame, this, this->ired1Cam, this->ired1Pub));
  this->newIred2FrameConn = this->ired2Cam->ConnectNewImageFrame(std::bind(&RealSenseCamRosPlugin::OnNewFrame, this, this->ired2Cam, this->ired2Pub));
  this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(&RealSenseCamRosPlugin::OnNewFrame, this, this->colorCam, this->colorPub));
  
  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RealSenseCamRosPlugin::OnUpdate, this));
  
  
  // ROS
  if (!ros::isInitialized()){
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");
  
  std::string rosTopicRoot = "/" + model_name;
  this->rosnode_ = new ros::NodeHandle(rosTopicRoot + "/realsense");

  // initialize camera_info_manager
  this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(*this->rosnode_, "realsensecam"));
  
  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);
  
  this->color_pub_ = this->itnode_->advertiseCamera(rosTopicRoot+"/realsense/color/image_raw", 2);
  this->ir1_pub_ = this->itnode_->advertiseCamera(rosTopicRoot+"/realsense/ir/image_raw", 2);
  this->ir2_pub_ = this->itnode_->advertiseCamera(rosTopicRoot+"/realsense/ir2/image_raw", 2);
  this->depth_pub_ = this->itnode_->advertiseCamera(rosTopicRoot+"/realsense/depth/image_raw", 2);

}

/////////////////////////////////////////////////
void RealSenseCamRosPlugin::OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub){
  msgs::ImageStamped msg;

  // Set Simulation Time
  //msgs::Set(msg.mutable_time(), this->world->GetSimTime());       // Gazebo7
  msgs::Set(msg.mutable_time(), this->world->SimTime().Double());   // Gazebo8

  // Set Image Dimensions
  msg.mutable_image()->set_width(cam->ImageWidth());
  msg.mutable_image()->set_height(cam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(cam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
  msg.mutable_image()->set_data(cam->ImageData(), cam->ImageDepth() * cam->ImageWidth() * cam->ImageHeight());

  // Publish realsense infrared stream
  pub->Publish(msg);
  
  
  // ROS
  //common::Time current_time = this->world->GetSimTime();       // Gazebo7
  common::Time current_time = this->world->SimTime().Double();   // Gazebo8

  // identify camera
  std::string camera_id = cam->Name();
  image_transport::CameraPublisher* image_pub;
  if      (camera_id.find(COLOR_CAMERA_NAME) != std::string::npos){camera_id = COLOR_CAMERA_NAME; image_pub = &(this->color_pub_);}
  else if (camera_id.find(IRED1_CAMERA_NAME) != std::string::npos){camera_id = IRED1_CAMERA_NAME; image_pub = &(this->ir1_pub_);}
  else if (camera_id.find(IRED2_CAMERA_NAME) != std::string::npos){camera_id = IRED2_CAMERA_NAME; image_pub = &(this->ir2_pub_);}
  else{ROS_ERROR("Unknown camera name\n"); camera_id = COLOR_CAMERA_NAME; image_pub = &(this->color_pub_);}

  // copy data into image
  this->image_msg_.header.frame_id = camera_id;
  this->image_msg_.header.stamp.sec = current_time.sec;
  this->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = cam->ImageFormat();
  if (pixel_format == "L_INT8"){pixel_format = sensor_msgs::image_encodings::MONO8;}
  else if (pixel_format == "RGB_INT8"){pixel_format = sensor_msgs::image_encodings::RGB8;}
  else{ROS_ERROR("Unsupported Gazebo ImageFormat\n"); pixel_format = sensor_msgs::image_encodings::BGR8;}

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_, pixel_format, cam->ImageHeight(), cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(), reinterpret_cast<const void*>(cam->ImageData()));

  sensor_msgs::CameraInfo cam_info_msg = cameraInfo(this->image_msg_);

  // publish to ROS
  image_pub->publish(this->image_msg_, cam_info_msg);
  
}

/////////////////////////////////////////////////
void RealSenseCamRosPlugin::OnNewDepthFrame(){
  // Get Depth Map dimensions
  unsigned int imageSize = this->depthCam->ImageWidth() * this->depthCam->ImageHeight();

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i)
  {
    // Check clipping and overflow
    if (depthDataFloat[i] < DEPTH_NEAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_FAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0)
    {
      this->depthMap[i] = 0;
    }
    else
    {
      this->depthMap[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  //msgs::Set(msg.mutable_time(), this->world->GetSimTime());       // Gazebo7
  msgs::Set(msg.mutable_time(), this->world->SimTime().Double());   // Gazebo8
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(
      this->depthMap.data(),
      sizeof(*this->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
  
  
  // ROS
  // get current time
  //common::Time current_time = this->world->GetSimTime();       // Gazebo7
  common::Time current_time = this->world->SimTime().Double();   // Gazebo8

  // copy data into image
  this->depth_msg_.header.frame_id = COLOR_CAMERA_NAME;
  this->depth_msg_.header.stamp.sec = current_time.sec;
  this->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_,
    pixel_format,
    this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
    2 * this->depthCam->ImageWidth(),
    reinterpret_cast<const void*>(this->depthMap.data()));

  sensor_msgs::CameraInfo depth_info_msg = cameraInfo(this->depth_msg_);

  // publish to ROS
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
  
}

/////////////////////////////////////////////////
void RealSenseCamRosPlugin::OnUpdate(){}


namespace{
  sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image){
    sensor_msgs::CameraInfo info_msg;

    info_msg.header = image.header;
    info_msg.height = image.height;
    info_msg.width = image.width;

    float focal = 463.889;

    info_msg.K[0] = focal;
    info_msg.K[4] = focal;
    info_msg.K[2] = info_msg.width * 0.5;
    info_msg.K[5] = info_msg.height * 0.5;
    info_msg.K[8] = 1.;

    info_msg.P[0] = info_msg.K[0];
    info_msg.P[5] = info_msg.K[4];
    info_msg.P[2] = info_msg.K[2];
    info_msg.P[6] = info_msg.K[5];
    info_msg.P[10] = info_msg.K[8];

    return info_msg;
  }
}

#include "RealSensePlugin.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

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

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(RealSenseCamPlugin)

namespace gazebo{
  struct RealSensePluginPrivate{
    /// \brief Pointer to the model containing the plugin.
    public: physics::ModelPtr rsModel;

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief Pointer to the Depth Camera Renderer.
    public: rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    public: rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Camera Renderer.
    public: rendering::CameraPtr ired1Cam;

    /// \brief Pointer to the Infrared2 Camera Renderer.
    public: rendering::CameraPtr ired2Cam;

    /// \brief Pointer to the transport Node.
    public: transport::NodePtr transportNode;

    // \brief Store Real Sense depth map data.
    public: std::vector<uint16_t> depthMap;

    /// \brief Pointer to the Depth Publisher.
    public: transport::PublisherPtr depthPub;

    /// \brief Pointer to the Color Publisher.
    public: transport::PublisherPtr colorPub;

    /// \brief Pointer to the Infrared Publisher.
    public: transport::PublisherPtr ired1Pub;

    /// \brief Pointer to the Infrared2 Publisher.
    public: transport::PublisherPtr ired2Pub;

    /// \brief Pointer to the Depth Camera callback connection.
    public: event::ConnectionPtr newDepthFrameConn;

    /// \brief Pointer to the Depth Camera callback connection.
    public: event::ConnectionPtr newIred1FrameConn;

    /// \brief Pointer to the Infrared Camera callback connection.
    public: event::ConnectionPtr newIred2FrameConn;

    /// \brief Pointer to the Color Camera callback connection.
    public: event::ConnectionPtr newColorFrameConn;

    /// \brief Pointer to the World Update event connection.
    public: event::ConnectionPtr updateConnection;
    
    
    
    
    public: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    /// \brief A pointer to the ROS node.
    ///  A node will be instantiated if it does not exist.
    public: ros::NodeHandle* rosnode_;
    public: image_transport::ImageTransport* itnode_;
    public: image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

    /// \brief ROS image messages
    public: sensor_msgs::Image image_msg_, depth_msg_;
  };
}

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

/////////////////////////////////////////////////
RealSenseCamPlugin::RealSenseCamPlugin() : dataPtr(new RealSensePluginPrivate){
  this->dataPtr->depthCam = nullptr;
  this->dataPtr->ired1Cam = nullptr;
  this->dataPtr->ired2Cam = nullptr;
  this->dataPtr->colorCam = nullptr;
}

/////////////////////////////////////////////////
RealSenseCamPlugin::~RealSenseCamPlugin(){}

/////////////////////////////////////////////////
void RealSenseCamPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/){
  // Output the name of the model
  std::string model_name = _model->GetName();
  std::cout << std::endl << "RealSenseCamPlugin: The rs_camera plugin is attach to model " << model_name << std::endl;

  // Store a pointer to the this model
  this->dataPtr->rsModel = _model;

  // Store a pointer to the world
  this->dataPtr->world = this->dataPtr->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->dataPtr->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(smanager->GetSensor(DEPTH_CAMERA_NAME))->DepthCamera();
  this->dataPtr->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(IRED1_CAMERA_NAME))->Camera();
  this->dataPtr->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(IRED2_CAMERA_NAME))->Camera();
  this->dataPtr->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>     (smanager->GetSensor(COLOR_CAMERA_NAME))->Camera();

  // Check if camera renderers have been found successfuly
  if(!this->dataPtr->depthCam){std::cerr << "RealSenseCamPlugin: Depth Camera has not been found"      << std::endl; return;}
  if(!this->dataPtr->ired1Cam){std::cerr << "RealSenseCamPlugin: InfraRed Camera 1 has not been found" << std::endl; return;}
  if(!this->dataPtr->ired2Cam){std::cerr << "RealSenseCamPlugin: InfraRed Camera 2 has not been found" << std::endl; return;}
  if(!this->dataPtr->colorCam){std::cerr << "RealSenseCamPlugin: Color Camera has not been found"      << std::endl; return;}

  // Setup Transport Node
  this->dataPtr->transportNode = transport::NodePtr(new transport::Node());
  this->dataPtr->transportNode->Init(this->dataPtr->world->Name());

  // Setup Publishers
  std::string rsTopicRoot = "~/" + this->dataPtr->rsModel->GetName() + "/rs/stream/";
  this->dataPtr->depthPub = this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->dataPtr->ired1Pub = this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + IRED1_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->dataPtr->ired2Pub = this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + IRED2_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->dataPtr->colorPub = this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + COLOR_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);

  // Listen to depth camera new frame event
  this->dataPtr->newDepthFrameConn = this->dataPtr->depthCam->ConnectNewDepthFrame(std::bind(&RealSenseCamPlugin::OnNewDepthFrame, this));
  this->dataPtr->newIred1FrameConn = this->dataPtr->ired1Cam->ConnectNewImageFrame(std::bind(&RealSenseCamPlugin::OnNewFrame, this, this->dataPtr->ired1Cam, this->dataPtr->ired1Pub));
  this->dataPtr->newIred2FrameConn = this->dataPtr->ired2Cam->ConnectNewImageFrame(std::bind(&RealSenseCamPlugin::OnNewFrame, this, this->dataPtr->ired2Cam, this->dataPtr->ired2Pub));
  this->dataPtr->newColorFrameConn = this->dataPtr->colorCam->ConnectNewImageFrame(std::bind(&RealSenseCamPlugin::OnNewFrame, this, this->dataPtr->colorCam, this->dataPtr->colorPub));

  // Listen to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RealSenseCamPlugin::OnUpdate, this));
  
  /*
  // ROS RELATED
  // Initialize ROS, if it has not already been initialized.
	if (!ros::isInitialized()){
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "realsense_gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("realsense_gazebo_client"));
	
	// Create topics to publish joint state.
	rgb_image_pub   = this->rosNode->advertise<sensor_msgs::Image>("/"+model_name+"/realsense/rgb_image", 10);
	depth_image_pub = this->rosNode->advertise<sensor_msgs::Image>("/"+model_name+"/realsense/depth_image", 10);
	right_image_pub = this->rosNode->advertise<sensor_msgs::Image>("/"+model_name+"/realsense/right_image"  , 10);
	left_image_pub  = this->rosNode->advertise<sensor_msgs::Image>("/"+model_name+"/realsense/left_image"  , 10);
	
	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&RealSenseCamPlugin::QueueThread, this));
	*/
	
	// Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  this->dataPtr->rosnode_ = new ros::NodeHandle("/realsense");

	// initialize camera_info_manager
  this->dataPtr->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(*this->dataPtr->rosnode_, "realsense"));

  this->dataPtr->itnode_ = new image_transport::ImageTransport(*this->dataPtr->rosnode_);

  this->dataPtr->color_pub_ = this->dataPtr->itnode_->advertiseCamera("camera/color/image_raw", 2);
  this->dataPtr->ir1_pub_ = this->dataPtr->itnode_->advertiseCamera("camera/ir/image_raw", 2);
  this->dataPtr->ir2_pub_ = this->dataPtr->itnode_->advertiseCamera("camera/ir2/image_raw", 2);
  this->dataPtr->depth_pub_ = this->dataPtr->itnode_->advertiseCamera("camera/depth/image_raw", 2);
}

/////////////////////////////////////////////////
void RealSenseCamPlugin::OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) const{
  msgs::ImageStamped msg;

  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->dataPtr->world->SimTime());

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
  common::Time current_time = this->dataPtr->world->GetSimTime();

  // identify camera
  std::string camera_id = cam->Name();
  image_transport::CameraPublisher* image_pub;
  if (camera_id.find(COLOR_CAMERA_NAME) != std::string::npos)
  {
    camera_id = COLOR_CAMERA_NAME;
    image_pub = &(this->dataPtr->color_pub_);
  }
  else if (camera_id.find(IRED1_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED1_CAMERA_NAME;
    image_pub = &(this->dataPtr->ir1_pub_);
  }
  else if (camera_id.find(IRED2_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED2_CAMERA_NAME;
    image_pub = &(this->dataPtr->ir2_pub_);
  }
  else
  {
    ROS_ERROR("Unknown camera name\n");
    camera_id = COLOR_CAMERA_NAME;
    image_pub = &(this->dataPtr->color_pub_);
  }

  // copy data into image
  this->dataPtr->image_msg_.header.frame_id = camera_id;
  this->dataPtr->image_msg_.header.stamp.sec = current_time.sec;
  this->dataPtr->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = cam->ImageFormat();
  if (pixel_format == "L_INT8")
  {
    pixel_format = sensor_msgs::image_encodings::MONO8;
  }
  else if (pixel_format == "RGB_INT8")
  {
    pixel_format = sensor_msgs::image_encodings::RGB8;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    pixel_format = sensor_msgs::image_encodings::BGR8;
  }

  // copy from simulation image to ROS msg
  fillImage(this->dataPtr->image_msg_, pixel_format, cam->ImageHeight(), cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(), reinterpret_cast<const void*>(cam->ImageData()));
  sensor_msgs::CameraInfo cam_info_msg = cameraInfo(this->dataPtr->image_msg_);

  // publish to ROS
  image_pub->publish(this->dataPtr->image_msg_, cam_info_msg);
}

/////////////////////////////////////////////////
void RealSenseCamPlugin::OnNewDepthFrame() const
{
  // Get Depth Map dimensions
  unsigned int imageSize = this->dataPtr->depthCam->ImageWidth() * this->dataPtr->depthCam->ImageHeight();

  // Check if depthMap size is equivalent to imageSize
  if (this->dataPtr->depthMap.size() != imageSize){
    try{
      this->dataPtr->depthMap.resize(imageSize);
    }
    catch (std::bad_alloc &e){
      std::cerr << "RealSenseCamPlugin: depthMap allocation failed: " << e.what() << std::endl;
      return;
    }
  }

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->dataPtr->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i){
    // Check clipping and overflow
    if (depthDataFloat[i] < DEPTH_NEAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_FAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0){
      this->dataPtr->depthMap[i] = 0;
    }
    else{
      this->dataPtr->depthMap[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  msgs::Set(msg.mutable_time(), this->dataPtr->world->SimTime());
  msg.mutable_image()->set_width(this->dataPtr->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->dataPtr->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->dataPtr->depthCam->ImageWidth() * this->dataPtr->depthCam->ImageDepth());
  msg.mutable_image()->set_data(this->dataPtr->depthMap.data(), sizeof(*this->dataPtr->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->dataPtr->depthPub->Publish(msg);
  
  
  // get current time
  common::Time current_time = this->dataPtr->world->GetSimTime();

  // copy data into image
  this->dataPtr->depth_msg_.header.frame_id = COLOR_CAMERA_NAME;
  this->dataPtr->depth_msg_.header.stamp.sec = current_time.sec;
  this->dataPtr->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->dataPtr->depth_msg_,
    pixel_format,
    this->dataPtr->depthCam->ImageHeight(), this->dataPtr->depthCam->ImageWidth(),
    2 * this->dataPtr->depthCam->ImageWidth(),
    reinterpret_cast<const void*>(this->dataPtr->depthMap.data()));

  sensor_msgs::CameraInfo depth_info_msg = cameraInfo(this->dataPtr->depth_msg_);

  // publish to ROS
  this->dataPtr->depth_pub_.publish(this->dataPtr->depth_msg_, depth_info_msg);
}

/////////////////////////////////////////////////
void RealSenseCamPlugin::OnUpdate(){}



/*
// ROS helper function that processes messages
void RealSenseCamPlugin::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok())
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
}*/

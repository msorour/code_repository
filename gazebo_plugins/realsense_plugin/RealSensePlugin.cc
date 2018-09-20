/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

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
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(RealSensePlugin)

namespace gazebo
{
  struct RealSensePluginPrivate
  {
    /// \brief Pointer to the model containing the plugin.
    public:
    physics::ModelPtr rsModel;

    /// \brief Pointer to the world.
    public:
    physics::WorldPtr world;

    /// \brief Pointer to the Depth Camera Renderer.
    public:
    rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    public:
    rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Camera Renderer.
    public:
    rendering::CameraPtr ired1Cam;

    /// \brief Pointer to the Infrared2 Camera Renderer.
    public:
    rendering::CameraPtr ired2Cam;

    /// \brief Pointer to the transport Node.
    public:
    transport::NodePtr transportNode;

    // \brief Store Real Sense depth map data.
    public:
    std::vector<uint16_t> depthMap;

    /// \brief Pointer to the Depth Publisher.
    public:
    transport::PublisherPtr depthPub;

    /// \brief Pointer to the Color Publisher.
    public:
    transport::PublisherPtr colorPub;

    /// \brief Pointer to the Infrared Publisher.
    public:
    transport::PublisherPtr ired1Pub;

    /// \brief Pointer to the Infrared2 Publisher.
    public:
    transport::PublisherPtr ired2Pub;

    /// \brief Pointer to the Depth Camera callback connection.
    public:
    event::ConnectionPtr newDepthFrameConn;

    /// \brief Pointer to the Depth Camera callback connection.
    public:
    event::ConnectionPtr newIred1FrameConn;

    /// \brief Pointer to the Infrared Camera callback connection.
    public:
    event::ConnectionPtr newIred2FrameConn;

    /// \brief Pointer to the Color Camera callback connection.
    public:
    event::ConnectionPtr newColorFrameConn;

    /// \brief Pointer to the World Update event connection.
    public:
    event::ConnectionPtr updateConnection;
  };
}

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin()
    : dataPtr(new RealSensePluginPrivate)
{
  this->dataPtr->depthCam = nullptr;
  this->dataPtr->ired1Cam = nullptr;
  this->dataPtr->ired2Cam = nullptr;
  this->dataPtr->colorCam = nullptr;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin()
{
}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  // Output the name of the model
  std::cout << std::endl
            << "RealSensePlugin: The rs_camera plugin is attach to model "
            << _model->GetName() << std::endl;

  // Store a pointer to the this model
  this->dataPtr->rsModel = _model;
std::cout <<"Debug 1!!!!!!!!!!!!!!!"<<std::endl;
  // Store a pointer to the world
  this->dataPtr->world = this->dataPtr->rsModel->GetWorld();
std::cout <<"Debug 2!!!!!!!!!!!!!!!"<<std::endl;
  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();
std::cout <<"Debug 3!!!!!!!!!!!!!!!"<<std::endl;
  // Get Cameras Renderers
  this->dataPtr->depthCam =
      std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
          smanager->GetSensor(DEPTH_CAMERA_NAME))->DepthCamera();
std::cout <<"Debug 4!!!!!!!!!!!!!!!"<<std::endl;          
  this->dataPtr->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                                smanager->GetSensor(IRED1_CAMERA_NAME))
                                ->Camera();
std::cout <<"Debug 5!!!!!!!!!!!!!!!"<<std::endl;                                
  this->dataPtr->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                                smanager->GetSensor(IRED2_CAMERA_NAME))
                                ->Camera();
std::cout <<"Debug 6!!!!!!!!!!!!!!!"<<std::endl;                                
  this->dataPtr->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                                smanager->GetSensor(COLOR_CAMERA_NAME))
                                ->Camera();
std::cout <<"Debug 7!!!!!!!!!!!!!!!"<<std::endl;
  // Check if camera renderers have been found successfuly
  if (!this->dataPtr->depthCam)
  {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }
  if (!this->dataPtr->ired1Cam)
  {
    std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              << std::endl;
    return;
  }
  if (!this->dataPtr->ired2Cam)
  {
    std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              << std::endl;
    return;
  }
  if (!this->dataPtr->colorCam)
  {
    std::cerr << "RealSensePlugin: Color Camera has not been found"
              << std::endl;
    return;
  }
std::cout <<"Debug 8!!!!!!!!!!!!!!!"<<std::endl;
  // Setup Transport Node
  this->dataPtr->transportNode = transport::NodePtr(new transport::Node());
  this->dataPtr->transportNode->Init(this->dataPtr->world->GetName());
std::cout <<"Debug 9!!!!!!!!!!!!!!!"<<std::endl;
  // Setup Publishers
  std::string rsTopicRoot =
      "~/" + this->dataPtr->rsModel->GetName() + "/rs/stream/";
std::cout <<"Debug 10!!!!!!!!!!!!!!!"<<std::endl;
  this->dataPtr->depthPub =
      this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->dataPtr->ired1Pub =
      this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + IRED1_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->dataPtr->ired2Pub =
      this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + IRED2_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->dataPtr->colorPub =
      this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + COLOR_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
std::cout <<"Debug 11!!!!!!!!!!!!!!!"<<std::endl;
  // Listen to depth camera new frame event
  this->dataPtr->newDepthFrameConn = this->dataPtr->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this));
std::cout <<"Debug 12!!!!!!!!!!!!!!!"<<std::endl;
  this->dataPtr->newIred1FrameConn =
      this->dataPtr->ired1Cam->ConnectNewImageFrame(
          std::bind(&RealSensePlugin::OnNewFrame, this, this->dataPtr->ired1Cam,
                    this->dataPtr->ired1Pub));
std::cout <<"Debug 13!!!!!!!!!!!!!!!"<<std::endl;
  this->dataPtr->newIred2FrameConn =
      this->dataPtr->ired2Cam->ConnectNewImageFrame(
          std::bind(&RealSensePlugin::OnNewFrame, this, this->dataPtr->ired2Cam,
                    this->dataPtr->ired2Pub));
std::cout <<"Debug 14!!!!!!!!!!!!!!!"<<std::endl;
  this->dataPtr->newColorFrameConn =
      this->dataPtr->colorCam->ConnectNewImageFrame(
          std::bind(&RealSensePlugin::OnNewFrame, this, this->dataPtr->colorCam,
                    this->dataPtr->colorPub));
std::cout <<"Debug 15!!!!!!!!!!!!!!!"<<std::endl;
  // Listen to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));
std::cout <<"Debug 16!!!!!!!!!!!!!!!"<<std::endl;      
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewFrame(const rendering::CameraPtr cam,
                                 const transport::PublisherPtr pub) const
{
std::cout <<"Debug 17!!!!!!!!!!!!!!!"<<std::endl;	
  msgs::ImageStamped msg;
std::cout <<"Debug 18!!!!!!!!!!!!!!!"<<std::endl;
  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->dataPtr->world->GetSimTime());
std::cout <<"Debug 19!!!!!!!!!!!!!!!"<<std::endl;
  // Set Image Dimensions
  msg.mutable_image()->set_width(cam->ImageWidth());
  msg.mutable_image()->set_height(cam->ImageHeight());
std::cout <<"Debug 20!!!!!!!!!!!!!!!"<<std::endl;
  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(cam->ImageFormat()));
std::cout <<"Debug 21!!!!!!!!!!!!!!!"<<std::endl;
  // Set Image Data
  msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
  msg.mutable_image()->set_data(
      cam->ImageData(),
      cam->ImageDepth() * cam->ImageWidth() * cam->ImageHeight());
std::cout <<"Debug 22!!!!!!!!!!!!!!!"<<std::endl;
  // Publish realsense infrared stream
  pub->Publish(msg);
std::cout <<"Debug 23!!!!!!!!!!!!!!!"<<std::endl;  
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame() const
{
std::cout <<"Debug 24!!!!!!!!!!!!!!!"<<std::endl;	
  // Get Depth Map dimensions
  unsigned int imageSize = this->dataPtr->depthCam->ImageWidth() *
                           this->dataPtr->depthCam->ImageHeight();
std::cout <<"Debug 25!!!!!!!!!!!!!!!"<<std::endl;
  // Check if depthMap size is equivalent to imageSize
  if (this->dataPtr->depthMap.size() != imageSize)
  {
    try
    {
      this->dataPtr->depthMap.resize(imageSize);
    }
    catch (std::bad_alloc &e)
    {
      std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
                << std::endl;
      return;
    }
  }
std::cout <<"Debug 26!!!!!!!!!!!!!!!"<<std::endl;
  // Instantiate message
  msgs::ImageStamped msg;
std::cout <<"Debug 27!!!!!!!!!!!!!!!"<<std::endl;
  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->dataPtr->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i)
  {
    // Check clipping and overflow
    if (depthDataFloat[i] < DEPTH_NEAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_FAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0)
    {
      this->dataPtr->depthMap[i] = 0;
    }
    else
    {
      this->dataPtr->depthMap[i] =
          (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }
std::cout <<"Debug 28!!!!!!!!!!!!!!!"<<std::endl;
  // Pack realsense scaled depth map
  msgs::Set(msg.mutable_time(), this->dataPtr->world->GetSimTime());
  msg.mutable_image()->set_width(this->dataPtr->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->dataPtr->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->dataPtr->depthCam->ImageWidth() *
                                this->dataPtr->depthCam->ImageDepth());
  msg.mutable_image()->set_data(
      this->dataPtr->depthMap.data(),
      sizeof(*this->dataPtr->depthMap.data()) * imageSize);
std::cout <<"Debug 29!!!!!!!!!!!!!!!"<<std::endl;
  // Publish realsense scaled depth map
  this->dataPtr->depthPub->Publish(msg);
std::cout <<"Debug 30!!!!!!!!!!!!!!!"<<std::endl;  
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate()
{
	std::cout <<"Debug 31!!!!!!!!!!!!!!!"<<std::endl;
}


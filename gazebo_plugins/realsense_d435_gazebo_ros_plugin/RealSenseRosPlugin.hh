#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>
#include <memory>
#include <vector>

namespace gazebo{
  /// \brief A plugin that simulates Real Sense camera streams.
  class RealSenseCamRosPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: RealSenseCamRosPlugin();

    /// \brief Destructor.
    public: ~RealSenseCamRosPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for the World Update event.
    public: void OnUpdate();

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped
    /// message.
    public: virtual void OnNewDepthFrame();

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub);

    /// \brief Pointer to the model containing the plugin.
    protected: physics::ModelPtr rsModel;

    /// \brief Pointer to the world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to the Depth Camera Renderer.
    protected: rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    protected: rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Camera Renderer.
    protected: rendering::CameraPtr ired1Cam;

    /// \brief Pointer to the Infrared2 Camera Renderer.
    protected: rendering::CameraPtr ired2Cam;

    /// \brief Pointer to the transport Node.
    protected: transport::NodePtr transportNode;

    // \brief Store Real Sense depth map data.
    protected: std::vector<uint16_t> depthMap;

    /// \brief Pointer to the Depth Publisher.
    protected: transport::PublisherPtr depthPub;

    /// \brief Pointer to the Color Publisher.
    protected: transport::PublisherPtr colorPub;

    /// \brief Pointer to the Infrared Publisher.
    protected: transport::PublisherPtr ired1Pub;

    /// \brief Pointer to the Infrared2 Publisher.
    protected: transport::PublisherPtr ired2Pub;

    /// \brief Pointer to the Depth Camera callback connection.
    protected: event::ConnectionPtr newDepthFrameConn;

    /// \brief Pointer to the Depth Camera callback connection.
    protected: event::ConnectionPtr newIred1FrameConn;

    /// \brief Pointer to the Infrared Camera callback connection.
    protected: event::ConnectionPtr newIred2FrameConn;

    /// \brief Pointer to the Color Camera callback connection.
    protected: event::ConnectionPtr newColorFrameConn;

    /// \brief Pointer to the World Update event connection.
    protected: event::ConnectionPtr updateConnection;
    
    // vector of sensors available
    protected: sensors::Sensor_V sensor;
    protected: std::vector<std::string> realsense_sensor;
    
    // ROS
    protected: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    /// \brief A pointer to the ROS node. A node will be instantiated if it does not exist.
    protected: ros::NodeHandle* rosnode_;
    
    private: image_transport::ImageTransport* itnode_;
    protected: image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

    /// \brief ROS image messages
    protected: sensor_msgs::Image image_msg_, depth_msg_;
    
  };
}
#endif

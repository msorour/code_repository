#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

/*
#include <thread>
#include "std_msgs/Float32MultiArray.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
*/

namespace gazebo{
  // Forward declare private data class
  struct RealSensePluginPrivate;

  /// \brief A plugin that simulates Real Sense camera streams.
  class GAZEBO_VISIBLE RealSenseCamPlugin : public ModelPlugin{
    /// \brief Constructor.
    public: RealSenseCamPlugin();

    /// \brief Destructor.
    public: ~RealSenseCamPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for the World Update event.
    public: void OnUpdate();

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewDepthFrame() const;

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) const;

    /// \brief Private data pointer.
    private: std::unique_ptr<RealSensePluginPrivate> dataPtr;
    
    /*
    // A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// ROS publishers
		private: ros::Publisher rgb_image_pub;
		private: ros::Publisher depth_image_pub;
		private: ros::Publisher right_image_pub;
		private: ros::Publisher left_image_pub;
		
		// ROS helper function that processes messages
		private: void QueueThread();
		
		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;
		
		// A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
		*/
		
		//private: image_transport::ImageTransport* itnode_;
		//private: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
		//private: sensor_msgs::Image image_msg_, depth_msg_;
		/*
		private: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    /// \brief A pointer to the ROS node.
    ///  A node will be instantiated if it does not exist.
    private: ros::NodeHandle* rosnode_;
    private: image_transport::ImageTransport* itnode_;
    private: image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

    /// \brief ROS image messages
    private: sensor_msgs::Image image_msg_, depth_msg_;
    */
  };
}
#endif

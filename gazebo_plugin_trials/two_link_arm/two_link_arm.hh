#ifndef _TWO_LINK_ARM_HH_
#define _TWO_LINK_ARM_HH_

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <map>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

//#include "ros_gazebo_plugin_comm/Vector7d.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo{
  class TwoLinkArm : public ModelPlugin{
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

    // Called by the world update start event
    public: void OnUpdate();
    
    // Set initial configuration
    public: void SetInitialJointConfig( std::map<std::string, double> init_joint_config_map );


		// ROS RELATED FUNCTIONS:    
		// Handle an incoming message from ROS
		public: void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& _msg);

		// ROS helper function that processes messages
		private: void QueueThread();

		
    // Pointer to the model.
    private: physics::ModelPtr model;

    // Pointer to the joint.
    //private: physics::JointPtr joint1;
    private: physics::Joint_V joint;
    
    
    private: float force_cmd[2];

    // A PID controller for the joint.
    private: common::PID pid;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // A node used for transport
    private: transport::NodePtr node;

    // A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
		
		
		// A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// A ROS subscriber
		private: ros::Subscriber rosSub;

		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		// A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
    
  };
}

#endif

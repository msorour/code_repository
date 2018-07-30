#ifndef _FRANKA_PANDA_CONTROL_PLUGIN_HH_
#define _FRANKA_PANDA_CONTROL_PLUGIN_HH_

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

#include <stdio.h>
#include <stdlib.h>

namespace gazebo{
  class AllegroRightHandControlPlugin : public ModelPlugin{
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Called by the world update start event
    public: void OnUpdate();
    
    // ROS RELATED FUNCTIONS:    
		// Handle an incoming message from ROS
		public: void GetJointForceCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg);
		
		// Handle an incoming message from ROS
		public: void SendJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg);

		// ROS helper function that processes messages
		private: void QueueThread();
		
		
    // Pointer to the model.
    private: physics::ModelPtr model;

    // Pointer to the joint.
    private: physics::Joint_V joint;
    
    private: float joint_force_cmd[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// A ROS subscriber
		private: ros::Subscriber JointTorqueCommandSub;
		
		// ROS publishers
		private: ros::Publisher JointPositionPub;
		private: ros::Publisher JointVelocityPub;

		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		// A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
    
  };
}

#endif

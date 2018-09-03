#ifndef _FRANKA_PANDA_CONTROL_PLUGIN_HH_
#define _FRANKA_PANDA_CONTROL_PLUGIN_HH_

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <map>
#include <thread>
#include "std_msgs/Float32MultiArray.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo{
  class GenericInterfacePlugin : public ModelPlugin{
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Called by the world update start event
    public: void OnUpdate();

		// ROS RELATED FUNCTIONS:    
		// Handle an incoming message from ROS
		public: void GetJointTorqueCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg);
		public: void GetJointPositionCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg);
		public: void GetJointVelocityCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg);
		
		// ROS helper function that processes messages
		private: void QueueThread();
		
		// simulation time.
    private: double sim_time;
    private: physics::WorldPtr world;
    
    // Pointer to the model.
    private: physics::ModelPtr model;

    // Pointer to the joint.
    //private: physics::JointPtr joint1;
    private: physics::Joint_V joint;
    
    private: physics::JointWrench wrench;
    private: ignition::math::Vector3d torque;
    
    private: int joint_count, active_joint_count=0, j=0;
    private: std_msgs::Float32MultiArray joint_torque_cmd, joint_position_cmd, joint_velocity_cmd;
    private: bool torque_cmd_flag=false, position_cmd_flag=false, velocity_cmd_flag=false;
    
    // A PID controller for the joint.
    private: common::PID pid;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// A ROS subscriber
		private: ros::Subscriber JointTorqueCommandSub, JointPositionCommandSub, JointVelocityCommandSub;
		
		// ROS publishers
		private: ros::Publisher JointPositionPub;
		private: ros::Publisher JointVelocityPub;
		private: ros::Publisher JointTorquePub;

		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		// A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
    
  };
}

#endif

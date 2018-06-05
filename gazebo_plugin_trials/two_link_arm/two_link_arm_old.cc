#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <map>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "two_link_arm.hh"

/*
namespace gazebo{
  class TwoLinkArm : public ModelPlugin{
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr){
      // Safety check
      if (_model->GetJointCount() == 0){
        std::cerr << "Invalid joint count, plugin not loaded\n";
        return;
      }
      
      // Just output a message for now
      std::cerr << "\nThe JointControl plugin is attach to model[" << _model->GetName() << "]\n";
      
      // Store the pointer to the model
      this->model = _model;
      
      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint  = _model->GetJoints();
      //this->joint1 = _model->GetJoints()[0];
      std::cerr << "\nJoints: " << this->joint[0]->GetName() << ", " << this->joint[1]->GetScopedName() << ", " << this->joint[2]->GetScopedName() << "\n";
      std::cerr << "\nJoints: " << this->joint[0]->GetType() << ", " << this->joint[1]->GetType() << ", " << this->joint[2]->GetType() << "\n";
      
      //this->model->SetJointPosition("two_link_arm::two_link_arm::joint1",1.0);
      
      std::map<std::string, double> init_joint_config_map;
      init_joint_config_map["two_link_arm::two_link_arm::joint1"] = 1.57;
      init_joint_config_map["two_link_arm::two_link_arm::joint2"] = 1.57;
      this->model->SetJointPositions(init_joint_config_map);
      
      // Setup a P-controller, with a gain of 0.1.
      //this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      //this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

      // Set the joint's target velocity. This target velocity is just for demonstration purposes.
      //this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 10.0);

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TwoLinkArm::OnUpdate, this));
      
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/set_initial_joint_config";

      // Subscribe to the topic, and register a callback
      //this->sub = this->node->Subscribe(topicName, &TwoLinkArm::SetInitialJointConfig, this);
    }

    // Called by the world update start event
    public: void OnUpdate(){
      int joint_count = this->model->GetJointCount();
      this->joint[1]->SetForce(0, 40);
      this->joint[2]->SetForce(0, 1);
      
      double th1 = this->joint[1]->GetAngle(1).Radian();
      double th2 = this->joint[2]->GetAngle(1).Radian();
      
      std::cerr << "\n Joints=" << joint_count << ", th1=" << th1 << ", th2=" << th2 << "\n";
    }
    
    // Set initial configuration
    public: void SetInitialJointConfig( std::map<std::string, double> init_joint_config_map ){
      this->model->SetJointPositions(init_joint_config_map);
    }
    
    // Handle incoming message
    // param[in] _msg Repurpose a vector3 message. This function will only use the x component.
    //private: void OnMsg(ConstVector3dPtr &_msg)
    //{
    //  this->SetVelocity(_msg->x());
    //}
    
    // Pointer to the model.
    private: physics::ModelPtr model;

    // Pointer to the joint.
    //private: physics::JointPtr joint1;
    private: physics::Joint_V joint;

    // A PID controller for the joint.
    private: common::PID pid;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // brief A node used for transport
    private: transport::NodePtr node;

    // brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TwoLinkArm)
}


*/










using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TwoLinkArm)

void TwoLinkArm::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/){
  // Safety check
  if (_model->GetJointCount() == 0){
    std::cerr << "Invalid joint count, plugin not loaded\n";
    return;
  }
  
  // Just output a message for now
  std::cerr << "\nThe JointControl plugin is attach to model[" << _model->GetName() << "]\n";
  
  // Store the pointer to the model
  this->model = _model;
  
  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  this->joint  = _model->GetJoints();
  //this->joint1 = _model->GetJoints()[0];
  std::cerr << "\nJoints: " << this->joint[0]->GetName() << ", " << this->joint[1]->GetScopedName() << ", " << this->joint[2]->GetScopedName() << "\n";
  std::cerr << "\nJoints: " << this->joint[0]->GetType() << ", " << this->joint[1]->GetType() << ", " << this->joint[2]->GetType() << "\n";
  
  //this->model->SetJointPosition("two_link_arm::two_link_arm::joint1",1.0);
  
  std::map<std::string, double> init_joint_config_map;
  init_joint_config_map["two_link_arm::two_link_arm::joint1"] = 1.57;
  init_joint_config_map["two_link_arm::two_link_arm::joint2"] = 1.57;
  this->model->SetJointPositions(init_joint_config_map);
  
  // Setup a P-controller, with a gain of 0.1.
  //this->pid = common::PID(0.1, 0, 0);

  // Apply the P-controller to the joint.
  //this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

  // Set the joint's target velocity. This target velocity is just for demonstration purposes.
  //this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 10.0);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
  std::bind(&TwoLinkArm::OnUpdate, this));
  
  // Create the node
  this->node = transport::NodePtr(new transport::Node());
  #if GAZEBO_MAJOR_VERSION < 8
  this->node->Init(this->model->GetWorld()->GetName());
  #else
  this->node->Init(this->model->GetWorld()->Name());
  #endif

  // Create a topic name
  std::string topicName = "~/" + this->model->GetName() + "/set_initial_joint_config";

  // Subscribe to the topic, and register a callback
  //this->sub = this->node->Subscribe(topicName, &TwoLinkArm::SetInitialJointConfig, this);

	
	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized()){
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so =
		ros::SubscribeOptions::create<std_msgs::Float32>(
		    "/" + this->model->GetName() + "/vel_cmd",
		    1,
		    boost::bind(&TwoLinkArm::OnRosMsg, this, _1),
		    ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread =
		std::thread(std::bind(&TwoLinkArm::QueueThread, this));
}


void TwoLinkArm::OnUpdate(){
  int joint_count = this->model->GetJointCount();
  this->joint[1]->SetForce(0, 40);
  this->joint[2]->SetForce(0, 1);
  
  double th1 = this->joint[1]->GetAngle(1).Radian();
  double th2 = this->joint[2]->GetAngle(1).Radian();
  
  std::cerr << "Joints=" << joint_count << ", th1=" << th1 << ", th2=" << th2 << "\n";
}

// TODO : ROS action server should have the initial joint configuration
void TwoLinkArm::SetInitialJointConfig( std::map<std::string, double> init_joint_config_map ){
  this->model->SetJointPositions(init_joint_config_map);
}



// Handle an incoming message from ROS
// param[in] _msg A float value that is used to set the velocity of the Velodyne.
void TwoLinkArm::OnRosMsg(const std_msgs::Float32ConstPtr &_msg){
  //this->SetVelocity(_msg->data);
}

// ROS helper function that processes messages
void TwoLinkArm::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok()){
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}



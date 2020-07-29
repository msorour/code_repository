#include "two_link_arm.hh"

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
  std::cerr << "\nThe TwoLinkArm control plugin is attached to model[" << _model->GetName() << "]\n";
  
  // Store the pointer to the model
  this->model = _model;
  
  // Get the joints
  this->joint  = _model->GetJoints();
  
  // Set initial joint configuration
  std::map<std::string, double> init_joint_config_map;
  init_joint_config_map["two_link_arm::two_link_arm::joint1"] = 1.57;
  init_joint_config_map["two_link_arm::two_link_arm::joint2"] = 1.57;
  this->model->SetJointPositions(init_joint_config_map);
  
  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&TwoLinkArm::OnUpdate, this) );
  
  // Initialize ROS, if it has not already been initialized.
	if (!ros::isInitialized()){
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	
	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
		"/force_cmd_vector",
	  1,
	  boost::bind(&TwoLinkArm::OnRosMsg, this, _1),
	  ros::VoidPtr(), 
	  &this->rosQueue );
	this->rosSub = this->rosNode->subscribe(so);
	
	/*
	// Create a topic for publishing joint state.
	ros::SubscribeOptions pub_joint_state = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
		"/joint_state/position",
	  1,
	  boost::bind(&TwoLinkArm::OnRosMsg, this, _1),
	  ros::VoidPtr(), 
	  &this->rosQueue );
	*/
	
	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&TwoLinkArm::QueueThread, this));
}


void TwoLinkArm::OnUpdate(){
  int joint_count = this->model->GetJointCount();
  this->joint[1]->SetForce(0, this->force_cmd[0]);
  this->joint[2]->SetForce(0, this->force_cmd[1]);
  
  double th1 = this->joint[1]->GetAngle(1).Radian();
  double th2 = this->joint[2]->GetAngle(1).Radian();
}

// TODO : ROS action server should have the initial joint configuration
void TwoLinkArm::SetInitialJointConfig( std::map<std::string, double> init_joint_config_map ){
  this->model->SetJointPositions(init_joint_config_map);
}


// Handle an incoming message from ROS
// param[in] _msg A float value that is used to set the joint force.
void TwoLinkArm::OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  //this->joint[2]->SetForce(0, _msg->data);
  force_cmd[0] = _msg->data[0];
  force_cmd[1] = _msg->data[1];
  
  double th1 = this->joint[1]->GetAngle(1).Radian();
  double th2 = this->joint[2]->GetAngle(1).Radian();
  
  std::cerr << "th1=" << th1 << ", th2=" << th2 << "\n";
}

// ROS helper function that processes messages
void TwoLinkArm::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok()){
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}



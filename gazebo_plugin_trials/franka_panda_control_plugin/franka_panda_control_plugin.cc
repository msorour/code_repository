#include "franka_panda_control_plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(FrankaPandaControlPlugin)

void FrankaPandaControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Safety check
  if (_model->GetJointCount() == 0){
    std::cerr << "Invalid joint count, plugin not loaded\n";
    return;
  }
  
  // A plugin attachment confirmation message
  std::cerr << "\nThe FrankaPandaControlPlugin control plugin is attached to model[" << _model->GetName() << "]\n";
  
  // Store the pointer to the model
  this->model = _model;
  
  double joint1_init=0;
  // Check that the joint1_init element exists, then read the value
	if (_sdf->HasElement("joint1_init")){
		joint1_init = _sdf->Get<double>("joint1_init");
  }
  
  // Get the joints
  this->joint  = _model->GetJoints();
  std::cerr << "Initial joint configuration: " << this->joint[1]->GetAngle(2).Radian() << ", " << this->joint[2]->GetAngle(2).Radian() << ", " << this->joint[3]->GetAngle(2).Radian() << ", " << this->joint[4]->GetAngle(2).Radian() << ", " << this->joint[5]->GetAngle(2).Radian() << ", " << this->joint[6]->GetAngle(2).Radian() << ", " << this->joint[7]->GetAngle(2).Radian() << "\n";
 	
  // Set initial joint configuration : TODO: proper loading from parameter server or something!
  std::map<std::string, double> init_joint_config_map;
  init_joint_config_map["panda_arm::panda_arm_joint1"] = joint1_init;
  init_joint_config_map["panda_arm::panda_arm_joint2"] = joint1_init;
  init_joint_config_map["panda_arm::panda_arm_joint3"] = joint1_init;
  init_joint_config_map["panda_arm::panda_arm_joint4"] = joint1_init;
  init_joint_config_map["panda_arm::panda_arm_joint5"] = joint1_init;
  init_joint_config_map["panda_arm::panda_arm_joint6"] = joint1_init;
  init_joint_config_map["panda_arm::panda_arm_joint7"] = joint1_init;
  this->model->SetJointPositions(init_joint_config_map);
  this->model->Update();
  std::cerr << "Initial joint configuration is set." << "\n";
  
  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&FrankaPandaControlPlugin::OnUpdate, this) );
  
  
  // ROS RELATED
  // Initialize ROS, if it has not already been initialized.
	if (!ros::isInitialized()){
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	
	// Create a topic to receive on which the jointspace force command vector, and subscribe to it.
	ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
		"/force_cmd_vector",
	  1,
	  boost::bind(&FrankaPandaControlPlugin::GetJointForceCommand, this, _1),
	  ros::VoidPtr(), 
	  &this->rosQueue );
	this->JointTorqueCommandSub = this->rosNode->subscribe(so);
	
	// Create topics to publish joint state.
	JointPositionPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/joint_state/position", 10);
	JointVelocityPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/joint_state/velocity", 10);
	
	
	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&FrankaPandaControlPlugin::QueueThread, this));
}


// OnUpdate
void FrankaPandaControlPlugin::OnUpdate(){
	// Apply joint torque command received
	// In Gazebo joint torque is named "force". 
	// We start with index "1" since joint[0] is a fixed joint, not part of the arm DOFs.
  this->joint[1]->SetForce(0, this->joint_force_cmd[0]);
  this->joint[2]->SetForce(0, this->joint_force_cmd[1]);
  this->joint[3]->SetForce(0, this->joint_force_cmd[2]);
  this->joint[4]->SetForce(0, this->joint_force_cmd[3]);
  this->joint[5]->SetForce(0, this->joint_force_cmd[4]);
  this->joint[6]->SetForce(0, this->joint_force_cmd[5]);
  this->joint[7]->SetForce(0, this->joint_force_cmd[6]);
  
  // Publish joint position vector
  std_msgs::Float32MultiArray joint_position_vector;
  joint_position_vector.data.clear();
	joint_position_vector.data.push_back(this->joint[1]->GetAngle(2).Radian());
	joint_position_vector.data.push_back(this->joint[2]->GetAngle(2).Radian());
  joint_position_vector.data.push_back(this->joint[3]->GetAngle(2).Radian());
  joint_position_vector.data.push_back(this->joint[4]->GetAngle(2).Radian());
  joint_position_vector.data.push_back(this->joint[5]->GetAngle(2).Radian());
  joint_position_vector.data.push_back(this->joint[6]->GetAngle(2).Radian());
  joint_position_vector.data.push_back(this->joint[7]->GetAngle(2).Radian());
  JointPositionPub.publish(joint_position_vector);
  
  // Publish joint velocity vector
  std_msgs::Float32MultiArray joint_velocity_vector;
  joint_velocity_vector.data.clear();
	joint_velocity_vector.data.push_back(this->joint[1]->GetVelocity(2));
	joint_velocity_vector.data.push_back(this->joint[2]->GetVelocity(2));
  joint_velocity_vector.data.push_back(this->joint[3]->GetVelocity(2));
  joint_velocity_vector.data.push_back(this->joint[4]->GetVelocity(2));
  joint_velocity_vector.data.push_back(this->joint[5]->GetVelocity(2));
  joint_velocity_vector.data.push_back(this->joint[6]->GetVelocity(2));
  joint_velocity_vector.data.push_back(this->joint[7]->GetVelocity(2));
  JointVelocityPub.publish(joint_velocity_vector);
}


// TODO : ROS action server should have the initial joint configuration
void FrankaPandaControlPlugin::SetInitialJointConfig( std::map<std::string, double> init_joint_config_map ){
  this->model->SetJointPositions(init_joint_config_map);
}


// Handle an incoming message from ROS
// param[in] _msg A float value that is used to set the joint force.
void FrankaPandaControlPlugin::GetJointForceCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_force_cmd[0] = _msg->data[0];
  joint_force_cmd[1] = _msg->data[1];
  joint_force_cmd[2] = _msg->data[2];
  joint_force_cmd[3] = _msg->data[3];
  joint_force_cmd[4] = _msg->data[4];
  joint_force_cmd[5] = _msg->data[5];
  joint_force_cmd[6] = _msg->data[6];
}


// ROS helper function that processes messages
void FrankaPandaControlPlugin::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok()){
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}



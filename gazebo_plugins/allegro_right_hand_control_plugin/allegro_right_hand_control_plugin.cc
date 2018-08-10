#include "allegro_right_hand_control_plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AllegroRightHandControlPlugin)

void AllegroRightHandControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Safety check
  if (_model->GetJointCount() == 0){
    std::cerr << "Invalid joint count, plugin not loaded\n";
    return;
  }
  
  // A plugin attachment confirmation message
  std::cerr << "\nThe AllegroRightHandControlPlugin control plugin is attached to model[" << _model->GetName() << "]\n";
  
  // Store the pointer to the model
  this->model = _model;
  
  double index_joint_init[4], middle_joint_init[4], pinky_joint_init[4], thumb_joint_init[4];
  
  // Check that the joint1_init element exists, then read the value
  char index_str[30], middle_str[30], pinky_str[30], thumb_str[30];
	for(int k=0; k<4; k++){
		snprintf(index_str, sizeof index_str, "%s%d%s", "index_joint", k+1, "_init");
		snprintf(middle_str, sizeof middle_str, "%s%d%s", "middle_joint", k+1, "_init");
		snprintf(pinky_str, sizeof pinky_str, "%s%d%s", "pinky_joint", k+1, "_init");
		snprintf(thumb_str, sizeof thumb_str, "%s%d%s", "thumb_joint", k+1, "_init");
		if (_sdf->HasElement(index_str))
			index_joint_init[k] = _sdf->Get<double>(index_str);
		if (_sdf->HasElement(middle_str))
			middle_joint_init[k] = _sdf->Get<double>(middle_str);
		if (_sdf->HasElement(pinky_str))
			pinky_joint_init[k] = _sdf->Get<double>(pinky_str);
		if (_sdf->HasElement(thumb_str))
			thumb_joint_init[k] = _sdf->Get<double>(thumb_str);
  }
	
  
  // Get the joints
  this->joint  = _model->GetJoints();
  for(int k=0; k<17; k++){
  	std::cerr << "Joint"<<k<< "Scoped Name: " << this->joint[k]->GetScopedName() << "\n";
  }
  
  // Set initial joint configuration : TODO: proper loading from parameter server or something!
  std::map<std::string, double> init_joint_config_map;
  char str[50];
	for(int k=0; k<4; k++){
		snprintf(str, sizeof str, "%s%d", "allegro_right_hand::index_joint", k+1);
		init_joint_config_map[str] = index_joint_init[k];
		snprintf(str, sizeof str, "%s%d", "allegro_right_hand::middle_joint", k+1);
		init_joint_config_map[str] = middle_joint_init[k];
		snprintf(str, sizeof str, "%s%d", "allegro_right_hand::pinky_joint", k+1);
		init_joint_config_map[str] = pinky_joint_init[k];
		snprintf(str, sizeof str, "%s%d", "allegro_right_hand::thumb_joint", k+1);
		init_joint_config_map[str] = thumb_joint_init[k];
  }
  this->model->SetJointPositions(init_joint_config_map);
  std::cerr << "Initial joint configuration is set." << "\n";
  
  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&AllegroRightHandControlPlugin::OnUpdate, this) );
  
  
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
		"/allegro_right_hand/joint_command/torque",
	  1,
	  boost::bind(&AllegroRightHandControlPlugin::GetJointForceCommand, this, _1),
	  ros::VoidPtr(), 
	  &this->rosQueue );
	this->JointTorqueCommandSub = this->rosNode->subscribe(so);
	
	// Create topics to publish joint state.
	JointPositionPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/joint_state/position", 10);
	JointVelocityPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/joint_state/velocity", 10);
	
	
	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&AllegroRightHandControlPlugin::QueueThread, this));
}


// OnUpdate
void AllegroRightHandControlPlugin::OnUpdate(){
	// Apply joint torque command received
	// In Gazebo joint torque is named "force". 
	// We start with index "1" since joint[0] is a fixed joint, not part of the allegro hand DOFs.
  float q[16];
  std_msgs::Float32MultiArray joint_position_vector;
  std_msgs::Float32MultiArray joint_velocity_vector;
  joint_position_vector.data.clear();
  joint_velocity_vector.data.clear();
  for(int k=0; k<16; k++){
  	q[k] = this->joint[k+1] ->GetAngle(2).Radian();
  	this->joint[k+1] ->SetForce(0, joint_force_cmd[k]);
  	
  	// Publish joint position vector
  	joint_position_vector.data.push_back(this->joint[k+1] ->GetAngle(2).Radian());
  	// Publish joint velocity vector
  	joint_velocity_vector.data.push_back(this->joint[k+1] ->GetVelocity(2));
  }
  JointPositionPub.publish(joint_position_vector);
  JointVelocityPub.publish(joint_velocity_vector);
}


// Handle an incoming message from ROS
// param[in] _msg A float value that is used to set the joint force.
void AllegroRightHandControlPlugin::GetJointForceCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	for(int k=0; k<16; k++){
  	joint_force_cmd[k] = _msg->data[k];
  }
}


// ROS helper function that processes messages
void AllegroRightHandControlPlugin::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok()){
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}



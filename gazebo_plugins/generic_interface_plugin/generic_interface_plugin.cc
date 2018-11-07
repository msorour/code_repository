#include "generic_interface_plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GenericInterfacePlugin)

void GenericInterfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  joint_count = _model->GetJointCount();
  std::string model_name = _model->GetName();
  // Safety check
  if (joint_count==0){std::cerr << "Invalid joint count, plugin not loaded\n";return;}
  
  // A plugin attachment confirmation message
  std::cerr << "\nThe GenericInterfacePlugin is attached to model[" << model_name << "]\n";
  
  // Store the pointer to the model
  this->model = _model;
  
  // Get the joints
  this->joint = _model->GetJoints();
  for(int k=0; k<joint_count; k++)
  	std::cerr << "Joint"<<k<< "Scoped Name: " << this->joint[k]->GetScopedName() << "\n";
  
  // Get number of active joints (assuming active joints are only revolute at the moment)
  for(int k=0; k<joint_count; k++){
    if(this->joint[k]->HasType(physics::Base::HINGE_JOINT) or this->joint[k]->HasType(physics::Base::SLIDER_JOINT))
      active_joint_count++;
  }
  
  // Check that the joint1_init elements exist, then read the values
  double joint_init[joint_count];
  char joint_init_str[30];
  j=0;
	for(int k=0; k<joint_count; k++){
	  if(this->joint[k]->HasType(physics::Base::HINGE_JOINT) or this->joint[k]->HasType(physics::Base::SLIDER_JOINT)){
		  snprintf(joint_init_str, sizeof joint_init_str, "%s%d%s", "joint", j+1, "_init");
		  if (_sdf->HasElement(joint_init_str))
			  joint_init[j] = _sdf->Get<double>(joint_init_str);
			j++;
    }
  }
  
  // Set initial joint configuration and initialize the torque command vector
  std::map<std::string, double> init_joint_config_map;
  char str[50];
  j=0;
	for(int k=0; k<joint_count; k++){
	  if(this->joint[k]->HasType(physics::Base::HINGE_JOINT) or this->joint[k]->HasType(physics::Base::SLIDER_JOINT)){
		  init_joint_config_map[this->joint[k]->GetScopedName()] = joint_init[j];
		  joint_torque_cmd.data.push_back(0.0);
		  joint_position_cmd.data.push_back(0.0);
		  joint_velocity_cmd.data.push_back(0.0);
		  j++;
		}
	}
  this->model->SetJointPositions(init_joint_config_map);
  std::cerr << "Initial joint configuration is set." << "\n";
  
  // Check that the joint1_velocity_limit elements exist, then read the values
  double joint_velocity_limit[joint_count];
  double max_allowed_force=1;
  char joint_velocity_limit_str[30];
  j=0;
	for(int k=0; k<joint_count; k++){
	  if(this->joint[k]->HasType(physics::Base::HINGE_JOINT) or this->joint[k]->HasType(physics::Base::SLIDER_JOINT)){
		  snprintf(joint_velocity_limit_str, sizeof joint_init_str, "%s%d%s", "joint", j+1, "_velocity_limit");
		  if (_sdf->HasElement(joint_init_str))
			  joint_velocity_limit[j] = _sdf->Get<double>(joint_velocity_limit_str);
			this->joint[k]->SetParam("velmax", 2, 0.1);
			this->joint[k]->SetParam("fmax", 2, 1.0);
			std::cerr << "VelocityLimit = "<< this->joint[k]->GetVelocityLimit(0) << "\n";
			j++;
    }
  }
  
  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin( std::bind(&GenericInterfacePlugin::OnUpdate, this) );
  
  
  // ROS RELATED
  // Initialize ROS, if it has not already been initialized.
	if (!ros::isInitialized()){
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	
	// Create a topic to receive on which command vectors, and subscribe to it.
	ros::SubscribeOptions so_TorqueCommands = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/torque",1,
	boost::bind(&GenericInterfacePlugin::GetJointTorqueCommand, this, _1),ros::VoidPtr(),&this->rosQueue);
	this->JointTorqueCommandSub = this->rosNode->subscribe(so_TorqueCommands);
	ros::SubscribeOptions so_PositionCommands = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/position",1,
	boost::bind(&GenericInterfacePlugin::GetJointPositionCommand, this, _1),ros::VoidPtr(),&this->rosQueue);
	this->JointPositionCommandSub = this->rosNode->subscribe(so_PositionCommands);
	ros::SubscribeOptions so_VelocityCommands = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/velocity",1,
	boost::bind(&GenericInterfacePlugin::GetJointVelocityCommand, this, _1),ros::VoidPtr(),&this->rosQueue);
	this->JointVelocityCommandSub = this->rosNode->subscribe(so_VelocityCommands);
	
	// Create topics to publish joint state.
	JointPositionPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_state/position", 10);
	JointVelocityPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_state/velocity", 10);
	JointTorquePub   = this->rosNode->advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_state/torque"  , 10);
	
	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&GenericInterfacePlugin::QueueThread, this));
}


// OnUpdate
void GenericInterfacePlugin::OnUpdate(){
	// In Gazebo joint torque is named "force". 
	// We start with index "1" since joint[0] is a fixed joint, not part of the arm DOFs.
  
  // Get simulation time and send it along
  world = this->model->GetWorld();
  //sim_time = this->world->GetSimTime().Double();  //Gazebo 7
  sim_time = this->world->SimTime().Double();    //Gazebo 8
  
  std_msgs::Float32MultiArray joint_position_vector;
  std_msgs::Float32MultiArray joint_velocity_vector;
  std_msgs::Float32MultiArray joint_torque_vector;
  joint_position_vector.data.clear();
  joint_velocity_vector.data.clear();
  joint_torque_vector.data.clear();
  joint_position_vector.data.push_back(sim_time);
  joint_velocity_vector.data.push_back(sim_time);
  
  j=0;
  for(int k=0; k<joint_count; k++){
    if(this->joint[k]->HasType(physics::Base::HINGE_JOINT) or this->joint[k]->HasType(physics::Base::SLIDER_JOINT)){
    	// Apply joint command received
    	if(torque_cmd_flag)
    	  this->joint[k]->SetForce(0, joint_torque_cmd.data[j]);
    	else if(position_cmd_flag)
    	  this->joint[k]->SetPosition(0, joint_position_cmd.data[j]);
    	else if(velocity_cmd_flag)
    	  this->joint[k]->SetVelocity(0, joint_velocity_cmd.data[j]);
    	
    	// Publish joint position vector
    	//joint_position_vector.data.push_back(this->joint[k]->GetAngle(2).Radian());   //Gazebo 7
    	joint_position_vector.data.push_back(this->joint[k]->Position(0));   //Gazebo 8
    	// Publish joint velocity vector
    	joint_velocity_vector.data.push_back(this->joint[k]->GetVelocity(0));
    	// Publish joint torque vector
    	//wrench = this->joint[k+1]->GetForceTorque(0u);
    	//std::cerr << "wrench = " << wrench.body2Torque[0] <<", "<<wrench.body2Torque[1]<<", "<<wrench.body2Torque[2] << "\n";
    	//joint_torque_vector.data.push_back(this->joint[k+1]->GetForceTorque(2));
    	j++;
    }
  }
  JointPositionPub.publish(joint_position_vector);
  JointVelocityPub.publish(joint_velocity_vector);
  JointTorquePub.publish(joint_torque_vector);
}


// Handle an incoming message from ROS
// param[in] _msg A float value that is used to set the joint force.
void GenericInterfacePlugin::GetJointTorqueCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	torque_cmd_flag=true;
	position_cmd_flag=false;
	velocity_cmd_flag=false;
	joint_torque_cmd.data.clear();
	for(int k=0; k<active_joint_count; k++)
	  joint_torque_cmd.data.push_back(_msg->data[k]);
}
void GenericInterfacePlugin::GetJointPositionCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  torque_cmd_flag=false;
	position_cmd_flag=true;
	velocity_cmd_flag=false;
	joint_position_cmd.data.clear();
	for(int k=0; k<active_joint_count; k++)
	  joint_position_cmd.data.push_back(_msg->data[k]);
}
void GenericInterfacePlugin::GetJointVelocityCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  torque_cmd_flag=false;
	position_cmd_flag=false;
	velocity_cmd_flag=true;
	joint_velocity_cmd.data.clear();
	for(int k=0; k<active_joint_count; k++)
	  joint_velocity_cmd.data.push_back(_msg->data[k]);
}


// ROS helper function that processes messages
void GenericInterfacePlugin::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok())
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
}


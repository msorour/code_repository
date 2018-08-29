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
  
  // TODO:MAKE IT MORE GENERIC
  // Check that the joint1_init elements exist, then read the values
  double joint_init[joint_count];
  char joint_init_str[30];
	for(int k=0; k<7; k++){
		snprintf(joint_init_str, sizeof joint_init_str, "%s%d%s", "joint", k+1, "_init");
		if (_sdf->HasElement(joint_init_str))
			joint_init[k] = _sdf->Get<double>(joint_init_str);
  }
  
  // Get the joints
  this->joint = _model->GetJoints();
  for(int k=0; k<joint_count; k++)
  	std::cerr << "Joint"<<k<< "Scoped Name: " << this->joint[k]->GetScopedName() << "\n";
  
  // TODO:MAKE IT MORE GENERIC
  // Set initial joint configuration
  std::map<std::string, double> init_joint_config_map;
  char str[50];
	for(int k=0; k<7; k++)
		init_joint_config_map[this->joint[k+1]->GetScopedName()] = joint_init[k];
  this->model->SetJointPositions(init_joint_config_map);
  std::cerr << "Initial joint configuration is set." << "\n";
  
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
	
	// Create a topic to receive on which the jointspace force command vector, and subscribe to it.
	ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
		"/"+model_name+"/joint_command/torque",
	  1,
	  boost::bind(&GenericInterfacePlugin::GetJointForceCommand, this, _1),
	  ros::VoidPtr(), 
	  &this->rosQueue );
	this->JointTorqueCommandSub = this->rosNode->subscribe(so);
	
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
  
  float q[7];
  for(int k=0; k<7; k++){
  	q[k] = this->joint[k+1] ->GetAngle(2).Radian();
  }
  PandaArmGravityCompensation(q);
  
  std_msgs::Float32MultiArray joint_position_vector;
  std_msgs::Float32MultiArray joint_velocity_vector;
  std_msgs::Float32MultiArray joint_torque_vector;
  joint_position_vector.data.clear();
  joint_velocity_vector.data.clear();
  joint_torque_vector.data.clear();
  
  for(int k=0; k<7; k++){
  	// Apply joint torque command received + computed gravity compensation torque
  	this->joint[k+1] ->SetForce(0, joint_force_cmd[k]+joint_gravity_torque[k]);
  	// Publish joint position vector
  	joint_position_vector.data.push_back(this->joint[k+1]->GetAngle(2).Radian());
  	// Publish joint velocity vector
  	joint_velocity_vector.data.push_back(this->joint[k+1]->GetVelocity(2));
  	// Publish joint torque vector
  	wrench = this->joint[k+1]->GetForceTorque(5);
  	std::cerr << "wrench = " << wrench.body2Torque[0] <<", "<<wrench.body2Torque[1]<<", "<<wrench.body2Torque[2] << "\n";
  	//joint_torque_vector.data.push_back(this->joint[k+1]->GetForceTorque(2));
  }
  JointPositionPub.publish(joint_position_vector);
  JointVelocityPub.publish(joint_velocity_vector);
  JointTorquePub.publish(joint_torque_vector);
}


// TODO : ROS action server should have the initial joint configuration
void GenericInterfacePlugin::SetInitialJointConfig( std::map<std::string, double> init_joint_config_map ){
  this->model->SetJointPositions(init_joint_config_map);
}


// Handle an incoming message from ROS
// param[in] _msg A float value that is used to set the joint force.
void GenericInterfacePlugin::GetJointForceCommand(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	for(int k=0; k<7; k++){
		joint_force_cmd[k] = _msg->data[k];
	}
}


// ROS helper function that processes messages
void GenericInterfacePlugin::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok()){
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}


// Vector of Gravity Compensation Torques
void GenericInterfacePlugin::PandaArmGravityCompensation(float q[7]){
	float M1=2.7, M2=2.7, M3=2.7, M4=2.7, M5=1.7, M6=1.6, M7=1.0;										// Link Masses
	float X1=0, X2=0, X3=0, X4=0, X5=0, X6=0, X7=0;																	// x-coordinate of the CG point of each link
	float Y1=0, Y2=0, Y3=0, Y4=0, Y5=0, Y6=-0.008, Y7=0;														// y-coordinate of the CG point of each link
	float Z1=-0.015, Z2=0.015, Z3=-0.015, Z4=0.015, Z5=-0.035, Z6=0.001, Z7=0.1;		// z-coordinate of the CG point of each link
	float D=0.088, RL3=0.316, RL5=0.384, GZ=9.81;
	
	float FX7=0.0, FY7=0.0, FZ7=0.0, CX7=0.0, CY7=0.0, CZ7=0.0;		// Vector of external forces, just in case we want to use it in the
	
	float C1 = cos(q[0]);
	float S1 = sin(q[0]);
	float C2 = cos(q[1]);
	float S2 = sin(q[1]);
	float C3 = cos(q[2]);
	float S3 = sin(q[2]);
	float C4 = cos(q[3]);
	float S4 = sin(q[3]);
	float C5 = cos(q[4]);
	float S5 = sin(q[4]);
	float C6 = cos(q[5]);
	float S6 = sin(q[5]);
	float C7 = cos(q[6]);
	float S7 = sin(q[6]);
	
	float VP12 = GZ*S2;
	float VP22 = C2*GZ;
	float VSP13 = VP12;
	float VSP23 = VP22;
	float VP13 = C3*VSP13;
	float VP23 = -S3*VSP13;
	float VSP14 = VP13;
	float VSP24 = VP23;
	float VSP34 = VSP23;
	float VP14 = C4*VSP14 - S4*VSP34;
	float VP24 = -C4*VSP34 - S4*VSP14;
	float VSP15 = VP14;
	float VSP25 = VP24;
	float VSP35 = VSP24;
	float VP15 = -C5*VSP15 - S5*VSP35;
	float VP25 = -C5*VSP35 + S5*VSP15;
	float VP16 = C6*VP15 - S6*VSP25;
	float VP26 = -C6*VSP25 - S6*VP15;
	float VSP17 = VP16;
	float VSP27 = VP26;
	float VSP37 = -VP25;
	float VP17 = C7*VSP17 + S7*VSP37;
	float VP27 = C7*VSP37 - S7*VSP17;
	float F31 = GZ*M1;
	float F12 = M2*VP12;
	float F22 = M2*VP22;
	float F13 = M3*VP13;
	float F23 = M3*VP23;
	float F33 = M3*VSP23;
	float F14 = M4*VP14;
	float F24 = M4*VP24;
	float F34 = M4*VSP24;
	float F15 = M5*VP15;
	float F25 = M5*VP25;
	float F35 = -M5*VSP25;
	float F16 = M6*VP16;
	float F26 = M6*VP26;
	float F36 = -M6*VP25;
	float F17 = M7*VP17;
	float F27 = M7*VP27;
	float F37 = -M7*VSP27;
	float E17 = F17 + FX7;
	float E27 = F27 + FY7;
	float E37 = F37 + FZ7;
	float N17 = CX7 - VP27*Z7 - VSP27*Y7;
	float N27 = CY7 + VP17*Z7 + VSP27*X7;
	float N37 = CZ7 - VP17*Y7 + VP27*X7;
	float FDI17 = C7*E17 - E27*S7;
	float FDI37 = C7*E27 + E17*S7;
	float E16 = F16 + FDI17;
	float E26 = -E37 + F26;
	float E36 = F36 + FDI37;
	float N16 = C7*N17 - N27*S7 - VP25*Y6 - VP26*Z6;
	float N26 = -D*FDI37 - N37 + VP16*Z6 + VP25*X6;
	float N36 = C7*N27 - D*E37 + N17*S7 - VP16*Y6 + VP26*X6;
	float FDI16 = C6*E16 - E26*S6;
	float FDI36 = C6*E26 + E16*S6;
	float E15 = F15 + FDI16;
	float E25 = -E36 + F25;
	float E35 = F35 + FDI36;
	float N15 = C6*N16 - N26*S6 - VP25*Z5 - VSP25*Y5;
	float N25 = -N36 + VP15*Z5 + VSP25*X5;
	float N35 = C6*N26 + N16*S6 - VP15*Y5 + VP25*X5;
	float FDI15 = -C5*E15 + E25*S5;
	float FDI35 = -C5*E25 - E15*S5;
	float E14 = F14 + FDI15;
	float E24 = -E35 + F24;
	float E34 = F34 + FDI35;
	float N14 = -C5*N15 - FDI35*RL5 + N25*S5 - VP24*Z4 + VSP24*Y4;
	float N24 = -D*FDI35 - N35 + VP14*Z4 - VSP24*X4;
	float N34 = -C5*N25 - D*E35 + FDI15*RL5 - N15*S5 - VP14*Y4 + VP24*X4;
	float FDI14 = C4*E14 - E24*S4;
	float FDI34 = -C4*E24 - E14*S4;
	float E13 = F13 + FDI14;
	float E23 = E34 + F23;
	float E33 = F33 + FDI34;
	float N13 = C4*N14 - N24*S4 + VP23*Z3 + VSP23*Y3;
	float N23 = D*FDI34 + N34 + VP13*Z3 - VSP23*X3;
	float N33 = -C4*N24 - D*E34 - N14*S4 - VP13*Y3 + VP23*X3;
	float FDI13 = C3*E13 - E23*S3;
	float FDI33 = -C3*E23 - E13*S3;
	float E12 = F12 + FDI13;
	float E22 = E33 + F22;
	float E32 = FDI33;
	float N12 = C3*N13 + FDI33*RL3 - N23*S3 - VP22*Z2;
	float N22 = N33 + VP12*Z2;
	float N32 = -C3*N23 - FDI13*RL3 - N13*S3 - VP12*Y2 + VP22*X2;
	float FDI12 = C2*E12 - E22*S2;
	float FDI32 = C2*E22 + E12*S2;
	float E11 = FDI12;
	float E21 = -E32;
	float E31 = F31 + FDI32;
	float N11 = C2*N12 + GZ*Y1 - N22*S2;
	float N21 = -GZ*X1 - N32;
	float N31 = C2*N22 + N12*S2;
	float FDI11 = C1*E11 - E21*S1;
	float FDI21 = C1*E21 + E11*S1;
	
	joint_gravity_torque[0] = N31;
	joint_gravity_torque[1] = N32;
	joint_gravity_torque[2] = N33;
	joint_gravity_torque[3] = N34;
	joint_gravity_torque[4] = N35;
	joint_gravity_torque[5] = N36;
	joint_gravity_torque[6] = N37;
	
	//for(int k=0;k<7;k++)
	//	joint_gravity_torque[k] = 0;
}





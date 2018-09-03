#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/FrankaPandaArmModel.h"
#include "../include/franka_panda_arm_plus_own_gripper.h"
#include <iostream>
#include <fstream>
#include <time.h>

using namespace std;
using namespace Eigen;
using namespace franka_panda_gazebo_controller;

void log_data(void);
void open_logs(void);
void close_logs(void);

void GetJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_position_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		arm_joint_position(k)  = _msg->data[k+1];
	for(int k=0; k<2; k++)
		gripper_joint_position(k)  = _msg->data[k+7+1];	
}
void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_velocity_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		arm_joint_velocity(k)  = _msg->data[k+1];
	for(int k=0; k<2; k++)
		gripper_joint_velocity(k)  = _msg->data[k+7+1];
}

int main(int argc, char **argv){
  string arm_name;
  arm_name = argv[2];
  
  ros::init(argc, argv, "franka_panda_arm_plus_own_gripper_gazebo_task_space_pose_controller_rpy");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/torque", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+arm_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+arm_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(500);
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
  
	
	arm_velocity_bE_desired << 0.05, 0, 0, 0, 0, 0;
	Kv = 200;
	
	open_logs();
  while (ros::ok() and ros::Time::now().toSec()<10.0 ){
  //while (ros::ok()){
  	time_now = ros::Time::now().toSec();
  	cout << "time_now = " << time_now  << endl;
  	geometric_jacobian = arm_geometric_jacobian_matrix(arm_joint_position,"base");
  	//geometric_jacobian = arm_geometric_jacobian_matrix(arm_joint_position,"effector");
  	
  	///
  	// Computing Joint Torque Command
  	// 1. control in joint space
  	//joint_velocity_desired  = Pinv_damped(geometric_jacobian, 0.001)*velocity_bE_desired;
  	//joint_velocity_error = joint_velocity_desired-joint_velocity;
  	
  	// 2. control in task space (better response)
  	arm_velocity_bE = geometric_jacobian*arm_joint_velocity;
  	arm_velocity_error = arm_velocity_bE_desired-arm_velocity_bE;
  	arm_joint_velocity_error = Pinv_damped(geometric_jacobian, 0.001)*arm_velocity_error;
  	//joint_velocity_error = Pinv_damped(geometric_jacobian, 0.001)*velocity_bE_desired;
  	
  	inertia_matrix = arm_inertia_matrix(arm_joint_position);
  	viscous_friction_torque = arm_viscous_friction_torque(arm_joint_velocity);
  	static_friction_torque = arm_static_friction_torque(arm_joint_velocity);
  	coriolis_centrifugal_torque = arm_coriolis_centrifugal_torque(arm_joint_position, arm_joint_velocity);
  	arm_joint_torque_command  = inertia_matrix*Kv*arm_joint_velocity_error + viscous_friction_torque + static_friction_torque + coriolis_centrifugal_torque;
  	
  	// torque override for testing
  	//arm_joint_torque_command  = arm_gravity_compensation_torque(arm_joint_position);
  	gripper_joint_torque_command << 0,0;
  	//joint_torque_command << 0,0,0,0,0,0,0;
  	///
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<7; k++)
  		torque_command.data.push_back(arm_joint_torque_command(k));
  	for(int k=0; k<2; k++)
  		torque_command.data.push_back(gripper_joint_torque_command(k));
  	JointTorqueCommandPub.publish(torque_command);
  	
  	// Data logging
  	log_data();
  	
  	ros::spinOnce();
    loop_rate.sleep();
  }
  close_logs();
	
  return 0;
}









void log_data(void){
	joint_velocity_response_log << time_now << " " << arm_joint_velocity.transpose() <<  endl;
	joint_velocity_error_log << time_now << " " << arm_joint_velocity_error.transpose() <<  endl;
	
	joint_torque_command_log << time_now << " " << arm_joint_torque_command.transpose() <<  endl;
	
	task_space_velocity_command_log << time_now << " " << arm_velocity_bE_desired.transpose() <<  endl;
	task_space_velocity_response_log << time_now << " " << arm_velocity_bE.transpose() <<  endl;
	task_space_velocity_error_log << time_now << " " << arm_velocity_error.transpose() <<  endl;
}

void open_logs(void){
	joint_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/joint_velocity_response_log.txt");
	joint_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/joint_velocity_error_log.txt");
	
	joint_torque_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/joint_torque_command_log.txt");
	
	task_space_velocity_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/task_space_velocity_command_log.txt");
	task_space_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/task_space_velocity_response_log.txt");
	task_space_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/task_space_velocity_error_log.txt");
}
void close_logs(void){
	joint_velocity_response_log.close();
	joint_velocity_error_log.close();
	
	joint_torque_command_log.close();
	
	task_space_velocity_command_log.close();
	task_space_velocity_response_log.close();
	task_space_velocity_error_log.close();
}

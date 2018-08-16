#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "../include/Eigen/Dense"
#include "../include/franka_panda_gazebo_task_space_velocity_controller_rpy.h"
#include "../include/FrankaPandaArmModel.h"
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
	for(int k=0; k<7; k++)
		joint_position(k)  = _msg->data[k];	
}
void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	for(int k=0; k<7; k++)
		joint_velocity(k)  = _msg->data[k];	
}

int main(int argc, char **argv){
  ros::init(argc, argv, "franka_panda_gazebo_controller");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub = n.advertise<std_msgs::Float32MultiArray>("/franka_panda_arm/joint_command/torque", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/franka_panda_arm/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/franka_panda_arm/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(500);
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
  
	
	velocity_bE_desired << 0, 0, 0, 0, 0.1, 0;
	Kv = 200;
	
	open_logs();
  while (ros::ok() and ros::Time::now().toSec()<10.0 ){
  //while (ros::ok()){
  	time_now = ros::Time::now().toSec();
  	cout << "time_now = " << time_now  << endl;
  	geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"base");
  	//geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"effector");
  	
  	///
  	// Computing Joint Torque Command
  	// 1. control in joint space
  	//joint_velocity_desired  = Pinv_damped(geometric_jacobian, 0.001)*velocity_bE_desired;
  	//joint_velocity_error = joint_velocity_desired-joint_velocity;
  	
  	// 2. control in task space (better response)
  	velocity_bE = geometric_jacobian*joint_velocity;
  	velocity_error = velocity_bE_desired-velocity_bE;
  	joint_velocity_error = Pinv_damped(geometric_jacobian, 0.001)*velocity_error;
  	
  	inertia_matrix = arm_inertia_matrix(joint_position);
  	viscous_friction_torque = arm_viscous_friction_torque(joint_velocity);
  	static_friction_torque = arm_static_friction_torque(joint_velocity);
  	coriolis_centrifugal_torque = arm_coriolis_centrifugal_torque(joint_position, joint_velocity);
  	joint_torque_command  = inertia_matrix*Kv*joint_velocity_error + viscous_friction_torque + static_friction_torque + coriolis_centrifugal_torque;
  	
  	///
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<7; k++)
  		torque_command.data.push_back(joint_torque_command(k));
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
	joint_velocity_response_log << time_now << " " << joint_velocity.transpose() <<  endl;
	joint_velocity_error_log << time_now << " " << joint_velocity_error.transpose() <<  endl;
	
	joint_torque_command_log << time_now << " " << joint_torque_command.transpose() <<  endl;
	
	task_space_velocity_command_log << time_now << " " << velocity_bE_desired.transpose() <<  endl;
	task_space_velocity_response_log << time_now << " " << velocity_bE.transpose() <<  endl;
	task_space_velocity_error_log << time_now << " " << velocity_error.transpose() <<  endl;
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

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/FrankaPandaArmModel.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include "../include/franka_panda_arm_gazebo_control.h"

using namespace std;
using namespace Eigen;
using namespace franka_panda_gazebo_controller;

void log_data(void);
void open_logs(void);
void close_logs(void);

void GetJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_position_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		joint_position(k)  = _msg->data[k+1];	
}
void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	joint_velocity_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		joint_velocity(k)  = _msg->data[k+1];	
}

int main(int argc, char **argv){
  string arm_name;
  arm_name = argv[2];
  
  ros::init(argc, argv, "franka_panda_gazebo_controller");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub   = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/torque", 10);
	ros::Publisher  JointVelocityCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/velocity", 10);
	ros::Publisher  JointPositionCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/position", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+arm_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+arm_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(500);
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
  
	joint_position_desired  << pi/6, pi/6, pi/6, -pi/6, pi/6, pi/6, pi/6;
	joint_velocity_desired  << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07;
	//joint_position_desired  << 0, 0, 0, 0, 0, 0, 0;
	joint_position_init = joint_position;
	cout << "initial joint vector = " << joint_position.transpose();
	
	start_time = ros::Time::now().toSec();
	trajectory_duration = 5.0;
	end_time = start_time + trajectory_duration;
	Kp = 110;
	Kv = 5;
	
	open_logs();
	while (ros::ok() and ros::Time::now().toSec()<10.0 ){
  //while (ros::ok()){  
  //while (ros::ok() and ros::Time::now().toSec()<5.3 ){
  	time_now = ros::Time::now().toSec();
  	cout << "time_now = " << time_now  << endl;
  	
  	arm_DGM  = arm_direct_geometric_model(joint_position);
  	//cout << "Franka Panda DGM = "  << endl << arm_DGM  << endl;
  	
  	for(int k=0; k<7; k++){
  		trajectory = OnlineMP_L5B(start_time, end_time, time_now, joint_position_init(k), joint_position_desired(k));
  		joint_position_traj(k) = trajectory(0);
  		joint_velocity_traj(k) = trajectory(1);
  	}
  	
  	// Computing Joint Torque Command
  	//joint_position_error = joint_position_traj-joint_position;
  	joint_velocity_error = joint_velocity_traj-joint_velocity;
  	joint_velocity_desired=joint_velocity_traj;
  	//joint_torque_command  = joint_acceleration_traj + Kp*joint_position_error + Kv*joint_velocity_error;
  	//joint_torque_command  = joint_acceleration_traj + Kp*joint_position_error;
  	
  	//joint_velocity_error = joint_velocity_desired-joint_velocity;
  	//joint_torque_command  = Kp*joint_velocity_error;
  	
  	inertia_matrix = arm_inertia_matrix(joint_position);
  	viscous_friction_torque = arm_viscous_friction_torque(joint_velocity);
  	static_friction_torque = arm_static_friction_torque(joint_velocity);
  	coriolis_centrifugal_torque = arm_coriolis_centrifugal_torque(joint_position, joint_velocity);
  	gravity_compensation_torque = arm_gravity_compensation_torque(joint_position);
  	joint_torque_command  = inertia_matrix*Kp*joint_velocity_error + viscous_friction_torque + static_friction_torque + coriolis_centrifugal_torque + gravity_compensation_torque;
  	
  	cout << "JointPosition      = " << joint_position.transpose()  << endl;
  	//cout << "JointTorqueCommand = " << joint_torque_command.transpose()  << endl;
  	
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<7; k++)
  		torque_command.data.push_back(joint_torque_command(k));
  	//JointTorqueCommandPub.publish(torque_command);
  	
  	// Sending Joint VElocity Command
  	std_msgs::Float32MultiArray velocity_command;
  	velocity_command.data.clear();
  	for(int k=0; k<7; k++)
  		velocity_command.data.push_back(joint_velocity_desired(k)); // equivalent to 
  	JointVelocityCommandPub.publish(velocity_command);
  	
  	// Data logging
  	log_data();
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
  close_logs();
	
  return 0;
}




void log_data(void){
	joint_position_command_log << joint_position_sim_time << " " << joint_position_traj.transpose() <<  endl;
	joint_position_response_log << joint_position_sim_time << " " << joint_position.transpose() <<  endl;
	joint_position_error_log << joint_position_sim_time << " " << joint_position_error.transpose() <<  endl;
	
	joint_velocity_command_log << joint_velocity_sim_time << " " << joint_velocity_desired.transpose() <<  endl;
	joint_velocity_response_log << joint_velocity_sim_time << " " << joint_velocity.transpose() <<  endl;
	joint_velocity_error_log << joint_velocity_sim_time << " " << joint_velocity_error.transpose() <<  endl;
	
	joint_torque_command_log << time_now << " " << joint_torque_command.transpose() <<  endl;
}

void open_logs(void){
	joint_position_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_position_command_log.txt");
	joint_position_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_position_response_log.txt");
	joint_position_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_position_error_log.txt");
	
	joint_velocity_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_velocity_command_log.txt");
	joint_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_velocity_response_log.txt");
	joint_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_velocity_error_log.txt");
	
	joint_torque_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_torque_command_log.txt");
}
void close_logs(void){
	joint_position_command_log.close();
	joint_position_response_log.close();
	joint_position_error_log.close();
	
	joint_velocity_command_log.close();
	joint_velocity_response_log.close();
	joint_velocity_error_log.close();
	
	joint_torque_command_log.close();
}

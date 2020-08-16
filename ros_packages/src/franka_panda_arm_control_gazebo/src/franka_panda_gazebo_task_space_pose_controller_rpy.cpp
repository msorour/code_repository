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
  
  ros::init(argc, argv, "franka_panda_gazebo_task_space_pose_controller_rpy_node");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub   = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/torque", 10);
	ros::Publisher  JointVelocityCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/velocity", 10);
	ros::Publisher  JointPositionCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/position", 10);
  ros::Subscriber JointPositionStateSub   = n.subscribe("/"+arm_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub   = n.subscribe("/"+arm_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(500);
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
  
	
	pose_rpy_bE_desired << 0.4, 0.2, 0.5, pi/2, 0, 0;
	
	start_time = ros::Time::now().toSec();
	trajectory_duration = 5.0;
	end_time = start_time + trajectory_duration;
	Kp = 13.7;
	Kv = 100;
	
	arm_DGM  = arm_direct_geometric_model(joint_position);
	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
 	cout << "initial pose_rpy_bE = " << pose_rpy_bE.transpose()  << endl;
 	
 	time_now = ros::Time::now().toSec();
 	time_past = time_now;
	
	geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"base");
	geometric_jacobian_past = geometric_jacobian;
	open_logs();
  //while (ros::ok() and ros::Time::now().toSec()<0.03 ){
  while (ros::ok()){
  	time_now = ros::Time::now().toSec();
  	cout << "time_now = " << time_now  << endl;
  	
  	arm_DGM  = arm_direct_geometric_model(joint_position);
  	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
  	pose_error = pose_rpy_bE_desired-pose_rpy_bE;
  	geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"base");
  	analytic_jacobian = geometric_to_analytic_jacobian_rpy(pose_rpy_bE)*geometric_jacobian;
  	
  	cout << "joint_position = " << joint_position.transpose()  << endl;
  	cout << "pose_rpy_bE = " << pose_rpy_bE.transpose()  << endl;
  	
  	/*
  	for(int k=0; k<7; k++){
  		trajectory = OnlineMP_L5B(start_time, end_time, time_now, joint_position_init(k), joint_position_desired(k));
  		joint_position_traj(k) = trajectory(0);
  		joint_velocity_traj(k) = trajectory(1);
  	}
  	*/
  	//geometric_jacobian_derivative = (geometric_jacobian-geometric_jacobian_past)/(time_now-time_past);
  	
  	
  	// Computing Joint Torque Command
  	velocity_bE_desired = Kp*pose_error;
  	joint_velocity_desired = Pinv_damped(analytic_jacobian, 0.001)*velocity_bE_desired;
  	//joint_velocity_desired = Pinv_damped(geometric_jacobian, 0.001)*velocity_bE_desired;
  	joint_acceleration_desired = Kv*(joint_velocity_desired-joint_velocity);
  	
  	inertia_matrix = arm_inertia_matrix(joint_position);
  	viscous_friction_torque = arm_viscous_friction_torque(joint_velocity);
  	static_friction_torque = arm_static_friction_torque(joint_velocity);
  	coriolis_centrifugal_torque = arm_coriolis_centrifugal_torque(joint_position, joint_velocity);
  	gravity_compensation_torque = arm_gravity_compensation_torque(joint_position);
  	joint_torque_command  = inertia_matrix*joint_acceleration_desired + viscous_friction_torque + static_friction_torque + coriolis_centrifugal_torque + gravity_compensation_torque;
  	//joint_torque_command  = arm_gravity_compensation_torque(joint_position);
  	//cout << "arm_gravity_compensation_torque = " << joint_torque_command.transpose()  << endl;
  	
  	
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<7; k++)
  		torque_command.data.push_back(joint_torque_command(k));
  	JointTorqueCommandPub.publish(torque_command);
  	
  	/*
  	// Sending Joint VElocity Command
  	std_msgs::Float32MultiArray velocity_command;
  	velocity_command.data.clear();
  	for(int k=0; k<7; k++)
  		velocity_command.data.push_back(joint_velocity_desired(k)); // equivalent to 
  	JointVelocityCommandPub.publish(velocity_command);
  	*/
  	
  	// Data logging
  	log_data();
  	
  	// For next iteration
  	time_past = time_now;
  	geometric_jacobian_past = geometric_jacobian;
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
  close_logs();
	
  return 0;
}









void log_data(void){
  joint_position_command_log << time_now << " " << joint_position.transpose() <<  endl;
	joint_position_response_log << joint_position_sim_time << " " << joint_position.transpose() <<  endl;
	joint_position_error_log << joint_position_sim_time << " " << joint_position.transpose() <<  endl;
	
	joint_velocity_command_log << time_now << " " << joint_velocity_desired.transpose() <<  endl;
	joint_velocity_response_log << joint_velocity_sim_time << " " << joint_velocity.transpose() <<  endl;
	joint_velocity_error_log << joint_velocity_sim_time << " " << joint_velocity_error.transpose() <<  endl;
	
	joint_torque_command_log << time_now << " " << joint_torque_command.transpose() <<  endl;
	
	task_space_pose_command_log << time_now << " " << pose_rpy_bE_desired.transpose() <<  endl;
	task_space_pose_response_log << joint_position_sim_time << " " << pose_rpy_bE.transpose() <<  endl;
	task_space_pose_error_log << joint_position_sim_time << " " << pose_error.transpose() <<  endl;
	
	task_space_velocity_desired_log << time_now << " " << velocity_bE_desired.transpose() <<  endl;
	task_space_velocity_traj_log << time_now << " " << velocity_bE_traj.transpose() <<  endl;
	task_space_velocity_response_log << joint_velocity_sim_time << " " << velocity_bE.transpose() <<  endl;
	task_space_velocity_error_log << joint_velocity_sim_time << " " << velocity_error.transpose() <<  endl;
}

void open_logs(void){
	joint_position_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_position_command_log.txt");
	joint_position_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_position_response_log.txt");
	joint_position_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_position_error_log.txt");
	
	joint_velocity_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_velocity_command_log.txt");
	joint_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_velocity_response_log.txt");
	joint_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_velocity_error_log.txt");
	
	joint_torque_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/joint_torque_command_log.txt");
	
	task_space_pose_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_pose_command_log.txt");
	task_space_pose_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_pose_response_log.txt");
	task_space_pose_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_pose_error_log.txt");
	
	task_space_velocity_desired_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_velocity_desired_log.txt");
	task_space_velocity_traj_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_velocity_traj_log.txt");
	task_space_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_velocity_response_log.txt");
	task_space_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_control_gazebo/logs/task_space_velocity_error_log.txt");
}
void close_logs(void){
	joint_position_command_log.close();
	joint_position_response_log.close();
	joint_position_error_log.close();
	
	joint_velocity_command_log.close();
	joint_velocity_response_log.close();
	joint_velocity_error_log.close();
	
	joint_torque_command_log.close();
	
	task_space_pose_command_log.close();
	task_space_pose_response_log.close();
	task_space_pose_error_log.close();
	
	task_space_velocity_desired_log.close();
	task_space_velocity_traj_log.close();
	task_space_velocity_response_log.close();
	task_space_velocity_error_log.close();
}

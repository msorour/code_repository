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
		joint_position(k)  = _msg->data[k+1];
	for(int k=0; k<2; k++)
		gripper_joint_position(k) = _msg->data[k+7+1];
}
void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	joint_velocity_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		joint_velocity(k)  = _msg->data[k+1];
	for(int k=0; k<2; k++)
		gripper_joint_velocity(k) = _msg->data[k+7+1];
}

int main(int argc, char **argv){
  string model_name;
  model_name = argv[2];
  
  ros::init(argc, argv, "franka_panda_arm_plus_own_gripper_gazebo_task_space_pose_controller_rpy");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub   = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/torque", 10);
	ros::Publisher  JointVelocityCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/velocity", 10);
	ros::Publisher  JointPositionCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/position", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+model_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+model_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(500);
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
  
	
	pose_rpy_bE_desired << 0.47, 0.06, 0.3, -pi/2, -pi/2, 0;
	gripper_full_open_position << 0.035, 0.035;
	gripper_closed_position << 0.01, 0.01;
	gripper_joint_position_desired = gripper_full_open_position;
	
	start_time = ros::Time::now().toSec();
	trajectory_duration = 5.0;
	end_time = start_time + trajectory_duration;
	Kp = 1.7;
	Kv = 200;
	Kp_gripper = 3.7;
	
	arm_DGM  = arm_direct_geometric_model(joint_position, "panda_gripper");
	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
 	cout << "initial pose_rpy_bE = " << pose_rpy_bE.transpose()  << endl;
 	pose_rpy_bE_init = pose_rpy_bE;
 	
 	time_now = ros::Time::now().toSec();
 	time_past = time_now;
	
	geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"base");
	geometric_jacobian_past = geometric_jacobian;
	open_logs();
  //while (ros::ok() and ros::Time::now().toSec()<20 ){
  while (ros::ok()){
  	time_now = ros::Time::now().toSec();
  	//cout << "time_now = " << time_now  << endl;
  	
  	
  	if(time_now>11.0)
  	  pose_rpy_bE_desired << 0.47, 0.063, 0.11, -pi/2, -pi/2, 0;
  	if(time_now>14.0)
  	  gripper_joint_position_desired = gripper_closed_position;
  	if(time_now>17.0)
  	  pose_rpy_bE_desired << 0.47, 0.063, 0.23, -pi/2, -pi/2, 0;
  	
  	
  	/*
  	////// computing current pose
  	//VectorXd dummy_joint(7);
  	//dummy_joint << 0, 0, 0, 0, 0, 0, 0;
  	//arm_DGM  = arm_direct_geometric_model(dummy_joint, "panda_gripper");
  	arm_DGM  = arm_direct_geometric_model(joint_position, "panda_gripper");
  	//cout << "Franka Panda DGM = "  << endl << arm_DGM  << endl;
  	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
  	//pose_error = pose_rpy_bE_desired-pose_rpy_bE;
  	cout << "joint_position = " << joint_position.transpose()  << endl;
  	cout << "pose_rpy_bE = " << pose_rpy_bE.transpose()  << endl;
  	
  	////// computing current task space velocity
  	geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"base");
  	analytic_jacobian = geometric_to_analytic_jacobian_rpy(pose_rpy_bE)*geometric_jacobian;
  	velocity_bE = analytic_jacobian*joint_velocity;
  	
  	////// trajectory generation
  	for(int k=0; k<6; k++){
  		trajectory = OnlineMP_L5B(start_time, end_time, time_now, pose_rpy_bE_init(k), pose_rpy_bE_desired(k));
  		pose_rpy_bE_traj(k) = trajectory(0);
  		velocity_bE_traj(k) = trajectory(1);
  		acceleration_bE_traj(k) = trajectory(2);
  	}
  	
  	////// error in pose, velocity tracking
  	//pose_error = pose_rpy_bE_traj-pose_rpy_bE;
  	//velocity_error = velocity_bE_traj-velocity_bE;
  	pose_error = pose_rpy_bE_desired-pose_rpy_bE;
  	
  	////// Computing Joint Torque Command
  	//velocity_bE_desired = Kp*pose_error;
  	//joint_velocity_desired = Pinv_damped(analytic_jacobian, 0.001)*velocity_bE_desired;
  	//joint_velocity_desired = Pinv_damped(geometric_jacobian, 0.001)*velocity_bE_desired;
  	//joint_acceleration_desired = Kv*(joint_velocity_desired-joint_velocity);
  	geometric_jacobian_derivative = (geometric_jacobian-geometric_jacobian_past)/(time_now-time_past);
  	Kp = 10;
  	acceleration_bE_desired = Kp*pose_error;
  	joint_acceleration_desired = Pinv_damped(analytic_jacobian, 0.001)*(acceleration_bE_desired-geometric_jacobian_derivative*joint_velocity);
  	
  	inertia_matrix = arm_inertia_matrix(joint_position);
  	viscous_friction_torque = arm_viscous_friction_torque(joint_velocity);
  	static_friction_torque = arm_static_friction_torque(joint_velocity);
  	coriolis_centrifugal_torque = arm_coriolis_centrifugal_torque(joint_position, joint_velocity);
  	joint_torque_command  = inertia_matrix*joint_acceleration_desired + viscous_friction_torque + static_friction_torque + coriolis_centrifugal_torque;
  	*/
  	
  	
  	
  	arm_DGM  = arm_direct_geometric_model(joint_position, "panda_gripper");
  	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
  	pose_error = pose_rpy_bE_desired-pose_rpy_bE;
  	geometric_jacobian = arm_geometric_jacobian_matrix(joint_position,"base");
  	analytic_jacobian = geometric_to_analytic_jacobian_rpy(pose_rpy_bE)*geometric_jacobian;
  	
  	velocity_bE_desired = Kp*pose_error;
  	arm_joint_velocity_desired = Pinv_damped(analytic_jacobian, 0.001)*velocity_bE_desired;
  	
  	gripper_joint_position_error = gripper_joint_position_desired - gripper_joint_position;
  	gripper_joint_velocity_desired = Kp_gripper*gripper_joint_position_error;
  	
  	
  	
  	
  	
  	// torque override for testing
  	//joint_torque_command  = arm_gravity_compensation_torque(joint_position);
  	//gripper_joint_torque_command << 0,0;
  	
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<7; k++)
  		torque_command.data.push_back(joint_torque_command(k));
  	for(int k=0; k<2; k++)
  		torque_command.data.push_back(gripper_joint_torque_command(k));
  	//JointTorqueCommandPub.publish(torque_command);
  	
  	// Sending Joint VElocity Command
  	std_msgs::Float32MultiArray velocity_command;
  	velocity_command.data.clear();
  	for(int k=0; k<7; k++)
  		velocity_command.data.push_back(arm_joint_velocity_desired(k));
  	for(int k=0; k<2; k++)
  		velocity_command.data.push_back(gripper_joint_velocity_desired(k));
  	JointVelocityCommandPub.publish(velocity_command);
  	
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
	joint_velocity_response_log << time_now << " " << joint_velocity.transpose() <<  endl;
	joint_velocity_error_log << time_now << " " << joint_velocity_error.transpose() <<  endl;
	
	joint_torque_command_log << time_now << " " << joint_torque_command.transpose() <<  endl;
	
	task_space_pose_command_log << time_now << " " << pose_rpy_bE_traj.transpose() <<  endl;
	task_space_pose_response_log << time_now << " " << pose_rpy_bE.transpose() <<  endl;
	task_space_pose_error_log << time_now << " " << pose_error.transpose() <<  endl;
	
	task_space_velocity_command_log << time_now << " " << velocity_bE_traj.transpose() <<  endl;
	task_space_velocity_response_log << time_now << " " << velocity_bE.transpose() <<  endl;
	task_space_velocity_error_log << time_now << " " << velocity_error.transpose() <<  endl;
	
	task_space_acceleration_command_log << time_now << " " << acceleration_bE_traj.transpose() <<  endl;
}

void open_logs(void){
	joint_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/joint_velocity_response_log.txt");
	joint_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/joint_velocity_error_log.txt");
	
	joint_torque_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/joint_torque_command_log.txt");
	
	task_space_pose_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_command_log.txt");
	task_space_pose_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_response_log.txt");
	task_space_pose_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_error_log.txt");
	
	task_space_velocity_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_velocity_command_log.txt");
	task_space_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_velocity_response_log.txt");
	task_space_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_velocity_error_log.txt");
	
	task_space_acceleration_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_acceleration_command_log.txt");
	task_space_acceleration_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_acceleration_response_log.txt");
	task_space_acceleration_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_acceleration_log.txt");
}
void close_logs(void){
	joint_velocity_response_log.close();
	joint_velocity_error_log.close();
	
	joint_torque_command_log.close();
	
	task_space_pose_command_log.close();
	task_space_pose_response_log.close();
	task_space_pose_error_log.close();
	
	task_space_velocity_command_log.close();
	task_space_velocity_response_log.close();
	task_space_velocity_error_log.close();
	
	task_space_acceleration_command_log.close();
	task_space_acceleration_response_log.close();
	task_space_acceleration_error_log.close();
}

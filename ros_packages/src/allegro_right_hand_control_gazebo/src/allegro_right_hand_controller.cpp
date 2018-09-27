#include <iostream>
#include <fstream>
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/AllegroRightHandModel.h"
#include "../include/allegro_right_hand.h"

using namespace std;
using namespace Eigen;
using namespace allegro_right_hand;

void GetJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_position_sim_time = _msg->data[0];
	for(int k=0; k<4; k++)
		index_joint_position(k)  = _msg->data[k+1];
	for(int k=0; k<4; k++)
		middle_joint_position(k) = _msg->data[k+4+1];
	for(int k=0; k<4; k++)
		pinky_joint_position(k)  = _msg->data[k+8+1];
	for(int k=0; k<4; k++)
		thumb_joint_position(k)  = _msg->data[k+12+1];
}

void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	joint_velocity_sim_time = _msg->data[0];
	for(int k=0; k<4; k++)
		index_joint_velocity(k)  = _msg->data[k+1];
	for(int k=0; k<4; k++)
		middle_joint_velocity(k) = _msg->data[k+4+1];
	for(int k=0; k<4; k++)
		pinky_joint_velocity(k)  = _msg->data[k+8+1];
	for(int k=0; k<4; k++)
		thumb_joint_velocity(k)  = _msg->data[k+12+1];
}

int main(int argc, char **argv){
  string model_name;
  model_name = argv[2];
  
  ros::init(argc, argv, "allegro_right_hand_controller");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub   = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/torque", 10);
	ros::Publisher  JointVelocityCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/velocity", 10);
	ros::Publisher  JointPositionCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/position", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+model_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+model_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(100);
  
  // Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
	
	//r_index << 0,0,0,0;     d_index << 0,0,D3i,D4i;   alpha_index << 0,pi/2,0,0;    theta_index << 0,pi/2,0,0;
  //r_thumb << 0,RL2t,0,0;  d_thumb << 0,D2t,0,D4t;   alpha_thumb << 0,pi/2,pi/2,0; theta_thumb << 0,pi/2,pi/2,0;
  //cout << "index_DGM_symbolic   = " << endl << index_DGM << endl;
  //cout << "index_DGM_iterative  = " << endl << general_transformation_matrix(4,0,r_index,d_index,alpha_index,theta_index+index_joint_position) << endl;
    
	Kp = 17.7;
	lambda = -30.9;
	
	while (ros::ok()){
  	
  	position_Ptt_desired << 0.04, 0.0, 0.07;
  	position_Pit_desired << 0.07, 0.0, 0.10;
  	position_Pmt_desired << 0.07, 0.0, 0.10;
  	position_Ppt_desired << 0.07, -0.0, 0.10;
  	
  	thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
  	index_DGM  = finger_direct_geometric_model("index",index_joint_position);
  	middle_DGM = finger_direct_geometric_model("middle",middle_joint_position);
  	pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);
  	
  	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
  	pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
  	pose_rpy_Pmt = transformation_matrix_to_pose_rpy(middle_DGM);
  	pose_rpy_Ppt = transformation_matrix_to_pose_rpy(pinky_DGM);
  	
  	thumb_position_jacobian  = finger_position_jacobian("thumb",thumb_joint_position);
  	index_position_jacobian  = finger_position_jacobian("index",index_joint_position);
  	middle_position_jacobian = finger_position_jacobian("middle",middle_joint_position);
  	pinky_position_jacobian  = finger_position_jacobian("pinky",pinky_joint_position);
  	
  	position_Ptt << pose_rpy_Ptt(0), pose_rpy_Ptt(1), pose_rpy_Ptt(2);
  	position_Pit << pose_rpy_Pit(0), pose_rpy_Pit(1), pose_rpy_Pit(2);
  	position_Pmt << pose_rpy_Pmt(0), pose_rpy_Pmt(1), pose_rpy_Pmt(2);
  	position_Ppt << pose_rpy_Ppt(0), pose_rpy_Ppt(1), pose_rpy_Ppt(2);
  	
  	position_Ptt_error = position_Ptt_desired-position_Ptt;
  	position_Pit_error = position_Pit_desired-position_Pit;
  	position_Pmt_error = position_Pmt_desired-position_Pmt;
  	position_Ppt_error = position_Ppt_desired-position_Ppt;
  	
  	velocity_Ptt_desired_3d = Kp*position_Ptt_error;
  	velocity_Pit_desired_3d = Kp*position_Pit_error;
  	velocity_Pmt_desired_3d = Kp*position_Pmt_error;
  	velocity_Ppt_desired_3d = Kp*position_Ppt_error;
  	
  	null_space_projector = I4 - Pinv_damped(thumb_position_jacobian, 0.001)*thumb_position_jacobian;
  	task_gradient = avoid_joint_limit_task_gradient(thumb_joint_position, thumb_joint_safe_mean, thumb_joint_safe_range);
  	thumb_joint_velocity_command  = Pinv_damped(thumb_position_jacobian, 0.001)*velocity_Ptt_desired_3d + lambda*null_space_projector*task_gradient;
  	
  	null_space_projector = I4 - Pinv_damped(index_position_jacobian, 0.001)*index_position_jacobian;
  	task_gradient = avoid_joint_limit_task_gradient(index_joint_position, index_joint_safe_mean, index_joint_safe_range);
  	index_joint_velocity_command  = Pinv_damped(index_position_jacobian, 0.001)*velocity_Pit_desired_3d + lambda*null_space_projector*task_gradient;
  	
  	null_space_projector = I4 - Pinv_damped(middle_position_jacobian, 0.001)*middle_position_jacobian;
  	task_gradient = avoid_joint_limit_task_gradient(middle_joint_position, middle_joint_safe_mean, middle_joint_safe_range);
  	middle_joint_velocity_command  = Pinv_damped(middle_position_jacobian, 0.001)*velocity_Pmt_desired_3d + lambda*null_space_projector*task_gradient;
  	
  	null_space_projector = I4 - Pinv_damped(pinky_position_jacobian, 0.001)*pinky_position_jacobian;
  	task_gradient = avoid_joint_limit_task_gradient(pinky_joint_position, pinky_joint_safe_mean, pinky_joint_safe_range);
  	pinky_joint_velocity_command  = Pinv_damped(pinky_position_jacobian, 0.001)*velocity_Ppt_desired_3d + lambda*null_space_projector*task_gradient;
  	
  	cout << "pose_rpy_Ptt = "  << pose_rpy_Ptt.transpose() << endl;
  	cout << "pose_rpy_Pit = "  << pose_rpy_Pit.transpose() << endl;
  	cout << "pose_rpy_Pmt = "  << pose_rpy_Pmt.transpose() << endl;
  	cout << "pose_rpy_Ppt = "  << pose_rpy_Ppt.transpose() << endl;
  	
  	cout << endl;
  	
  	
  	
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<4; k++)
  		torque_command.data.push_back(index_joint_torque_command(k));
  	for(int k=0; k<4; k++)
  		torque_command.data.push_back(middle_joint_torque_command(k));
  	for(int k=0; k<4; k++)
  		torque_command.data.push_back(pinky_joint_torque_command(k));
  	for(int k=0; k<4; k++)
  		torque_command.data.push_back(thumb_joint_torque_command(k));
  	//JointTorqueCommandPub.publish(torque_command);
  	
  	// Sending Joint Velocity Command
  	std_msgs::Float32MultiArray velocity_command;
  	velocity_command.data.clear();
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(index_joint_velocity_command(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(middle_joint_velocity_command(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(pinky_joint_velocity_command(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(thumb_joint_velocity_command(k));
  	JointVelocityCommandPub.publish(velocity_command);
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

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
  ros::Rate loop_rate(200);
	
	r_index << 0,0,0,0;     d_index << 0,0,D3i,D4i;   alpha_index << 0,pi/2,0,0;    theta_index << 0,pi/2,0,0;
  r_thumb << 0,RL2t,0,0;  d_thumb << 0,D2t,0,D4t;   alpha_thumb << 0,pi/2,pi/2,0; theta_thumb << 0,pi/2,pi/2,0;
    
	thumb_joint_position_desired  << 0.7, 0.7, 0.7, 0.7;
	index_joint_position_desired  << pi/3, pi/3, 0.0, 0.0;
	middle_joint_position_desired << 0.0, 0.0, 0.0, 0.0;
	pinky_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	
	
	while (ros::ok()){
  	
  	/*
  	index_DGM  = finger_direct_geometric_model("index",index_joint_position);
  	middle_DGM = finger_direct_geometric_model("middle",middle_joint_position);
  	pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);
  	
  	cout << "thumb DGM = "  << endl << thumb_DGM  << endl;
  	cout << "index DGM = "  << endl << index_DGM  << endl;
  	cout << "middle DGM = " << endl << middle_DGM << endl;
  	cout << "pinky DGM = "  << endl << pinky_DGM  << endl << endl;
  	
  	// Computing Joint Torque Command
  	thumb_joint_torque_command  = 7.7*(thumb_joint_position_desired-thumb_joint_position);
  	index_joint_torque_command  = 7.7*(index_joint_position_desired-index_joint_position);
  	middle_joint_torque_command = 7.7*(middle_joint_position_desired-middle_joint_position);
  	pinky_joint_torque_command  = 7.7*(pinky_joint_position_desired-pinky_joint_position);
  	*/
  	
  	
  	
  	
  	/*
  	pose_rpy_Ptt_desired << 0.03, 0.1, 0.05, pose_rpy_Ptt(3), pose_rpy_Ptt(4), pose_rpy_Ptt(5);
  	thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
  	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
  	pose_rpy_Ptt_error = pose_rpy_Ptt_desired - pose_rpy_Ptt;
  	geometric_jacobian = finger_geometric_jacobian_matrix("thumb",thumb_joint_position);
  	analytic_jacobian = geometric_to_analytic_jacobian_rpy(pose_rpy_Ptt)*geometric_jacobian;
  	velocity_Ptt_desired = Kp*pose_rpy_Ptt_error;
  	thumb_joint_velocity_command = Pinv_damped(analytic_jacobian, 0.001)*velocity_Ptt_desired;
  	*/
  	
  	
  	/*
  	pose_rpy_Pit_desired << 0.07, 0.07, 0.1, pose_rpy_Pit(3), pose_rpy_Pit(4), pose_rpy_Pit(5);
  	index_DGM  = finger_direct_geometric_model("index",index_joint_position);
  	pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
  	pose_rpy_Pit_error = pose_rpy_Pit_desired - pose_rpy_Pit;
  	geometric_jacobian = finger_geometric_jacobian_matrix("index",index_joint_position);
  	analytic_jacobian = geometric_to_analytic_jacobian_rpy(pose_rpy_Pit)*geometric_jacobian;
  	velocity_Pit_desired = Kp*pose_rpy_Pit_error;
  	//velocity_Pit_desired << 0.01,0,0,0,0,0;
  	index_joint_velocity_command = Pinv_damped(analytic_jacobian, 0.001)*velocity_Pit_desired;
  	*/
  	
  	//cout << "pose_rpy_Pit                 = "  << pose_rpy_Pit.transpose() << endl;
  	//cout << "pose_rpy_Pit_error           = "  << pose_rpy_Pit_error.transpose() << endl;
  	//cout << "pose_rpy_Pit_error norm      = "  << pose_rpy_Pit_error.norm() << endl;
  	//cout << "index_joint_position         = "  << index_joint_position.transpose() << endl;
  	
  	
  	
  	/*
  	position_Pit_desired << 0.07, 0.07, 0.1;
  	index_DGM  = finger_direct_geometric_model("index",index_joint_position);
  	pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
  	position_Pit << pose_rpy_Pit(0), pose_rpy_Pit(1), pose_rpy_Pit(2);
  	position_Pit_error = position_Pit_desired-position_Pit;
  	geometric_jacobian = finger_geometric_jacobian_matrix("index",index_joint_position);
  	index_position_jacobian << geometric_jacobian(0,0), geometric_jacobian(0,1), geometric_jacobian(0,2), geometric_jacobian(0,3),
  	                           geometric_jacobian(1,0), geometric_jacobian(1,1), geometric_jacobian(1,2), geometric_jacobian(1,3),
  	                           geometric_jacobian(2,0), geometric_jacobian(2,1), geometric_jacobian(2,2), geometric_jacobian(2,3);
  	*/
  	
  	//velocity_Pit_desired_3d = Kp*position_Pit_error;
  	//index_joint_velocity_command = Pinv_damped(index_position_jacobian, 0.001)*velocity_Pit_desired_3d;
  	
  	// new
  	Kp = 9.7;
  	position_Ptt_desired << 0.04, 0.0, 0.07;
  	position_Pit_desired << 0.07, 0.05, 0.10;
  	position_Pmt_desired << 0.07, 0.0, 0.10;
  	position_Ppt_desired << 0.07, -0.05, 0.10;
  	
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
  	
  	thumb_joint_velocity_command  = Pinv_damped(thumb_position_jacobian, 0.001)*velocity_Ptt_desired_3d;
  	index_joint_velocity_command  = Pinv_damped(index_position_jacobian, 0.001)*velocity_Pit_desired_3d;
  	middle_joint_velocity_command = Pinv_damped(middle_position_jacobian, 0.001)*velocity_Pmt_desired_3d;
  	pinky_joint_velocity_command  = Pinv_damped(pinky_position_jacobian, 0.001)*velocity_Ppt_desired_3d;
  	
  	cout << "pose_rpy_Ptt = "  << pose_rpy_Ptt.transpose() << endl;
  	cout << "pose_rpy_Pit = "  << pose_rpy_Pit.transpose() << endl;
  	cout << "pose_rpy_Pmt = "  << pose_rpy_Pmt.transpose() << endl;
  	cout << "pose_rpy_Ppt = "  << pose_rpy_Ppt.transpose() << endl;
  	
  	
  	//cout << "index_DGM_symbolic   = " << endl << index_DGM << endl;
  	//cout << "index_DGM_iterative  = " << endl << general_transformation_matrix(4,0,r_index,d_index,alpha_index,theta_index+index_joint_position) << endl;
  	
  	//cout << "thumb_DGM_symbolic   = " << endl << thumb_DGM << endl;
  	//cout << "thumb_DGM_iterative  = " << endl << general_transformation_matrix(4,0,r_thumb,d_thumb,alpha_thumb,theta_thumb+thumb_joint_position) << endl;
  	
  	//cout << "geometric_jacobian           = " << endl << geometric_jacobian << endl;
  	//cout << "analytic_jacobian            = " << endl << geometric_jacobian << endl;
  	//cout << "analytic_jacobian_inverse    = " << endl << Pinv_damped(analytic_jacobian, 0.001) << endl;
  	//cout << "velocity_Pit_desired         = " << velocity_Pit_desired.transpose()  << endl;
  	//cout << "index_joint_velocity_command = " << index_joint_velocity_command.transpose()  << endl;
  	
  	//cout << "thumb_joint_position         = "  << thumb_joint_position.transpose()  << endl;
  	//cout << "thumb pose rpy               = "  << pose_rpy_Ptt.transpose()  << endl;
  	//cout << "pose_rpy_Ptt_error           = "  << pose_rpy_Ptt_error.transpose()  << endl;
  	//cout << "pose_rpy_Ptt_error norm      = "  << pose_rpy_Ptt_error.norm()  << endl;
  	//cout << "velocity_Ptt_desired         = "  << velocity_Ptt_desired.transpose()  << endl;
  	//cout << "thumb_joint_velocity_command = "  << thumb_joint_velocity_command.transpose()  << endl;
  	
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
  	
  	
  	// Sending Joint VElocity Command
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

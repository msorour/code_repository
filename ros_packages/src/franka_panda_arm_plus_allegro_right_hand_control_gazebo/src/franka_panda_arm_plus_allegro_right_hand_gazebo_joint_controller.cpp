#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/FrankaPandaArmModel.h"
#include "../../../include/AllegroRightHandModel.h"
#include "../include/franka_panda_arm_plus_allegro_right_hand.h"
#include <iostream>
#include <fstream>
#include <time.h>

using namespace std;
using namespace Eigen;
using namespace franka_panda_gazebo_controller;

void GetJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  joint_position_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		arm_joint_position(k)    = _msg->data[k+1];
	for(int k=0; k<4; k++)
		index_joint_position(k)  = _msg->data[k+7+1];
	for(int k=0; k<4; k++)
		middle_joint_position(k) = _msg->data[k+7+4+1];
	for(int k=0; k<4; k++)
		pinky_joint_position(k)  = _msg->data[k+7+8+1];
	for(int k=0; k<4; k++)
		thumb_joint_position(k)  = _msg->data[k+7+12+1];
}
void GetJointVelocityState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	joint_velocity_sim_time = _msg->data[0];
	for(int k=0; k<7; k++)
		arm_joint_velocity(k)    = _msg->data[k+1];
	for(int k=0; k<4; k++)
		index_joint_velocity(k)  = _msg->data[k+7+1];
	for(int k=0; k<4; k++)
		middle_joint_velocity(k) = _msg->data[k+7+4+1];
	for(int k=0; k<4; k++)
		pinky_joint_velocity(k)  = _msg->data[k+7+8+1];
	for(int k=0; k<4; k++)
		thumb_joint_velocity(k)  = _msg->data[k+7+12+1];
}

int main(int argc, char **argv){
  string arm_name;
  arm_name = argv[2];
  
  ros::init(argc, argv, "franka_panda_plus_allegro_right_hand_gazebo_joint_controller");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+arm_name+"/joint_command/torque", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+arm_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+arm_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(100);

  while (ros::ok()){
  	
  	std_msgs::Float32MultiArray force_cmd_vector;
  	force_cmd_vector.data.clear();
  	
  	force_cmd_vector.data.push_back(0.0);		// arm joint1
  	force_cmd_vector.data.push_back(0.0);
  	force_cmd_vector.data.push_back(0.0);
  	force_cmd_vector.data.push_back(0.0);
  	force_cmd_vector.data.push_back(0.0);
  	force_cmd_vector.data.push_back(0.0);
  	force_cmd_vector.data.push_back(0.0);
  	force_cmd_vector.data.push_back(5.0);	// gripper finger1
  	force_cmd_vector.data.push_back(5.0);	// gripper finger2
  	
  	JointTorqueCommandPub.publish(force_cmd_vector);
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
	
  return 0;
}

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "../include/Eigen/Dense"
#include "../include/AllegroRightHandModel.h"
#include <iostream>

using namespace std;
using namespace Eigen;

Eigen::Vector4d thumb_joint_position, index_joint_position, middle_joint_position, pinky_joint_position;

void GetJointPositionState(const std_msgs::Float32MultiArray::ConstPtr& _msg){
	for(int k=0; k<4; k++)
		index_joint_position(k,0)  = _msg->data[k];
	for(int k=0; k<4; k++)
		middle_joint_position(k,0) = _msg->data[k+4];
	for(int k=0; k<4; k++)
		pinky_joint_position(k,0)  = _msg->data[k+8];
	for(int k=0; k<4; k++)
		thumb_joint_position(k,0)  = _msg->data[k+12];
	
	//ROS_INFO("ThumbJointPosition: [%f, %f, %f, %f]", thumb_joint_position(0,0), thumb_joint_position(1,0), thumb_joint_position(2,0), thumb_joint_position(3,0));
}

int main(int argc, char **argv){
  ros::init(argc, argv, "allegro_right_hand_controller");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/joint_command/torque", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/allegro_right_hand/joint_state/position", 10, GetJointPositionState);
  ros::Rate loop_rate(100);
	
	Vector4d thumb_joint_position_desired,  thumb_joint_torque_command;
	Vector4d index_joint_position_desired,  index_joint_torque_command;
	Vector4d middle_joint_position_desired, middle_joint_torque_command;
	Vector4d pinky_joint_position_desired,  pinky_joint_torque_command;
	
	thumb_joint_position_desired  << pi/2, 0.0, 0.0, 0.0;
	index_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	middle_joint_position_desired << 0.0, 0.0, 0.0, 0.0;
	pinky_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	
	Matrix4d thumb_DGM, index_DGM, middle_DGM, pinky_DGM;
	
	while (ros::ok()){
  	
  	thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
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
  	
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray joint_torque_command;
  	joint_torque_command.data.clear();
  	//index
  	for(int k=0; k<4; k++)
  		joint_torque_command.data.push_back(index_joint_torque_command(k,0));
  	//middle
  	for(int k=0; k<4; k++)
  		joint_torque_command.data.push_back(middle_joint_torque_command(k,0));
  	//pinky
  	for(int k=0; k<4; k++)
  		joint_torque_command.data.push_back(pinky_joint_torque_command(k,0));
  	//thumb
  	for(int k=0; k<4; k++)
  		joint_torque_command.data.push_back(thumb_joint_torque_command(k,0));
  	
  	JointTorqueCommandPub.publish(joint_torque_command);
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
	
  return 0;
}

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
	
	//thumb_joint_position_desired  << pi, pi/2, pi/2, pi/2;
	//index_joint_position_desired  << pi/3, pi/3, 0.0, 0.0;
	thumb_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	index_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	middle_joint_position_desired << 0.0, 0.0, 0.0, 0.0;
	pinky_joint_position_desired  << 0.0, 0.0, 0.0, 0.0;
	
	int number_of_sample_points = 10;
	
	while (ros::ok()){
  	for(int loop1=0;loop1<=number_of_sample_points;loop1++){
  	  for(int loop2=0;loop2<=number_of_sample_points;loop2++){
  	    for(int loop3=0;loop3<=number_of_sample_points;loop3++){
  	      for(int loop4=0;loop4<=number_of_sample_points;loop4++){
        	  index_joint_position_desired << (index_joint_safe_min(0)+index_joint_safe_range(0)*loop1/number_of_sample_points),
        	                                  (index_joint_safe_min(1)+index_joint_safe_range(1)*loop2/number_of_sample_points),
        	                                  (index_joint_safe_min(2)+index_joint_safe_range(2)*loop3/number_of_sample_points),
        	                                  (index_joint_safe_min(3)+index_joint_safe_range(3)*loop4/number_of_sample_points);
        	  
        	  thumb_joint_position_desired << (thumb_joint_safe_min(0)+thumb_joint_safe_range(0)*loop1/number_of_sample_points),
        	                                  (thumb_joint_safe_min(1)+thumb_joint_safe_range(1)*loop2/number_of_sample_points),
        	                                  (thumb_joint_safe_min(2)+thumb_joint_safe_range(2)*loop3/number_of_sample_points),
        	                                  (thumb_joint_safe_min(3)+thumb_joint_safe_range(3)*loop4/number_of_sample_points);
        	  
        	  //cout << "index_joint_position_desired = " << index_joint_position_desired.transpose() << endl;
        	
        	
          	Kp = 3.7;
          	thumb_DGM  = finger_direct_geometric_model("thumb",thumb_joint_position);
          	index_DGM  = finger_direct_geometric_model("index",index_joint_position);
          	middle_DGM = finger_direct_geometric_model("middle",middle_joint_position);
          	pinky_DGM  = finger_direct_geometric_model("pinky",pinky_joint_position);
          	
          	pose_rpy_Ptt = transformation_matrix_to_pose_rpy(thumb_DGM);
          	pose_rpy_Pit = transformation_matrix_to_pose_rpy(index_DGM);
          	pose_rpy_Pmt = transformation_matrix_to_pose_rpy(middle_DGM);
          	pose_rpy_Ppt = transformation_matrix_to_pose_rpy(pinky_DGM);
          	
          	
          	
          	thumb_joint_velocity_command  = Kp*(thumb_joint_position_desired-thumb_joint_position);
          	index_joint_velocity_command  = Kp*(index_joint_position_desired-index_joint_position);
          	middle_joint_velocity_command = Kp*(middle_joint_position_desired-middle_joint_position);
          	pinky_joint_velocity_command  = Kp*(pinky_joint_position_desired-pinky_joint_position);
          	
          	
          	
          	
          	
          	
          	
          	
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
        }
      }
    }
  }
	
  return 0;
}

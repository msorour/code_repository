#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include "../include/Eigen/Dense"
#include <iostream>
#include "../include/allegro_hand_parameters.h"
#include "../include/useful_implementations.h"
#include "../include/AllegroRightHandModel.h"
#include <time.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

// ROS
std_msgs::Float32MultiArray index_tip_position;
std_msgs::Float32MultiArray middle_tip_position;
std_msgs::Float32MultiArray pinky_tip_position;
std_msgs::Float32MultiArray thumb_tip_position;

void UpdateIndexCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  index_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  index_tip_position.data.push_back(_msg->data[i]);
	index_joint_position << index_tip_position.data[0], index_tip_position.data[1], index_tip_position.data[2];
}

void UpdateMiddleCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  middle_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  middle_tip_position.data.push_back(_msg->data[i]);
	middle_joint_position << middle_tip_position.data[0], middle_tip_position.data[1], middle_tip_position.data[2];
}

void UpdatePinkyCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  pinky_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  pinky_tip_position.data.push_back(_msg->data[i]);
	pinky_joint_position << pinky_tip_position.data[0], pinky_tip_position.data[1], pinky_tip_position.data[2];
}

void UpdateThumbCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  thumb_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  thumb_tip_position.data.push_back(_msg->data[i]);
	thumb_joint_position << thumb_tip_position.data[0], thumb_tip_position.data[1], thumb_tip_position.data[2];
}

/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, char* argv[]){
  // ROS
  ros::init(argc, argv, "publish_finger_tip_position_commands");
  ros::NodeHandle n;
  ros::Publisher  index_tip_position_command_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/index/tip_position" , 10);
  ros::Publisher  middle_tip_position_command_pub = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/middle/tip_position", 10);
  ros::Publisher  pinky_tip_position_command_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/pinky/tip_position" , 10);
  ros::Publisher  thumb_tip_position_command_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/thumb/tip_position" , 10);
  
  ros::Subscriber  index_joint_position_sub  = n.subscribe("/allegro_right_hand/joint_state/index/position" , 10, UpdateIndexCurrentPosition);
  ros::Subscriber  middle_joint_position_sub = n.subscribe("/allegro_right_hand/joint_state/middle/position" , 10, UpdateMiddleCurrentPosition);
  ros::Subscriber  pinky_joint_position_sub  = n.subscribe("/allegro_right_hand/joint_state/pinky/position" , 10, UpdatePinkyCurrentPosition);
  ros::Subscriber  thumb_joint_position_sub  = n.subscribe("/allegro_right_hand/joint_state/thumb/position" , 10, UpdateThumbCurrentPosition);
  
  
  ros::Rate loop_rate(300);
  
  
  Eigen::Vector3d position_Ptt_desired, position_Pit_desired, position_Pmt_desired, position_Ppt_desired;
  
  
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
	
	position_Ptt_init << pose_rpy_Ptt(0), pose_rpy_Ptt(1), pose_rpy_Ptt(2);
	position_Pit_init << pose_rpy_Pit(0), pose_rpy_Pit(1), pose_rpy_Pit(2);
	position_Pmt_init << pose_rpy_Pmt(0), pose_rpy_Pmt(1), pose_rpy_Pmt(2);
	position_Ppt_init << pose_rpy_Ppt(0), pose_rpy_Ppt(1), pose_rpy_Ppt(2);
	
	
	Eigen::Vector3d x_traj, y_traj, z_traj;
  
  clock_t begin, end, now;
	double time_now, start_time, trajectory_duration, end_time;
	double sample_time;
  
  double ros_time_start = ros::Time::now().toNSec();
  double ros_time_now;
  
  trajectory_duration = 5.0;
	begin = clock();
	start_time = (double)(begin)/CLOCKS_PER_SEC;
	end_time = start_time + trajectory_duration;
	std::cout << "start_time : "<<start_time<<std::endl;
	std::cout << "end_time   : "<<end_time<<std::endl;
	
  double ros_time_now;
  while( ros::ok() ){
    index_tip_position_command.data.clear();
    middle_tip_position_command.data.clear();
    pinky_tip_position_command.data.clear();
    thumb_tip_position_command.data.clear();
    
    char execute;
		std::cout << "Press [h] to go to home position or [g] to do simple grasp ... " << std::endl;
		std::cin.get(execute);
		if(execute == 'h'){
			
			// home configuration
			position_Ptt_desired << 0.0455482, 0.156374, 0.0554319;
			position_Pit_desired << 0.01,  0.05, 0.23;
			position_Pmt_desired << 0.01,   0.0, 0.23;
			position_Ppt_desired << 0.01, -0.05, 0.23;
			
			
			
			
			
			
			
			while( ros::ok() and (time_now-start_time) < trajectory_duration ){
				now = clock();
				time_now = (double)(now)/CLOCKS_PER_SEC;
				
				// get current joint value per finger
				for(int i=0; i<4; i++)
					index_joint_position(i) = q[i];
				for(int i=4; i<8; i++)
					middle_joint_position(i-4) = q[i];
				for(int i=8; i<12; i++)
					pinky_joint_position(i-8) = q[i];
				for(int i=12; i<16; i++)
					thumb_joint_position(i-12) = q[i];
				
				Kp_finger = 0.9;
				lambda = -0.5;
				
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
				
				
				x_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ptt_init(0), position_Ptt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ptt_init(1), position_Ptt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ptt_init(2), position_Ptt_desired(2));
				position_Ptt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pit_init(0), position_Pit_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pit_init(1), position_Pit_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pit_init(2), position_Pit_desired(2));
				position_Pit_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pmt_init(0), position_Pmt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pmt_init(1), position_Pmt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Pmt_init(2), position_Pmt_desired(2));
				position_Pmt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ppt_init(0), position_Ppt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ppt_init(1), position_Ppt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, time_now, position_Ppt_init(2), position_Ppt_desired(2));
				position_Ppt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				position_Ptt_error = position_Ptt_traj - position_Ptt;
				position_Pit_error = position_Pit_traj - position_Pit;
				position_Pmt_error = position_Pmt_traj - position_Pmt;
				position_Ppt_error = position_Ppt_traj - position_Ppt;
				
				velocity_Ptt_desired_3d = Kp_finger*position_Ptt_error;
				velocity_Pit_desired_3d = Kp_finger*position_Pit_error;
				velocity_Pmt_desired_3d = Kp_finger*position_Pmt_error;
				velocity_Ppt_desired_3d = Kp_finger*position_Ppt_error;
				
				
				finger_null_space_projector = I4 - Pinv_damped(thumb_position_jacobian, 0.001)*thumb_position_jacobian;
				finger_task_gradient = avoid_joint_limit_task_gradient(thumb_joint_position, thumb_joint_safe_mean, thumb_joint_safe_range);
				thumb_joint_velocity_command  = Pinv_damped(thumb_position_jacobian, 0.001)*velocity_Ptt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
				
				finger_null_space_projector = I4 - Pinv_damped(index_position_jacobian, 0.001)*index_position_jacobian;
				finger_task_gradient = avoid_joint_limit_task_gradient(index_joint_position, index_joint_safe_mean, index_joint_safe_range);
				index_joint_velocity_command  = Pinv_damped(index_position_jacobian, 0.001)*velocity_Pit_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
		
				finger_null_space_projector = I4 - Pinv_damped(middle_position_jacobian, 0.001)*middle_position_jacobian;
				finger_task_gradient = avoid_joint_limit_task_gradient(middle_joint_position, middle_joint_safe_mean, middle_joint_safe_range);
				middle_joint_velocity_command  = Pinv_damped(middle_position_jacobian, 0.001)*velocity_Pmt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
		
				finger_null_space_projector = I4 - Pinv_damped(pinky_position_jacobian, 0.001)*pinky_position_jacobian;
				finger_task_gradient = avoid_joint_limit_task_gradient(pinky_joint_position, pinky_joint_safe_mean, pinky_joint_safe_range);
				pinky_joint_velocity_command  = Pinv_damped(pinky_position_jacobian, 0.001)*velocity_Ppt_desired_3d + lambda*finger_null_space_projector*finger_task_gradient;
				
				
				
				// joint torque values to set after safety check 
				//std::cout<<"torque_desired   = " << torque_desired.transpose() << std::endl;
				torque_desired << index_joint_velocity_command, middle_joint_velocity_command, pinky_joint_velocity_command, thumb_joint_velocity_command;
				//std::cout<<"torque_desired   = " << torque_desired.transpose() << std::endl;
				
				
				
				
				ros::spinOnce();
				loop_rate.sleep();
			}
			
			
			
			
			
			
			
		  
		  ros::Duration(0.5).sleep();
			
			
		  
		}
		else if(execute == 'g'){
			
			// grasp configuration
			//
			position_Ptt_desired << 0.04,  -0.02, 0.05;
			position_Pit_desired << 0.04,  0.04, 0.05;
			position_Pmt_desired << 0.04,   0.0, 0.05;
			position_Ppt_desired << 0.04, -0.04, 0.05;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  
		  
		  ros::Duration(0.5).sleep();
		  
		  
    }
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


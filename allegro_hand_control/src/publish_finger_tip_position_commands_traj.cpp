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

std_msgs::Float32MultiArray index_tip_position_command;
std_msgs::Float32MultiArray middle_tip_position_command;
std_msgs::Float32MultiArray pinky_tip_position_command;
std_msgs::Float32MultiArray thumb_tip_position_command;

Eigen::Vector3d thumb_tip_position_current, index_tip_position_current, middle_tip_position_current, pinky_tip_position_current;


void UpdateIndexCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  index_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  index_tip_position.data.push_back(_msg->data[i]);
	index_tip_position_current << index_tip_position.data[0], index_tip_position.data[1], index_tip_position.data[2];
	//std::cout << "some data came in !" << std::endl;
}

void UpdateMiddleCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  middle_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  middle_tip_position.data.push_back(_msg->data[i]);
	middle_tip_position_current << middle_tip_position.data[0], middle_tip_position.data[1], middle_tip_position.data[2];
}

void UpdatePinkyCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  pinky_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  pinky_tip_position.data.push_back(_msg->data[i]);
	pinky_tip_position_current << pinky_tip_position.data[0], pinky_tip_position.data[1], pinky_tip_position.data[2];
}

void UpdateThumbCurrentPosition(const std_msgs::Float32MultiArray::ConstPtr& _msg){
  thumb_tip_position.data.clear();
  // be careful !!
  // _msg->data[0] is ros time in milliseconds
	for(int i=1; i<4; i++)
	  thumb_tip_position.data.push_back(_msg->data[i]);
	thumb_tip_position_current << thumb_tip_position.data[0], thumb_tip_position.data[1], thumb_tip_position.data[2];
}

/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, char* argv[]){
  // ROS
  ros::init(argc, argv, "publish_finger_tip_position_commands_traj");
  ros::NodeHandle n;
  ros::Publisher  index_tip_position_command_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/index/tip_position" , 10);
  ros::Publisher  middle_tip_position_command_pub = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/middle/tip_position", 10);
  ros::Publisher  pinky_tip_position_command_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/pinky/tip_position" , 10);
  ros::Publisher  thumb_tip_position_command_pub  = n.advertise<std_msgs::Float32MultiArray>("/allegro_right_hand/workspace_command/thumb/tip_position" , 10);
  
  ros::Subscriber  index_joint_position_sub  = n.subscribe("/allegro_right_hand/end_tip_state/index/position" , 10, UpdateIndexCurrentPosition);
  ros::Subscriber  middle_joint_position_sub = n.subscribe("/allegro_right_hand/end_tip_state/middle/position" , 10, UpdateMiddleCurrentPosition);
  ros::Subscriber  pinky_joint_position_sub  = n.subscribe("/allegro_right_hand/end_tip_state/pinky/position" , 10, UpdatePinkyCurrentPosition);
  ros::Subscriber  thumb_joint_position_sub  = n.subscribe("/allegro_right_hand/end_tip_state/thumb/position" , 10, UpdateThumbCurrentPosition);
  
  ros::Rate loop_rate(300);
  
  Eigen::Vector3d x_traj, y_traj, z_traj;
  
	Eigen::Vector3d position_Ptt, position_Ptt_desired, position_Ptt_init, position_Ptt_traj, position_Ptt_error;
	Eigen::Vector3d position_Pit, position_Pit_desired, position_Pit_init, position_Pit_traj, position_Pit_error;
	Eigen::Vector3d position_Pmt, position_Pmt_desired, position_Pmt_init, position_Pmt_traj, position_Pmt_error;
	Eigen::Vector3d position_Ppt, position_Ppt_desired, position_Ppt_init, position_Ppt_traj, position_Ppt_error;
  
  clock_t begin, end, now;
	double time_now, start_time, trajectory_duration, end_time;
	double sample_time;
  
  
  //small delay till communication is okay!
	begin = clock();
	start_time = (double)(begin)/CLOCKS_PER_SEC;
	time_now = start_time;
	end_time = start_time + 1.0;
	while(ros::ok() and time_now < end_time){
		now = clock();
		time_now = (double)(now)/CLOCKS_PER_SEC;
		position_Ptt_init = thumb_tip_position_current;
		position_Pit_init = index_tip_position_current;
		position_Pmt_init = middle_tip_position_current;
		position_Ppt_init = pinky_tip_position_current;
		ros::spinOnce();
	}	
	std::cout << "position_Ptt_init = " << thumb_tip_position_current.transpose() << std::endl;
	
  trajectory_duration = 10.0;
	begin = clock();
	start_time = (double)(ros::Time::now().toNSec())/1000000000;
	end_time = start_time + trajectory_duration;
	std::cout << "start_time : "<<ros::Time::now().toNSec()<<std::endl;
	std::cout << "start_time : "<<start_time<<std::endl;
	std::cout << "end_time   : "<<end_time<<std::endl;
	std::cout << "duration   : "<<end_time-start_time<<std::endl;
	
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
			
			
			while( ros::ok() and ((ros_time_now)/1000000000-start_time) < trajectory_duration ){
				ros_time_now = ros::Time::now().toNSec();
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ptt_init(0), position_Ptt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ptt_init(1), position_Ptt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ptt_init(2), position_Ptt_desired(2));
				position_Ptt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pit_init(0), position_Pit_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pit_init(1), position_Pit_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pit_init(2), position_Pit_desired(2));
				position_Pit_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pmt_init(0), position_Pmt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pmt_init(1), position_Pmt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pmt_init(2), position_Pmt_desired(2));
				position_Pmt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ppt_init(0), position_Ppt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ppt_init(1), position_Ppt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ppt_init(2), position_Ppt_desired(2));
				position_Ppt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				
				
				
				index_tip_position_command.data.clear();
				middle_tip_position_command.data.clear();
				pinky_tip_position_command.data.clear();
				thumb_tip_position_command.data.clear();
				
				index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				
				for(int i=0; i<3; i++){
					index_tip_position_command.data.push_back (position_Pit_traj(i));
					middle_tip_position_command.data.push_back(position_Pmt_traj(i));
					pinky_tip_position_command.data.push_back (position_Ppt_traj(i));
					thumb_tip_position_command.data.push_back (position_Ptt_traj(i));
				}
				
				index_tip_position_command_pub.publish ( index_tip_position_command );
				middle_tip_position_command_pub.publish( middle_tip_position_command );
				pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
				thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
				
				ros::Duration(1.0).sleep();
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		else if(execute == 'g'){
			
			// grasp configuration
			//
			position_Ptt_desired << 0.10,  -0.05, 0.05;
			position_Pit_desired << 0.08,  0.04, 0.05;
			position_Pmt_desired << 0.08,   0.0, 0.05;
			position_Ppt_desired << 0.08, -0.04, 0.05;
			
			while( ros::ok() and ((ros_time_now)/1000000000-start_time) < trajectory_duration ){
				ros_time_now = ros::Time::now().toNSec();
				
				x_traj = OnlineMP_L5B(start_time, end_time-trajectory_duration/2, (ros_time_now)/1000000000, position_Ptt_init(0), position_Ptt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time-trajectory_duration/2, (ros_time_now)/1000000000, position_Ptt_init(1), position_Ptt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time-trajectory_duration/2, (ros_time_now)/1000000000, position_Ptt_init(2), position_Ptt_desired(2));
				position_Ptt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pit_init(0), position_Pit_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pit_init(1), position_Pit_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pit_init(2), position_Pit_desired(2));
				position_Pit_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pmt_init(0), position_Pmt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pmt_init(1), position_Pmt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Pmt_init(2), position_Pmt_desired(2));
				position_Pmt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				x_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ppt_init(0), position_Ppt_desired(0));
				y_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ppt_init(1), position_Ppt_desired(1));
				z_traj = OnlineMP_L5B(start_time, end_time, (ros_time_now)/1000000000, position_Ppt_init(2), position_Ppt_desired(2));
				position_Ppt_traj << x_traj(0), y_traj(0), z_traj(0);
				
				std::cout<< "position_Ptt_init= " << position_Ptt_init.transpose() << ",   position_Ptt_traj= " << position_Ptt_traj.transpose() << ",   position_Ptt_desired= " << position_Ptt_desired.transpose() << std::endl;
				
				
				index_tip_position_command.data.clear();
				middle_tip_position_command.data.clear();
				pinky_tip_position_command.data.clear();
				thumb_tip_position_command.data.clear();
				
				index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
				
				for(int i=0; i<3; i++){
					index_tip_position_command.data.push_back (position_Pit_traj(i));
					middle_tip_position_command.data.push_back(position_Pmt_traj(i));
					pinky_tip_position_command.data.push_back (position_Ppt_traj(i));
					thumb_tip_position_command.data.push_back (position_Ptt_traj(i));
				}
				
				index_tip_position_command_pub.publish ( index_tip_position_command );
				middle_tip_position_command_pub.publish( middle_tip_position_command );
				pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
				thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
				
				//std::cout<< "start_time: " << start_time << "time_now: " << (ros_time_now)/1000000000 << "end_time: " << end_time << std::endl;
				
				ros::spinOnce();
				ros::Duration(0.5).sleep();
				
				//loop_rate.sleep();
			}
    }
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


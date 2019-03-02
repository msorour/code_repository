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
  ros::Rate loop_rate(300);
  
  std_msgs::Float32MultiArray index_tip_position_command;
  std_msgs::Float32MultiArray middle_tip_position_command;
  std_msgs::Float32MultiArray pinky_tip_position_command;
  std_msgs::Float32MultiArray thumb_tip_position_command;
  
  Eigen::Vector3d position_Ptt_desired, position_Pit_desired, position_Pmt_desired, position_Ppt_desired;
  
  Eigen::Vector3d position_Ptt_desired1, position_Pit_desired1, position_Pmt_desired1, position_Ppt_desired1;
  Eigen::Vector3d position_Ptt_desired2, position_Pit_desired2, position_Pmt_desired2, position_Ppt_desired2;
  Eigen::Vector3d position_Ptt_desired3, position_Pit_desired3, position_Pmt_desired3, position_Ppt_desired3;
  Eigen::Vector3d position_Ptt_desired4, position_Pit_desired4, position_Pmt_desired4, position_Ppt_desired4;
  Eigen::Vector3d position_Ptt_desired5, position_Pit_desired5, position_Pmt_desired5, position_Ppt_desired5;
  
  position_Ptt_desired << 0.01,  0.14, 0.07;
  position_Pit_desired << 0.01,  0.05, 0.23;
  position_Pmt_desired << 0.01,   0.0, 0.23;
  position_Ppt_desired << 0.01, -0.05, 0.23;
  
  // hard code grasping steps
  position_Ptt_desired1 << 0.05,  -0.02, 0.05;
	position_Pit_desired1 << 0.05,  0.04, 0.20;
	position_Pmt_desired1 << 0.05,   0.0, 0.20;
	position_Ppt_desired1 << 0.05, -0.04, 0.20;
	
	position_Ptt_desired2 << 0.08,  -0.02, 0.05;
	position_Pit_desired2 << 0.08,  0.04, 0.16;
	position_Pmt_desired2 << 0.08,   0.0, 0.16;
	position_Ppt_desired2 << 0.08, -0.04, 0.16;
	
	position_Ptt_desired3 << 0.07,  -0.02, 0.05;
	position_Pit_desired3 << 0.07,  0.04, 0.13;
	position_Pmt_desired3 << 0.07,   0.0, 0.13;
	position_Ppt_desired3 << 0.07, -0.04, 0.13;
	
	position_Ptt_desired4 << 0.06,  -0.02, 0.05;
	position_Pit_desired4 << 0.06,  0.04, 0.10;
	position_Pmt_desired4 << 0.06,   0.0, 0.10;
	position_Ppt_desired4 << 0.06, -0.04, 0.10;
  
  position_Ptt_desired5 << 0.04,  -0.02, 0.05;
	position_Pit_desired5 << 0.04,  0.04, 0.05;
	position_Pmt_desired5 << 0.04,   0.0, 0.05;
	position_Ppt_desired5 << 0.04, -0.04, 0.05;
	
	
	
	
  
  double ros_time_now;
  while( ros::ok() ){
    index_tip_position_command.data.clear();
    middle_tip_position_command.data.clear();
    pinky_tip_position_command.data.clear();
    thumb_tip_position_command.data.clear();
    
    char execute;
		std::cout << "Press [h] to go to home position or [g] to do simple grasp or [t] to move thumb ... " << std::endl;
		std::cin.get(execute);
		if(execute == 'h'){
			
			//
			// send step#4 grasp
			//
			position_Ptt_desired = position_Ptt_desired4;
			position_Pit_desired = position_Pit_desired4;
			position_Pmt_desired = position_Pmt_desired4;
			position_Ppt_desired = position_Ppt_desired4;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
			
			
			
			
			//
			// send step#3 grasp
			//
			position_Ptt_desired = position_Ptt_desired3;
			position_Pit_desired = position_Pit_desired3;
			position_Pmt_desired = position_Pmt_desired3;
			position_Ppt_desired = position_Ppt_desired3;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
			
			
			
			
			//
			// send step#2 grasp
			//
			position_Ptt_desired = position_Ptt_desired2;
			position_Pit_desired = position_Pit_desired2;
			position_Pmt_desired = position_Pmt_desired2;
			position_Ppt_desired = position_Ppt_desired2;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
			
			
			
			
			//
			// send step#1 grasp
			//
			position_Ptt_desired = position_Ptt_desired1;
			position_Pit_desired = position_Pit_desired1;
			position_Pmt_desired = position_Pmt_desired1;
			position_Ppt_desired = position_Ppt_desired1;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
			
			
			
			position_Ptt_desired << 0.045,  0.065, 0.085;
		  position_Pit_desired << 0.00,  0.05, 0.23;
		  position_Pmt_desired << 0.00,   0.0, 0.23;
		  position_Ppt_desired << 0.00, -0.05, 0.23;
		  
		  
		  
		  
		  
		  
		}
		else if(execute == 'g'){
			/*
  		position_Ptt_desired << 0.02,  0.00, 0.09;
			position_Pit_desired << 0.03,  0.04, 0.08;
			position_Pmt_desired << 0.03,   0.0, 0.08;
			position_Ppt_desired << 0.03, -0.04, 0.08;
			*/
			
			//
			// send step#1 grasp
			//
			position_Ptt_desired = position_Ptt_desired1;
			position_Pit_desired = position_Pit_desired1;
			position_Pmt_desired = position_Pmt_desired1;
			position_Ppt_desired = position_Ppt_desired1;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
		  
		  
		  
		  //
			// send step#2 grasp
			//
			position_Ptt_desired = position_Ptt_desired2;
			position_Pit_desired = position_Pit_desired2;
			position_Pmt_desired = position_Pmt_desired2;
			position_Ppt_desired = position_Ppt_desired2;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
		  
		  
		  //
			// send step#3 grasp
			//
			position_Ptt_desired = position_Ptt_desired3;
			position_Pit_desired = position_Pit_desired3;
			position_Pmt_desired = position_Pmt_desired3;
			position_Ppt_desired = position_Ppt_desired3;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
		  
		  
		  //
			// send step#4 grasp
			//
			position_Ptt_desired = position_Ptt_desired4;
			position_Pit_desired = position_Pit_desired4;
			position_Pmt_desired = position_Pmt_desired4;
			position_Ppt_desired = position_Ppt_desired4;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
		  
		  
		  //
			// send step#5 grasp
			//
			position_Ptt_desired = position_Ptt_desired5;
			position_Pit_desired = position_Pit_desired5;
			position_Pmt_desired = position_Pmt_desired5;
			position_Ppt_desired = position_Ppt_desired5;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
			
    }
    
    
    else if(execute == 't'){
		  position_Ptt_desired = position_Ptt_desired1;
			position_Pit_desired = position_Pit_desired1;
			position_Pmt_desired = position_Pmt_desired1;
			position_Ppt_desired = position_Ppt_desired1;
			
			ros_time_now = ros::Time::now().toNSec();
			
		  index_tip_position_command.data.clear();
		  middle_tip_position_command.data.clear();
		  pinky_tip_position_command.data.clear();
		  thumb_tip_position_command.data.clear();
		  
		  index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
		  
		  for(int i=0; i<3; i++){
		    index_tip_position_command.data.push_back (position_Pit_desired(i));
		    middle_tip_position_command.data.push_back(position_Pmt_desired(i));
		    pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
		    thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
		  }
		  
		  index_tip_position_command_pub.publish ( index_tip_position_command );
		  middle_tip_position_command_pub.publish( middle_tip_position_command );
		  pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
		  thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
		  
		  ros::Duration(0.5).sleep();
    }
    
    
    
    ros_time_now = ros::Time::now().toNSec();
    index_tip_position_command.data.push_back( (ros_time_now)/1000000 );
    middle_tip_position_command.data.push_back( (ros_time_now)/1000000 );
    pinky_tip_position_command.data.push_back( (ros_time_now)/1000000 );
    thumb_tip_position_command.data.push_back( (ros_time_now)/1000000 );
    for(int i=0; i<3; i++){
      index_tip_position_command.data.push_back (position_Pit_desired(i));
      middle_tip_position_command.data.push_back(position_Pmt_desired(i));
      pinky_tip_position_command.data.push_back (position_Ppt_desired(i));
      thumb_tip_position_command.data.push_back (position_Ptt_desired(i));
    }
    index_tip_position_command_pub.publish ( index_tip_position_command );
    middle_tip_position_command_pub.publish( middle_tip_position_command );
    pinky_tip_position_command_pub.publish ( pinky_tip_position_command );
    thumb_tip_position_command_pub.publish ( thumb_tip_position_command );
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


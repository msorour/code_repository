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
  
  position_Ptt_desired << 0.02,  0.00, 0.08;
  
  position_Pit_desired << 0.05,  0.05, 0.19;
  position_Pmt_desired << 0.05,   0.0, 0.19;
  position_Ppt_desired << 0.05, -0.05, 0.19;
  
  double ros_time_now;
  while( ros::ok() ){
    index_tip_position_command.data.clear();
    middle_tip_position_command.data.clear();
    pinky_tip_position_command.data.clear();
    thumb_tip_position_command.data.clear();
    
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


#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "franka_panda_plus_own_gripper_gazebo_joint_controller");
	ros::NodeHandle n;
	ros::Publisher ForceCmdVectorPub = n.advertise<std_msgs::Float32MultiArray>("/force_cmd_vector", 10);
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
  	
  	ForceCmdVectorPub.publish(force_cmd_vector);
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
	
  return 0;
}

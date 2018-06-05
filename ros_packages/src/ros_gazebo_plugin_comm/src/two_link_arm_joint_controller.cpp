#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ros_gazebo_plugin_comm/Vector7d.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv){
	
  ros::init(argc, argv, "two_link_arm_joint_controller");
	ros::NodeHandle n;
	ros::Publisher force_cmd = n.advertise<std_msgs::Float32>("/force_cmd/th1", 10);
	//ros::Publisher force_cmd_vector = n.advertise<ros_gazebo_plugin_comm::Vector7d>("/force_cmd_vector", 10);
  ros::Publisher force_cmd_vector = n.advertise<std_msgs::Float32MultiArray>("/force_cmd_vector", 10);
  ros::Rate loop_rate(100);

  while (ros::ok()){
  	float force_vector_2[7] = {1,2,3,4,5,6,7};
  	
  	std_msgs::Float32MultiArray force_vector;
  	force_vector.data.clear();
  	
  	//ros_gazebo_plugin_comm::Vector7d force_vector;
  	force_vector.data.push_back(40.1);
  	force_vector.data.push_back(7.1);
  	force_cmd_vector.publish(force_vector);
  	
    std_msgs::Float32 force;
		force.data = 7.0;
		force_cmd.publish(force);
		
    ros::spinOnce();
    loop_rate.sleep();
  }
	
  return 0;
}

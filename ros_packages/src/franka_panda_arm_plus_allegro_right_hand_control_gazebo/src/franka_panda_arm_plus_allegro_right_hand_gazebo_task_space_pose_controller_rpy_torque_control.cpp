#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "std_msgs/Float32MultiArray.h"
#include "../../../include/Eigen/Dense"
#include "../../../include/useful_implementations.h"
#include "../../../include/AllegroRightHandModel.h"
#include "../../../include/FrankaPandaArmModel.h"
#include "../include/franka_panda_arm_plus_allegro_right_hand.h"

#include <iostream>
#include <fstream>
#include <time.h>

using namespace std;
using namespace Eigen;
using namespace franka_panda_gazebo_controller;


void log_data(void);
void open_logs(void);
void close_logs(void);

// /master_cell/kinect2/sd/points

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  public:
    std::string window_name="";
    ImageConverter(std::string image_topic_name) : it_(nh_){
      // Subscrive to input video feed
      image_sub_ = it_.subscribe(image_topic_name, 1, &ImageConverter::imageCb, this);
      cv::namedWindow(window_name);
    }
    
    ~ImageConverter(){cv::destroyWindow(window_name);}

    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      try{cv_ptr = cv_bridge::toCvCopy(msg);}
      catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what()); return;}
      
      // Update GUI Window
      cv::imshow(window_name, cv_ptr->image);
      cv::waitKey(3);
    }
};

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
  string model_name;
  model_name = argv[2];
  
  ros::init(argc, argv, "franka_panda_arm_plus_allegro_right_hand_gazebo_task_space_pose_controller_rpy");
	ros::NodeHandle n;
	ros::Publisher  JointTorqueCommandPub   = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/torque", 10);
	ros::Publisher  JointVelocityCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/velocity", 10);
	ros::Publisher  JointPositionCommandPub = n.advertise<std_msgs::Float32MultiArray>("/"+model_name+"/joint_command/position", 10);
  ros::Subscriber JointPositionStateSub = n.subscribe("/"+model_name+"/joint_state/position", 10, GetJointPositionState);
  ros::Subscriber JointVelocityStateSub = n.subscribe("/"+model_name+"/joint_state/velocity", 10, GetJointVelocityState);
  ros::Rate loop_rate(100);
  
	
	// Wait for a few moments till correct joint values are loaded
	while (ros::ok() and ros::Time::now().toSec() < start_program_delay*10 ){
  	ros::spinOnce();
    loop_rate.sleep();
  }
  
  // safe maximum and minimum joint values
	arm_joint_safe_max << arm_joint_0_max-joint_safety_margin, arm_joint_1_max-joint_safety_margin, arm_joint_2_max-joint_safety_margin, arm_joint_3_max-joint_safety_margin, 
                        arm_joint_4_max-joint_safety_margin, arm_joint_5_max-joint_safety_margin, arm_joint_6_max-joint_safety_margin;
	arm_joint_safe_min << arm_joint_0_min+joint_safety_margin, arm_joint_1_min+joint_safety_margin, arm_joint_2_min+joint_safety_margin, arm_joint_3_min+joint_safety_margin, 
	                      arm_joint_4_min+joint_safety_margin, arm_joint_5_min+joint_safety_margin, arm_joint_6_min+joint_safety_margin;
	// safe joint range (stroke)
	arm_joint_safe_range = arm_joint_safe_max-arm_joint_safe_min;
	// safe mean value per joint
	arm_joint_safe_mean = (arm_joint_safe_max+arm_joint_safe_min)/2;
	
	pose_rpy_bE_desired << 0.2, -0.2, 0.3, 0, 0, 0;
	
  position_Ptt_desired << 0.08, 0.07, 0.04;
	position_Pit_desired << 0.07, 0.05, 0.17;
	position_Pmt_desired << 0.07, 0.0, 0.17;
	position_Ppt_desired << 0.07, -0.05, 0.17;
	
	start_time = ros::Time::now().toSec();
	trajectory_duration = 5.0;
	end_time = start_time + trajectory_duration;
	Kp_arm = 1.7;
	Kp_finger = 5.7;
	lambda = -3;
	
	arm_DGM  = arm_direct_geometric_model(arm_joint_position, "end_tip");
	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
 	cout << "initial pose_rpy_bE = " << pose_rpy_bE.transpose()  << endl;
 	pose_rpy_bE_init = pose_rpy_bE;
 	
 	
 	
 	
 	
 	
  
  
  
  
  
  
  
 	
 	
 	time_now = ros::Time::now().toSec();
 	time_past = time_now;
	
	geometric_jacobian = arm_geometric_jacobian_matrix(arm_joint_position,"base");
	geometric_jacobian_past = geometric_jacobian;
	open_logs();
	//while (ros::ok() and ros::Time::now().toSec()<20 ){
  while (ros::ok()){
  	time_now = ros::Time::now().toSec();
  	//cout << "time_now = " << time_now  << endl;
  	
  	if(time_now>=5.0 and time_now<10.0){
  	  pose_rpy_bE_desired << 0.45, -0.3, 0.3, -pi/2, 0, 0;
  	  // Allegro Hand
    	position_Ptt_desired << 0.08, 0.07, 0.04;
    	position_Pit_desired << 0.07, 0.05, 0.19;
    	position_Pmt_desired << 0.07, 0.0, 0.19;
    	position_Ppt_desired << 0.07, -0.05, 0.19;
  	}
  	else if(time_now>=10.0 and time_now<13.0)
  	  pose_rpy_bE_desired << 0.45, 0.0, 0.12, -pi/2, 0, 0;
  	else if(time_now>=13.0 and time_now<16.0){
  	  pose_rpy_bE_desired << 0.4, 0.0, 0.07, -pi/2, 0, 0;
  	  // Allegro Hand
    	position_Ptt_desired << 0.08, 0.07, 0.04;
    	position_Pit_desired << 0.07, 0.05, 0.19;
    	position_Pmt_desired << 0.07, 0.0, 0.19;
    	position_Ppt_desired << 0.07, -0.05, 0.19;
  	}
  	else if(time_now>=16.0 and time_now<19.0){
  	  pose_rpy_bE_desired << 0.4, 0.0, 0.07, -pi/2, 0, 0;
  	  // Allegro Hand
    	position_Ptt_desired << 0.07, 0.03, 0.07;
    	position_Pit_desired << 0.08, 0.05, 0.09;
    	position_Pmt_desired << 0.08, 0.0, 0.09;
    	position_Ppt_desired << 0.08, -0.05, 0.09;
  	}
  	else if(time_now>=19.0){
  	  pose_rpy_bE_desired << 0.4, 0.0, 0.22, -pi/2, 0, 0;
  	  // Allegro Hand
    	position_Ptt_desired << 0.07, 0.04, 0.04;
    	position_Pit_desired << 0.09, 0.05, 0.08;
    	position_Pmt_desired << 0.09, 0.0, 0.07;
    	position_Ppt_desired << 0.09, -0.05, 0.06;
  	}
  	
  	arm_DGM  = arm_direct_geometric_model(arm_joint_position, "allegro_hand");
  	pose_rpy_bE = transformation_matrix_to_pose_rpy(arm_DGM);
  	pose_error = pose_rpy_bE_desired-pose_rpy_bE;
  	//cout << "pose_error = " << pose_error.transpose() << endl;
  	//cout << "joint_velocity_error = " << (arm_joint_velocity_desired-arm_joint_velocity).transpose() << endl;
  	geometric_jacobian = arm_geometric_jacobian_matrix(arm_joint_position,"base");
  	analytic_jacobian = geometric_to_analytic_jacobian_rpy(pose_rpy_bE)*geometric_jacobian;
  	
  	/*Vector3d V, W;
  	cout << "velocity_bE_desired Before = " << velocity_bE_desired.transpose() << endl;
  	V << velocity_bE_desired(0), velocity_bE_desired(1), velocity_bE_desired(2);
  	W << velocity_bE_desired(3), velocity_bE_desired(4), velocity_bE_desired(5);
  	V = linear_thresholding(V, 0.5);
  	W = linear_thresholding(W, 1.5);
  	velocity_bE_desired << V, W;
  	cout << "velocity_bE_desired After = " << velocity_bE_desired.transpose() << endl;
  	*/
  	
  	//Vector3d position_error, orientation_error;
  	//position_error << pose_error(0), pose_error(1), pose_error(2);
  	//orientation_error << pose_error(3), pose_error(4), pose_error(5);
  	velocity_bE_desired = Kp_arm*pose_error;
  	//velocity_bE_desired << 1.7*position_error, 7*orientation_error;
  	// avoid joint limits task
  	arm_null_space_projector = I7 - Pinv_damped(analytic_jacobian, 0.001)*analytic_jacobian;
  	arm_task_gradient = avoid_joint_limit_task_gradient(arm_joint_position, arm_joint_safe_mean, arm_joint_safe_range);
  	arm_joint_velocity_desired = Pinv_damped(analytic_jacobian, 0.001)*velocity_bE_desired + lambda*arm_null_space_projector*arm_task_gradient;
  	//arm_joint_velocity_desired = Pinv_damped(analytic_jacobian, 0.0001)*velocity_bE_desired;
  	
  	
  	
  	
  	
  	///
  	///
  	///
  	// Computing Joint Torque Command
  	velocity_bE_desired = Kp*pose_error;
  	arm_joint_velocity_desired = Pinv_damped(analytic_jacobian, 0.001)*velocity_bE_desired;
  	//arm_joint_velocity_desired = Pinv_damped(geometric_jacobian, 0.001)*velocity_bE_desired;
  	arm_joint_acceleration_desired = Kv*(arm_joint_velocity_desired-arm_joint_velocity);
  	
  	inertia_matrix = arm_inertia_matrix(arm_joint_position);
  	viscous_friction_torque = arm_viscous_friction_torque(arm_joint_velocity);
  	static_friction_torque = arm_static_friction_torque(arm_joint_velocity);
  	coriolis_centrifugal_torque = arm_coriolis_centrifugal_torque(arm_joint_position, arm_joint_velocity);
  	gravity_compensation_torque = arm_gravity_compensation_torque(arm_joint_position);
  	arm_joint_torque_command  = inertia_matrix*arm_joint_acceleration_desired + viscous_friction_torque + static_friction_torque + coriolis_centrifugal_torque + gravity_compensation_torque;
  	//arm_joint_torque_command  = arm_gravity_compensation_torque(arm_joint_position);
  	//cout << "arm_gravity_compensation_torque = " << arm_joint_torque_command.transpose()  << endl;
  	
  	
  	// Sending Joint Torque Command
  	std_msgs::Float32MultiArray torque_command;
  	torque_command.data.clear();
  	for(int k=0; k<7; k++)
  		torque_command.data.push_back(arm_joint_torque_command(k));
  	//JointTorqueCommandPub.publish(torque_command);
  	
  	
  	
  	
  	// Allegro Hand
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
  	
  	//index_joint_velocity_command << 0,0,0,0;
  	//middle_joint_velocity_command << 0,0,0,0;
  	//pinky_joint_velocity_command << 0,0,0,0;
  	//thumb_joint_velocity_command << 0,0,0,0;
  	
  	
  	
  	
  	
  	
  	
  	
  	
  	// Sending Joint VElocity Command
  	std_msgs::Float32MultiArray velocity_command;
  	velocity_command.data.clear();
  	for(int k=0; k<7; k++)
  		velocity_command.data.push_back(arm_joint_velocity_desired(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(index_joint_velocity_command(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(middle_joint_velocity_command(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(pinky_joint_velocity_command(k));
  	for(int k=0; k<4; k++)
  		velocity_command.data.push_back(thumb_joint_velocity_command(k));
  	JointVelocityCommandPub.publish(velocity_command);
  	
  	
  	// Data logging
  	log_data();
  	
  	// For next iteration
  	time_past = time_now;
  	geometric_jacobian_past = geometric_jacobian;
  	
    ros::spinOnce();
    loop_rate.sleep();
  }
  close_logs();
  return 0;
}









void log_data(void){
	joint_velocity_response_log << time_now << " " << arm_joint_velocity.transpose() <<  endl;
	joint_velocity_error_log << time_now << " " << arm_joint_velocity_error.transpose() <<  endl;
	
	joint_torque_command_log << time_now << " " << arm_joint_torque_command.transpose() <<  endl;
	
	task_space_pose_command_log << time_now << " " << pose_rpy_bE_traj.transpose() <<  endl;
	task_space_pose_response_log << time_now << " " << pose_rpy_bE.transpose() <<  endl;
	task_space_pose_error_log << time_now << " " << pose_error.transpose() <<  endl;
	
	task_space_velocity_command_log << time_now << " " << velocity_bE_traj.transpose() <<  endl;
	task_space_velocity_response_log << time_now << " " << velocity_bE.transpose() <<  endl;
	task_space_velocity_error_log << time_now << " " << velocity_error.transpose() <<  endl;
	
	task_space_acceleration_command_log << time_now << " " << acceleration_bE_traj.transpose() <<  endl;
}

void open_logs(void){
	joint_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/joint_velocity_response_log.txt");
	joint_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/joint_velocity_error_log.txt");
	
	joint_torque_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/joint_torque_command_log.txt");
	
	task_space_pose_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_command_log.txt");
	task_space_pose_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_response_log.txt");
	task_space_pose_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_error_log.txt");
	
	task_space_velocity_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_velocity_command_log.txt");
	task_space_velocity_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_velocity_response_log.txt");
	task_space_velocity_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_velocity_error_log.txt");
	
	task_space_acceleration_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_acceleration_command_log.txt");
	task_space_acceleration_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_acceleration_response_log.txt");
	task_space_acceleration_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_arm_plus_own_gripper_control_gazebo/logs/task_space_pose_acceleration_log.txt");
}
void close_logs(void){
	joint_velocity_response_log.close();
	joint_velocity_error_log.close();
	
	joint_torque_command_log.close();
	
	task_space_pose_command_log.close();
	task_space_pose_response_log.close();
	task_space_pose_error_log.close();
	
	task_space_velocity_command_log.close();
	task_space_velocity_response_log.close();
	task_space_velocity_error_log.close();
	
	task_space_acceleration_command_log.close();
	task_space_acceleration_response_log.close();
	task_space_acceleration_error_log.close();
}

#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace franka_panda_gazebo_controller
{
	VectorXd arm_joint_position(7), arm_joint_velocity(7);
	VectorXd arm_joint_velocity_desired(7), arm_joint_torque_command(7);
	VectorXd gripper_joint_position(2), gripper_joint_velocity(2);
	VectorXd gripper_joint_velocity_desired(2), gripper_joint_torque_command(2);
	
	VectorXd arm_joint_velocity_error(7);
	Vector3d arm_trajectory;
	
	Matrix4d arm_DGM;
	MatrixXd inertia_matrix(7,7);
	VectorXd viscous_friction_torque(7);
	VectorXd static_friction_torque(7);
	VectorXd coriolis_centrifugal_torque(7);
	
	double Kv, time_now, start_program_delay=0.02;
	//double time_init, traj_duration;
	
	ofstream joint_velocity_command_log;
	ofstream joint_velocity_response_log;
	ofstream joint_velocity_error_log;
	
	ofstream joint_torque_command_log;
	
	ofstream task_space_velocity_command_log;
	ofstream task_space_velocity_response_log;
	ofstream task_space_velocity_error_log;
	
	VectorXd arm_velocity_bE(6), arm_velocity_bE_desired(6), arm_velocity_error(6);
	MatrixXd geometric_jacobian(6,7);
	
}

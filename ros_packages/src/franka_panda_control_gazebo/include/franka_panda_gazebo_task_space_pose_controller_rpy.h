#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace franka_panda_gazebo_controller
{
	VectorXd joint_position(7), joint_velocity(7);
	VectorXd joint_velocity_desired(7), joint_torque_command(7);
	
	VectorXd joint_velocity_error(7);
	Vector3d trajectory;
	
	Matrix4d arm_DGM;
	MatrixXd inertia_matrix(7,7);
	VectorXd viscous_friction_torque(7);
	VectorXd static_friction_torque(7);
	VectorXd coriolis_centrifugal_torque(7);
	
	double time_now, start_time, trajectory_duration, end_time;
	double time_past;
	double Kp, Kv;
	
	double start_program_delay=0.02;
	
	ofstream joint_position_command_log;
	ofstream joint_position_response_log;
	ofstream joint_position_error_log;
	
	ofstream joint_velocity_command_log;
	ofstream joint_velocity_response_log;
	ofstream joint_velocity_error_log;
	
	ofstream joint_torque_command_log;
	
	ofstream task_space_velocity_command_log;
	ofstream task_space_velocity_response_log;
	ofstream task_space_velocity_error_log;
	
	
	VectorXd pose_rpy_bE(6), pose_rpy_bE_desired(6), pose_rpy_bE_traj(6), pose_error(6);
	VectorXd velocity_bE(6), velocity_bE_desired(6), velocity_bE_traj(6), velocity_error(6);
	MatrixXd geometric_jacobian(6,7), geometric_jacobian_past(6,7), geometric_jacobian_derivative(6,7);
	MatrixXd analytic_jacobian(6,7);
	
}

#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace franka_panda_gazebo_controller
{
	VectorXd joint_position(7), joint_velocity(7);
	
	VectorXd joint_position_desired(7), joint_torque_command(7);
	
	VectorXd joint_position_init(7), joint_position_traj(7), joint_velocity_traj(7), joint_acceleration_traj(7);
	VectorXd joint_position_error(7), joint_velocity_error(7);
	Vector3d trajectory;
	
	Matrix4d arm_DGM;
	double time_now, start_time, trajectory_duration, end_time;
	double Kp, Kv;
	
	double start_program_delay=0.02;
	
	ofstream joint_position_command_log;
	ofstream joint_position_response_log;
	ofstream joint_position_error_log;
}

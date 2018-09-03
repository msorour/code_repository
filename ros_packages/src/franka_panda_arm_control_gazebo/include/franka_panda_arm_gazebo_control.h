namespace franka_panda_gazebo_controller
{
	Eigen::VectorXd joint_position(7), joint_velocity(7);
	Eigen::VectorXd joint_position_desired(7), joint_velocity_desired(7), joint_acceleration_desired(7), joint_torque_command(7);
	
	Eigen::VectorXd joint_position_init(7), joint_position_traj(7), joint_velocity_traj(7), joint_acceleration_traj(7);
	Eigen::VectorXd joint_position_error(7), joint_velocity_error(7);
	Eigen::Vector3d trajectory;
	
	Eigen::Matrix4d arm_DGM;
	Eigen::MatrixXd inertia_matrix(7,7);
	Eigen::VectorXd viscous_friction_torque(7), static_friction_torque(7), coriolis_centrifugal_torque(7), gravity_compensation_torque(7);
	
	double joint_position_sim_time, joint_velocity_sim_time;
	double time_now, start_time, trajectory_duration, end_time, time_past;
	double Kp, Kv;
	double start_program_delay=0.02;
	
	Eigen::VectorXd pose_rpy_bE(6), pose_rpy_bE_desired(6), pose_rpy_bE_traj(6), pose_error(6);
	Eigen::VectorXd velocity_bE(6), velocity_bE_desired(6), velocity_bE_traj(6), velocity_error(6);
	Eigen::MatrixXd geometric_jacobian(6,7), geometric_jacobian_past(6,7), geometric_jacobian_derivative(6,7);
	Eigen::MatrixXd analytic_jacobian(6,7);
	
	std::ofstream joint_position_command_log;
	std::ofstream joint_position_response_log;
	std::ofstream joint_position_error_log;
	
	std::ofstream joint_velocity_command_log;
	std::ofstream joint_velocity_response_log;
	std::ofstream joint_velocity_error_log;
	
	std::ofstream joint_torque_command_log;
	
	std::ofstream task_space_pose_command_log;
	std::ofstream task_space_pose_response_log;
	std::ofstream task_space_pose_error_log;
	
	std::ofstream task_space_velocity_command_log;
	std::ofstream task_space_velocity_response_log;
	std::ofstream task_space_velocity_error_log;
}

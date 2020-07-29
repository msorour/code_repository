namespace franka_panda_gazebo_controller
{
	Eigen::VectorXd joint_position(7), joint_velocity(7);
	Eigen::VectorXd joint_velocity_desired(7), joint_acceleration_desired(7), joint_torque_command(7);
	
	Eigen::VectorXd gripper_joint_position(2), gripper_joint_velocity(2);
	Eigen::VectorXd gripper_full_open_position(2), gripper_closed_position(2);
	Eigen::VectorXd gripper_joint_position_desired(2), gripper_joint_velocity_desired(2), gripper_joint_acceleration_desired(2), gripper_joint_torque_command(2);
	Eigen::VectorXd gripper_joint_position_error(2);
	
	Eigen::VectorXd joint_velocity_error(7);
	Eigen::Vector3d trajectory;
	
	Eigen::VectorXd arm_joint_position(7), arm_joint_velocity(7);
	Eigen::VectorXd arm_joint_velocity_desired(7), arm_joint_torque_command(7);
	
	Eigen::VectorXd arm_joint_velocity_error(7);
	Eigen::Vector3d arm_trajectory;
	
	Eigen::Matrix4d arm_DGM;
	Eigen::MatrixXd inertia_matrix(7,7);
	Eigen::VectorXd viscous_friction_torque(7);
	Eigen::VectorXd static_friction_torque(7);
	Eigen::VectorXd coriolis_centrifugal_torque(7);
	
	Eigen::VectorXd pose_rpy_bE(6), pose_rpy_bE_desired(6), pose_rpy_bE_init(6), pose_rpy_bE_traj(6), pose_error(6);
	Eigen::VectorXd velocity_bE(6), velocity_bE_desired(6), velocity_bE_traj(6), velocity_error(6);
	Eigen::VectorXd acceleration_bE_desired(6), acceleration_bE_traj(6);
	Eigen::MatrixXd geometric_jacobian(6,7), geometric_jacobian_past(6,7), geometric_jacobian_derivative(6,7);
	Eigen::MatrixXd analytic_jacobian(6,7);
	Eigen::VectorXd arm_velocity_bE(6), arm_velocity_bE_desired(6), arm_velocity_error(6);
	
	double joint_position_sim_time, joint_velocity_sim_time;
	double time_now, start_time, trajectory_duration, end_time;
	double time_past;
	double Kp, Kv, Kp_gripper;
	
	double start_program_delay=0.02;
	
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
	
	std::ofstream task_space_acceleration_command_log;
	std::ofstream task_space_acceleration_response_log;
	std::ofstream task_space_acceleration_error_log;
	
}

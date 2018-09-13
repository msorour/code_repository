namespace allegro_right_hand{
  Eigen::Vector4d r_index, d_index, alpha_index, theta_index;
  Eigen::Vector4d r_thumb, d_thumb, alpha_thumb, theta_thumb;
  
  Eigen::Matrix4d thumb_DGM, index_DGM, middle_DGM, pinky_DGM;
  Eigen::MatrixXd thumb_position_jacobian(3,4), index_position_jacobian(3,4), middle_position_jacobian(3,4), pinky_position_jacobian(3,4);
  
  Eigen::Vector4d thumb_joint_position_desired,  thumb_joint_velocity_desired,  thumb_joint_torque_command,  thumb_joint_velocity_command,  thumb_joint_position,  thumb_joint_velocity;
	Eigen::Vector4d index_joint_position_desired,  index_joint_velocity_desired,  index_joint_torque_command,  index_joint_velocity_command,  index_joint_position,  index_joint_velocity;
	Eigen::Vector4d middle_joint_position_desired, middle_joint_velocity_desired, middle_joint_torque_command, middle_joint_velocity_command, middle_joint_position, middle_joint_velocity;
	Eigen::Vector4d pinky_joint_position_desired,  pinky_joint_velocity_desired,  pinky_joint_torque_command,  pinky_joint_velocity_command,  pinky_joint_position,  pinky_joint_velocity;
	
	Eigen::VectorXd pose_rpy_Ptt(6), pose_rpy_Ptt_desired(6), pose_rpy_Ptt_init(6), pose_rpy_Ptt_traj(6), pose_rpy_Ptt_error(6);
	Eigen::VectorXd pose_rpy_Pit(6), pose_rpy_Pit_desired(6), pose_rpy_Pit_init(6), pose_rpy_Pit_traj(6), pose_rpy_Pit_error(6);
	Eigen::VectorXd pose_rpy_Pmt(6), pose_rpy_Pmt_desired(6), pose_rpy_Pmt_init(6), pose_rpy_Pmt_traj(6), pose_rpy_Pmt_error(6);
	Eigen::VectorXd pose_rpy_Ppt(6), pose_rpy_Ppt_desired(6), pose_rpy_Ppt_init(6), pose_rpy_Ppt_traj(6), pose_rpy_Ppt_error(6);
	
  Eigen::VectorXd velocity_Ptt(6), velocity_Ptt_desired(6), velocity_Ptt_init(6), velocity_Ptt_traj(6), velocity_Ptt_error(6);
	Eigen::VectorXd velocity_Pit(6), velocity_Pit_desired(6), velocity_Pit_init(6), velocity_Pit_traj(6), velocity_Pit_error(6);
	Eigen::VectorXd velocity_Pmt(6), velocity_Pmt_desired(6), velocity_Pmt_init(6), velocity_Pmt_traj(6), velocity_Pmt_error(6);
	Eigen::VectorXd velocity_Ppt(6), velocity_Ppt_desired(6), velocity_Ppt_init(6), velocity_Ppt_traj(6), velocity_Ppt_error(6);
	
	double index_joint_0_safe_max = index_joint_0_max-joint_safety_margin;  double index_joint_0_safe_min = index_joint_0_min+joint_safety_margin;
	double index_joint_1_safe_max = index_joint_1_max-joint_safety_margin;  double index_joint_1_safe_min = index_joint_1_min+joint_safety_margin;
	double index_joint_2_safe_max = index_joint_2_max-joint_safety_margin;  double index_joint_2_safe_min = index_joint_2_min+joint_safety_margin;
	double index_joint_3_safe_max = index_joint_3_max-joint_safety_margin;  double index_joint_3_safe_min = index_joint_3_min+joint_safety_margin;
	
	double thumb_joint_0_safe_max = thumb_joint_0_max-joint_safety_margin;  double thumb_joint_0_safe_min = thumb_joint_0_min+joint_safety_margin;
	double thumb_joint_1_safe_max = thumb_joint_1_max-joint_safety_margin;  double thumb_joint_1_safe_min = thumb_joint_1_min+joint_safety_margin;
	double thumb_joint_2_safe_max = thumb_joint_2_max-joint_safety_margin;  double thumb_joint_2_safe_min = thumb_joint_2_min+joint_safety_margin;
	double thumb_joint_3_safe_max = thumb_joint_3_max-joint_safety_margin;  double thumb_joint_3_safe_min = thumb_joint_3_min+joint_safety_margin;
	
	double index_joint_0_safe_range = index_joint_0_safe_max-index_joint_0_safe_min;
	double index_joint_1_safe_range = index_joint_1_safe_max-index_joint_1_safe_min;
	double index_joint_2_safe_range = index_joint_2_safe_max-index_joint_2_safe_min;
	double index_joint_3_safe_range = index_joint_3_safe_max-index_joint_3_safe_min;
	
	double thumb_joint_0_safe_range = thumb_joint_0_safe_max-thumb_joint_0_safe_min;
  double thumb_joint_1_safe_range = thumb_joint_1_safe_max-thumb_joint_1_safe_min;
  double thumb_joint_2_safe_range = thumb_joint_2_safe_max-thumb_joint_2_safe_min;
  double thumb_joint_3_safe_range = thumb_joint_3_safe_max-thumb_joint_3_safe_min;
  
  Eigen::Vector3d position_Ptt, position_Ptt_desired, position_Ptt_init, position_Ptt_traj, position_Ptt_error;
  Eigen::Vector3d position_Pit, position_Pit_desired, position_Pit_init, position_Pit_traj, position_Pit_error;
  Eigen::Vector3d position_Pmt, position_Pmt_desired, position_Pmt_init, position_Pmt_traj, position_Pmt_error;
  Eigen::Vector3d position_Ppt, position_Ppt_desired, position_Ppt_init, position_Ppt_traj, position_Ppt_error;
  
  Eigen::Vector3d velocity_Ptt_desired_3d, velocity_Pit_desired_3d, velocity_Pmt_desired_3d, velocity_Ppt_desired_3d;
  
  
  
  
  
  
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

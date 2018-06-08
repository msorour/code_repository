
rosrun franka_panda_control_gazebo franka_panda_gazebo_joint_controller & roslaunch franka_panda_control_gazebo trial.launch & gzserver -u ../worlds/sim_scenario#1.world --verbose & gzclient


#roslaunch franka_panda_control_gazebo gazebo_launch.launch & roslaunch franka_panda_control_gazebo robot_description.launch & roslaunch franka_panda_control_gazebo gazebo_plugin.launch & roslaunch franka_panda_control_gazebo robot_spawn.launch


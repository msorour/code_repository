#include"Eigen/Dense"
#include <iostream>
#include <string>
#include <fstream>
//#include "useful_implementations.h"
#include "franka_panda_am_parameters.h"


Eigen::Matrix4d arm_direct_geometric_model(Eigen::VectorXd joint_state, std::string effector_frame);
Eigen::MatrixXd arm_geometric_jacobian_matrix(Eigen::VectorXd joint_state, std::string reference_frame);
Eigen::MatrixXd arm_inertia_matrix(Eigen::VectorXd joint_state);
Eigen::VectorXd arm_viscous_friction_torque(Eigen::VectorXd joint_velocity);
Eigen::VectorXd arm_static_friction_torque(Eigen::VectorXd joint_velocity);
Eigen::VectorXd arm_coriolis_centrifugal_torque(Eigen::VectorXd joint_state, Eigen::VectorXd joint_velocity);
Eigen::Matrix4d base_to_arm_frame0(void);
Eigen::Matrix4d arm_frame7_to_tip(void);
Eigen::Matrix4d arm_tip_to_panda_gripper(void);



Eigen::Matrix4d arm_direct_geometric_model(Eigen::VectorXd joint_state, std::string effector_frame="end_tip"){
	double t1, t2, t3, t4, t5, t6, t7;
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	t5 = joint_state(4);
	t6 = joint_state(5);
	t7 = joint_state(6);
	double T0T711,T0T721,T0T731, T0T712,T0T722,T0T732, T0T713,T0T723,T0T733, T0T714,T0T724,T0T734;
	double sx, sy, sz, nx, ny, nz, ax, ay, az, Px, Py, Pz;
	/*
	// Column#1
	T0T711 = ((-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*sin(t6))*cos(t7) + (-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*sin(t5) + (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t5))*sin(t7);
	T0T721 = ((-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6))*cos(t7) + (-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*sin(t5) + (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t5))*sin(t7);
	T0T731 = ((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6))*cos(t7) + (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t5) - sin(t2)*sin(t3)*cos(t5))*sin(t7);
	
	// Column#2
	T0T712 = -((-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*sin(t6))*sin(t7) + (-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*sin(t5) + (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t5))*cos(t7);
	T0T722 = -((-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6))*sin(t7) + (-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*sin(t5) + (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t5))*cos(t7);
	T0T732 = -((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6))*sin(t7) + (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t5) - sin(t2)*sin(t3)*cos(t5))*cos(t7);
	
	// Column#3
	T0T713 = (-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t5))*sin(t6) - ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*cos(t6);
	T0T723 = (-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*sin(t6) - ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*cos(t6);
	T0T733 = (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*sin(t6) - (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*cos(t6);
	
	// Column#4
	T0T714 = D*((-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) - (-sin(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*sin(t6)) + D*((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1)) - D*(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3)) - RL3*sin(t2)*cos(t1) + RL5*((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4));
	T0T724 = D*((-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6)) + D*((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4)) - D*(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1)) - RL3*sin(t1)*sin(t2) + RL5*((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4));
	T0T734 = D*((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6)) + D*(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2)) - D*sin(t2)*cos(t3) + RL3*cos(t2) + RL5*(sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4));
	
	
	Eigen::Matrix4d DGM;
	DGM << T0T711, T0T712, T0T713, T0T714,
				 T0T721, T0T722, T0T723, T0T724,
				 T0T731, T0T732, T0T733, T0T734,
				 			0, 			0, 			0, 		  1;
	
	*/
	
	
	sx = ((-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) + (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*sin(t6))*cos(t7) + (-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*sin(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*cos(t5))*sin(t7);
  sy = ((-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6))*cos(t7) + (-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*sin(t5) + (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t5))*sin(t7);
  sz = ((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6))*cos(t7) + (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t5) - sin(t2)*sin(t3)*cos(t5))*sin(t7);
  nx = -((-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) + (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*sin(t6))*sin(t7) + (-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*sin(t5) - (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*cos(t5))*cos(t7);
  ny = -((-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6))*sin(t7) + (-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*sin(t5) + (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*cos(t5))*cos(t7);
  nz = -((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6))*sin(t7) + (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t5) - sin(t2)*sin(t3)*cos(t5))*cos(t7);
  ax = (-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) + (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*sin(t6) - ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*cos(t6);
  ay = (-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*sin(t6) - ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*cos(t6);
  az = (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*sin(t6) - (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*cos(t6);
  Px = D*((-((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1))*cos(t5) + (sin(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*sin(t5))*cos(t6) + ((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) - sin(t2)*cos(t1)*cos(t4))*sin(t6)) + D*((-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + sin(t2)*sin(t4)*cos(t1)) - D*(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3)) - RL3*sin(t2)*cos(t1) - RL5*(-(-sin(t1)*sin(t3) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + sin(t2)*cos(t1)*cos(t4));
  Py = D*((-((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4))*cos(t5) - (-sin(t1)*sin(t3)*cos(t2) + cos(t1)*cos(t3))*sin(t5))*cos(t6) + ((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) - sin(t1)*sin(t2)*cos(t4))*sin(t6)) + D*((sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t2)*sin(t4)) - D*(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1)) - RL3*sin(t1)*sin(t2) - RL5*(-(sin(t1)*cos(t2)*cos(t3) + sin(t3)*cos(t1))*sin(t4) + sin(t1)*sin(t2)*cos(t4));
  Pz = D*((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6)) + D*(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2)) - D*sin(t2)*cos(t3) + RL3*cos(t2) + RL5*(sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4));

  Eigen::Matrix4d DGM, overall_DGM;
	DGM << sx, nx, ax, Px,
				 sy, ny, ay, Py,
				 sz, nz, az, Pz,
				 	0,	0,	0,  1;
	
	if(effector_frame=="panda_gripper")
	  overall_DGM = base_to_arm_frame0()*DGM*arm_frame7_to_tip()*arm_tip_to_panda_gripper();
	else if(effector_frame=="allegro_hand")
	  overall_DGM = base_to_arm_frame0()*DGM*arm_frame7_to_tip();
	else if(effector_frame=="end_tip")
	  overall_DGM = base_to_arm_frame0()*DGM*arm_frame7_to_tip();
	return overall_DGM;
}







// this function computes the geometric jacobian relating the velocity of the end effector frame to the base frame.
// the second argument specifies if you want the effector velocity projected in its own frame "effector" or in the base frame "base"
Eigen::MatrixXd arm_geometric_jacobian_matrix(Eigen::VectorXd joint_state, std::string reference_frame="base"){
	double t1, t2, t3, t4, t5, t6, t7;
	
	double J11,J12,J13,J14,J15,J16,J17;
	double J21,J22,J23,J24,J25,J26,J27;
	double J31,J32,J33,J34,J35,J36,J37;
	double J41,J42,J43,J44,J45,J46,J47;
	double J51,J52,J53,J54,J55,J56,J57;
	double J61,J62,J63,J64,J65,J66,J67;
	
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	t5 = joint_state(4);
	t6 = joint_state(5);
	t7 = joint_state(6);
	
	/*
	// generated by me not SYMORO!
	J11 = (-((-(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*cos(t5) + sin(t3)*sin(t5)*cos(t2))*cos(t6) + (-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*sin(t6))*cos(t7) - (-(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*sin(t5) - sin(t3)*cos(t2)*cos(t5))*sin(t7))*(D*(-(sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*cos(t6) + sin(t3)*sin(t4)*sin(t6)) + D*sin(t3)*cos(t4) - D*sin(t3) + RL5*sin(t3)*sin(t4)) + ((-(sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*cos(t6) + sin(t3)*sin(t4)*sin(t6))*cos(t7) + (-sin(t3)*sin(t5)*cos(t4) + cos(t3)*cos(t5))*sin(t7))*(D*((-(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*cos(t5) + sin(t3)*sin(t5)*cos(t2))*cos(t6) + (-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*sin(t6)) + D*(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4)) - D*cos(t2)*cos(t3) - RL3*sin(t2) - RL5*(sin(t2)*cos(t4) - sin(t4)*cos(t2)*cos(t3)));
  J21 = (((-(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*cos(t5) + sin(t3)*sin(t5)*cos(t2))*cos(t6) + (-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*sin(t6))*sin(t7) - (-(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*sin(t5) - sin(t3)*cos(t2)*cos(t5))*cos(t7))*(D*(-(sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*cos(t6) + sin(t3)*sin(t4)*sin(t6)) + D*sin(t3)*cos(t4) - D*sin(t3) + RL5*sin(t3)*sin(t4)) + (-(-(sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*cos(t6) + sin(t3)*sin(t4)*sin(t6))*sin(t7) + (-sin(t3)*sin(t5)*cos(t4) + cos(t3)*cos(t5))*cos(t7))*(D*((-(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*cos(t5) + sin(t3)*sin(t5)*cos(t2))*cos(t6) + (-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*sin(t6)) + D*(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4)) - D*cos(t2)*cos(t3) - RL3*sin(t2) - RL5*(sin(t2)*cos(t4) - sin(t4)*cos(t2)*cos(t3)));
  J31 = D*sin(t2)*sin(t3)*cos(t4)*cos(t6) + D*sin(t2)*sin(t3)*cos(t5) - D*sin(t2)*sin(t3)*cos(t6) - D*sin(t2)*sin(t4)*sin(t6)*sin(t3 + t5) + D*sin(t2)*sin(t5)*cos(t3)*cos(t4) - D*sin(t4)*sin(t5)*cos(t2) - D*sin(t5)*sin(t6)*cos(t2)*cos(t4) + D*sin(t5)*sin(t6)*cos(t2) + RL3*sin(t2)*sin(t3)*sin(t4)*cos(t6) + RL3*sin(t2)*sin(t3)*sin(t6)*cos(t4)*cos(t5) + RL3*sin(t2)*sin(t5)*sin(t6)*cos(t3) + RL5*sin(t2)*sin(t3)*sin(t6)*cos(t5) + RL5*sin(t2)*sin(t5)*sin(t6)*cos(t3)*cos(t4) - RL5*sin(t4)*sin(t5)*sin(t6)*cos(t2);
  J41 = ((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6))*cos(t7) + (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t5) - sin(t2)*sin(t3)*cos(t5))*sin(t7);
  J51 = -((-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*cos(t6) + (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*sin(t6))*sin(t7) + (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*sin(t5) - sin(t2)*sin(t3)*cos(t5))*cos(t7);
  J61 = (-(sin(t2)*cos(t3)*cos(t4) - sin(t4)*cos(t2))*cos(t5) + sin(t2)*sin(t3)*sin(t5))*sin(t6) - (sin(t2)*sin(t4)*cos(t3) + cos(t2)*cos(t4))*cos(t6);

  J12 = (-((sin(t3)*sin(t5) - cos(t3)*cos(t4)*cos(t5))*cos(t6) + sin(t4)*sin(t6)*cos(t3))*cos(t7) + (sin(t3)*cos(t5) + sin(t5)*cos(t3)*cos(t4))*sin(t7))*(D*(sin(t4)*cos(t5)*cos(t6) + sin(t6)*cos(t4)) - D*sin(t4) + RL3 + RL5*cos(t4)) + ((sin(t4)*cos(t5)*cos(t6) + sin(t6)*cos(t4))*cos(t7) + sin(t4)*sin(t5)*sin(t7))*(D*((sin(t3)*sin(t5) - cos(t3)*cos(t4)*cos(t5))*cos(t6) + sin(t4)*sin(t6)*cos(t3)) + D*cos(t3)*cos(t4) - D*cos(t3) + RL5*sin(t4)*cos(t3));
  J22 = (((sin(t3)*sin(t5) - cos(t3)*cos(t4)*cos(t5))*cos(t6) + sin(t4)*sin(t6)*cos(t3))*sin(t7) + (sin(t3)*cos(t5) + sin(t5)*cos(t3)*cos(t4))*cos(t7))*(D*(sin(t4)*cos(t5)*cos(t6) + sin(t6)*cos(t4)) - D*sin(t4) + RL3 + RL5*cos(t4)) + (-(sin(t4)*cos(t5)*cos(t6) + sin(t6)*cos(t4))*sin(t7) + sin(t4)*sin(t5)*cos(t7))*(D*((sin(t3)*sin(t5) - cos(t3)*cos(t4)*cos(t5))*cos(t6) + sin(t4)*sin(t6)*cos(t3)) + D*cos(t3)*cos(t4) - D*cos(t3) + RL5*sin(t4)*cos(t3));
  J32 = -D*sin(t3)*sin(t5)*cos(t4) - D*sin(t4)*sin(t6)*cos(t3 + t5) + D*cos(t3)*cos(t4)*cos(t6) + D*cos(t3)*cos(t5) - D*cos(t3)*cos(t6) - RL3*sin(t3)*sin(t5)*sin(t6) + RL3*sin(t4)*cos(t3)*cos(t6) + RL3*sin(t6)*cos(t3)*cos(t4)*cos(t5) - RL5*sin(t3)*sin(t5)*sin(t6)*cos(t4) + RL5*sin(t6)*cos(t3)*cos(t5);
  J42 = ((sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*cos(t6) - sin(t3)*sin(t4)*sin(t6))*cos(t7) + (sin(t3)*sin(t5)*cos(t4) - cos(t3)*cos(t5))*sin(t7);
  J52 = -((sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*cos(t6) - sin(t3)*sin(t4)*sin(t6))*sin(t7) + (sin(t3)*sin(t5)*cos(t4) - cos(t3)*cos(t5))*cos(t7);
  J62 = (sin(t3)*cos(t4)*cos(t5) + sin(t5)*cos(t3))*sin(t6) + sin(t3)*sin(t4)*cos(t6);

  J13 = -D*(-(sin(t4)*sin(t6) - cos(t4)*cos(t5)*cos(t6))*cos(t7) + sin(t5)*sin(t7)*cos(t4))*sin(t5)*cos(t6) + (-sin(t5)*cos(t6)*cos(t7) + sin(t7)*cos(t5))*(D*(sin(t4)*sin(t6) - cos(t4)*cos(t5)*cos(t6)) + D*cos(t4) - D + RL5*sin(t4));
  J23 = -D*((sin(t4)*sin(t6) - cos(t4)*cos(t5)*cos(t6))*sin(t7) + sin(t5)*cos(t4)*cos(t7))*sin(t5)*cos(t6) + (sin(t5)*sin(t7)*cos(t6) + cos(t5)*cos(t7))*(D*(sin(t4)*sin(t6) - cos(t4)*cos(t5)*cos(t6)) + D*cos(t4) - D + RL5*sin(t4));
  J33 = (-D*sin(t4) - D*sin(t6)*cos(t4) + D*sin(t6) - RL5*sin(t4)*sin(t6))*sin(t5);
  J43 = (sin(t4)*cos(t5)*cos(t6) + sin(t6)*cos(t4))*cos(t7) + sin(t4)*sin(t5)*sin(t7);
  J53 = -(sin(t4)*cos(t5)*cos(t6) + sin(t6)*cos(t4))*sin(t7) + sin(t4)*sin(t5)*cos(t7);
  J63 = sin(t4)*sin(t6)*cos(t5) - cos(t4)*cos(t6);

  J14 = -(D*sin(t5)*sin(t6)*sin(t7) + D*sin(t6)*cos(t7) + RL5*sin(t5)*sin(t7) + RL5*cos(t5)*cos(t6)*cos(t7));
  J24 = -D*sin(t5)*sin(t6)*cos(t7) + D*sin(t6)*sin(t7) - RL5*sin(t5)*cos(t7) + RL5*sin(t7)*cos(t5)*cos(t6);
  J34 = -D*cos(t5) + D*cos(t6) - RL5*sin(t6)*cos(t5);
  J44 = -sin(t5)*cos(t6)*cos(t7) + sin(t7)*cos(t5);
  J54 = sin(t5)*sin(t7)*cos(t6) + cos(t5)*cos(t7);
  J64 = -sin(t5)*sin(t6);

  J15 = -D*sin(t7)*cos(t6);
  J25 = -D*cos(t6)*cos(t7);
  J35 = 0;
  J45 = sin(t6)*cos(t7);
  J55 = -sin(t6)*sin(t7);
  J65 = -cos(t6);

  J16 = 0;
  J26 = 0;
  J36 = -D;
  J46 = sin(t7);
  J56 = cos(t7);
  J66 = 0;

  J17 = 0;
  J27 = 0;
  J37 = 0;
  J47 = 0;
  J57 = 0;
  J67 = 1;
  */
  
  J11 = -D*sin(t2)*sin(t3)*sin(t4)*cos(t5)*cos(t6)*cos(t7) + D*sin(t2)*sin(t3)*sin(t5)*sin(t6)*sin(t7) - D*sin(t2)*sin(t3)*sin(t6)*cos(t4)*cos(t7) + D*sin(t2)*sin(t3)*sin(t6)*cos(t7) - D*sin(t2)*sin(t4)*sin(t5)*cos(t3)*cos(t6)*cos(t7) - D*sin(t2)*sin(t4)*sin(t7)*cos(t3)*cos(t6) + D*sin(t2)*sin(t4)*sin(t7)*cos(t3 + t5) - D*sin(t2)*sin(t6)*sin(t7)*cos(t3)*cos(t4)*cos(t5) + D*sin(t4)*sin(t6)*sin(t7)*cos(t2)*cos(t5) - D*sin(t5)*cos(t2)*cos(t4)*cos(t6)*cos(t7) + D*sin(t5)*cos(t2)*cos(t6)*cos(t7) + D*sin(t7)*cos(t2)*cos(t4)*cos(t5) - D*sin(t7)*cos(t2)*cos(t4)*cos(t6) - D*sin(t7)*cos(t2)*cos(t5) - RL3*sin(t2)*sin(t3)*sin(t4)*sin(t6)*cos(t7) + RL3*sin(t2)*sin(t3)*sin(t5)*sin(t7)*cos(t4) + RL3*sin(t2)*sin(t3)*cos(t4)*cos(t5)*cos(t6)*cos(t7) + RL3*sin(t2)*sin(t5)*cos(t3)*cos(t6)*cos(t7) - RL3*sin(t2)*sin(t7)*cos(t3)*cos(t5) + RL5*sin(t2)*sin(t3)*sin(t5)*sin(t7) + RL5*sin(t2)*sin(t3)*cos(t5)*cos(t6)*cos(t7) + RL5*sin(t2)*sin(t5)*cos(t3)*cos(t4)*cos(t6)*cos(t7) - RL5*sin(t2)*sin(t7)*cos(t3)*cos(t4)*cos(t5) - RL5*sin(t4)*sin(t5)*cos(t2)*cos(t6)*cos(t7) + RL5*sin(t4)*sin(t7)*cos(t2)*cos(t5);
J21 = D*sin(t2)*sin(t3)*sin(t5)*sin(t6)*cos(t7) + D*sin(t2)*sin(t3)*sin(t6)*sin(t7)*cos(t4) - D*sin(t2)*sin(t3)*sin(t6)*sin(t7) + D*sin(t2)*sin(t4)*sin(t7)*sin(t3 + t5)*cos(t6) - D*sin(t2)*sin(t4)*cos(t3)*cos(t6)*cos(t7) + D*sin(t2)*sin(t4)*cos(t7)*cos(t3 + t5) - D*sin(t2)*sin(t6)*cos(t3)*cos(t4)*cos(t5)*cos(t7) + D*sin(t4)*sin(t6)*cos(t2)*cos(t5)*cos(t7) + D*sin(t5)*sin(t7)*cos(t2)*cos(t4)*cos(t6) - D*sin(t5)*sin(t7)*cos(t2)*cos(t6) + D*cos(t2)*cos(t4)*cos(t5)*cos(t7) - D*cos(t2)*cos(t4)*cos(t6)*cos(t7) - D*cos(t2)*cos(t5)*cos(t7) + RL3*sin(t2)*sin(t3)*sin(t4)*sin(t6)*sin(t7) + RL3*sin(t2)*sin(t3)*sin(t5)*cos(t4)*cos(t7) - RL3*sin(t2)*sin(t3)*sin(t5)*cos(t7) - RL3*sin(t2)*sin(t3)*sin(t7)*cos(t4)*cos(t5)*cos(t6) - RL3*sin(t2)*sin(t5)*sin(t7)*cos(t3)*cos(t6) - RL3*sin(t2)*cos(t7)*cos(t3 + t5) + RL5*sin(t2)*sin(t3)*sin(t5)*cos(t7) - RL5*sin(t2)*sin(t3)*sin(t7)*cos(t5)*cos(t6) - RL5*sin(t2)*sin(t5)*sin(t7)*cos(t3)*cos(t4)*cos(t6) - RL5*sin(t2)*cos(t3)*cos(t4)*cos(t5)*cos(t7) + RL5*sin(t4)*sin(t5)*sin(t7)*cos(t2)*cos(t6) + RL5*sin(t4)*cos(t2)*cos(t5)*cos(t7);
J31 = -D*sin(t2)*sin(t3)*sin(t4)*sin(t6)*cos(t5) + D*sin(t2)*sin(t3)*cos(t4)*cos(t6) + D*sin(t2)*sin(t3)*cos(t5) - D*sin(t2)*sin(t3)*cos(t6) - D*sin(t2)*sin(t4)*sin(t5)*sin(t6)*cos(t3) + D*sin(t2)*sin(t5)*cos(t3)*cos(t4) - D*sin(t4)*sin(t5)*cos(t2) - D*sin(t5)*sin(t6)*cos(t2)*cos(t4) + D*sin(t5)*sin(t6)*cos(t2) + RL3*sin(t2)*sin(t3)*sin(t4)*cos(t6) + RL3*sin(t2)*sin(t3)*sin(t6)*cos(t4)*cos(t5) + RL3*sin(t2)*sin(t5)*sin(t6)*cos(t3) + RL5*sin(t2)*sin(t3)*sin(t6)*cos(t5) + RL5*sin(t2)*sin(t5)*sin(t6)*cos(t3)*cos(t4) - RL5*sin(t4)*sin(t5)*sin(t6)*cos(t2);
J41 = sin(t2)*sin(t3)*sin(t5)*cos(t6)*cos(t7) - sin(t2)*sin(t3)*sin(t7)*cos(t5) + sin(t2)*sin(t4)*sin(t6)*cos(t3)*cos(t7) - sin(t2)*sin(t5)*sin(t7)*cos(t3)*cos(t4) - sin(t2)*cos(t3)*cos(t4)*cos(t5)*cos(t6)*cos(t7) + sin(t4)*sin(t5)*sin(t7)*cos(t2) + sin(t4)*cos(t2)*cos(t5)*cos(t6)*cos(t7) + sin(t6)*cos(t2)*cos(t4)*cos(t7);
J51 = -sin(t2)*sin(t3)*sin(t5)*sin(t7)*cos(t6) - sin(t2)*sin(t3)*cos(t5)*cos(t7) - sin(t2)*sin(t4)*sin(t6)*sin(t7)*cos(t3) - sin(t2)*sin(t5)*cos(t3)*cos(t4)*cos(t7) + sin(t2)*sin(t7)*cos(t3)*cos(t4)*cos(t5)*cos(t6) + sin(t4)*sin(t5)*cos(t2)*cos(t7) - sin(t4)*sin(t7)*cos(t2)*cos(t5)*cos(t6) - sin(t6)*sin(t7)*cos(t2)*cos(t4);
J61 = sin(t2)*sin(t3)*sin(t5)*sin(t6) - sin(t2)*sin(t4)*cos(t3)*cos(t6) - sin(t2)*sin(t6)*cos(t3)*cos(t4)*cos(t5) + sin(t4)*sin(t6)*cos(t2)*cos(t5) - cos(t2)*cos(t4)*cos(t6);

J12 = -D*sin(t3)*sin(t4)*sin(t7)*cos(t5) + D*sin(t3)*sin(t4)*sin(t7)*cos(t6) + D*sin(t3)*sin(t6)*sin(t7)*cos(t4)*cos(t5) - D*sin(t4)*sin(t5)*sin(t7)*cos(t3) - D*sin(t4)*cos(t6)*cos(t7)*cos(t3 + t5) + D*sin(t5)*sin(t6)*sin(t7)*cos(t3) - D*sin(t6)*cos(t3)*cos(t4)*cos(t7) + D*sin(t6)*cos(t3)*cos(t7) - RL3*sin(t3)*sin(t5)*cos(t6)*cos(t7) + RL3*sin(t3)*sin(t7)*cos(t5) - RL3*sin(t4)*sin(t6)*cos(t3)*cos(t7) + RL3*sin(t5)*sin(t7)*cos(t3)*cos(t4) + RL3*cos(t3)*cos(t4)*cos(t5)*cos(t6)*cos(t7) - RL5*sin(t3)*sin(t5)*cos(t4)*cos(t6)*cos(t7) + RL5*sin(t3)*sin(t7)*cos(t4)*cos(t5) + RL5*sin(t5)*sin(t7)*cos(t3) + RL5*cos(t3)*cos(t5)*cos(t6)*cos(t7);
J22 = -D*sin(t3)*sin(t4)*cos(t5)*cos(t7) + D*sin(t3)*sin(t4)*cos(t6)*cos(t7) + D*sin(t3)*sin(t6)*cos(t4)*cos(t5)*cos(t7) - D*sin(t4)*sin(t5)*cos(t3)*cos(t7) + D*sin(t4)*sin(t7)*cos(t6)*cos(t3 + t5) + D*sin(t5)*sin(t6)*cos(t3)*cos(t7) + D*sin(t6)*sin(t7)*cos(t3)*cos(t4) - D*sin(t6)*sin(t7)*cos(t3) + RL3*sin(t3)*sin(t5)*sin(t7)*cos(t6) + RL3*sin(t3)*cos(t5)*cos(t7) + RL3*sin(t4)*sin(t6)*sin(t7)*cos(t3) + RL3*sin(t5)*cos(t3)*cos(t4)*cos(t7) - RL3*sin(t7)*cos(t3)*cos(t4)*cos(t5)*cos(t6) + RL5*sin(t3)*sin(t5)*sin(t7)*cos(t4)*cos(t6) + RL5*sin(t3)*cos(t4)*cos(t5)*cos(t7) + RL5*sin(t5)*cos(t3)*cos(t7) - RL5*sin(t7)*cos(t3)*cos(t5)*cos(t6);
J32 = D*sin(t3)*sin(t4)*sin(t5)*sin(t6) - D*sin(t3)*sin(t5)*cos(t4) - D*sin(t4)*sin(t6)*cos(t3)*cos(t5) + D*cos(t3)*cos(t4)*cos(t6) + D*cos(t3)*cos(t5) - D*cos(t3)*cos(t6) - RL3*sin(t3)*sin(t5)*sin(t6) + RL3*sin(t4)*cos(t3)*cos(t6) + RL3*sin(t6)*cos(t3)*cos(t4)*cos(t5) - RL5*sin(t3)*sin(t5)*sin(t6)*cos(t4) + RL5*sin(t6)*cos(t3)*cos(t5);
J42 = -sin(t3)*sin(t4)*sin(t6)*cos(t7) + sin(t3)*sin(t5)*sin(t7)*cos(t4) + sin(t3)*cos(t4)*cos(t5)*cos(t6)*cos(t7) + sin(t5)*cos(t3)*cos(t6)*cos(t7) - sin(t7)*cos(t3)*cos(t5);
J52 = sin(t3)*sin(t4)*sin(t6)*sin(t7) + sin(t3)*sin(t5)*cos(t4)*cos(t7) - sin(t3)*sin(t7)*cos(t4)*cos(t5)*cos(t6) - sin(t5)*sin(t7)*cos(t3)*cos(t6) - cos(t3)*cos(t5)*cos(t7);
J62 = sin(t3)*sin(t4)*cos(t6) + sin(t3)*sin(t6)*cos(t4)*cos(t5) + sin(t5)*sin(t6)*cos(t3);

J13 = D*sin(t4)*sin(t6)*sin(t7)*cos(t5) - D*sin(t5)*cos(t4)*cos(t6)*cos(t7) + D*sin(t5)*cos(t6)*cos(t7) + D*sin(t7)*cos(t4)*cos(t5) - D*sin(t7)*cos(t4)*cos(t6) - D*sin(t7)*cos(t5) - RL5*sin(t4)*sin(t5)*cos(t6)*cos(t7) + RL5*sin(t4)*sin(t7)*cos(t5);
J23 = D*sin(t4)*sin(t6)*cos(t5)*cos(t7) + D*sin(t5)*sin(t7)*cos(t4)*cos(t6) - D*sin(t5)*sin(t7)*cos(t6) + D*cos(t4)*cos(t5)*cos(t7) - D*cos(t4)*cos(t6)*cos(t7) - D*cos(t5)*cos(t7) + RL5*sin(t4)*sin(t5)*sin(t7)*cos(t6) + RL5*sin(t4)*cos(t5)*cos(t7);
J33 = (-D*sin(t4) - D*sin(t6)*cos(t4) + D*sin(t6) - RL5*sin(t4)*sin(t6))*sin(t5);
J43 = sin(t4)*sin(t5)*sin(t7) + sin(t4)*cos(t5)*cos(t6)*cos(t7) + sin(t6)*cos(t4)*cos(t7);
J53 = sin(t4)*sin(t5)*cos(t7) - sin(t4)*sin(t7)*cos(t5)*cos(t6) - sin(t6)*sin(t7)*cos(t4);
J63 = sin(t4)*sin(t6)*cos(t5) - cos(t4)*cos(t6);

J14 = -D*sin(t5)*sin(t6)*sin(t7) - D*sin(t6)*cos(t7) - RL5*sin(t5)*sin(t7) - RL5*cos(t5)*cos(t6)*cos(t7);
J24 = -D*sin(t5)*sin(t6)*cos(t7) + D*sin(t6)*sin(t7) - RL5*sin(t5)*cos(t7) + RL5*sin(t7)*cos(t5)*cos(t6);
J34 = -D*cos(t5) + D*cos(t6) - RL5*sin(t6)*cos(t5);
J44 = -sin(t5)*cos(t6)*cos(t7) + sin(t7)*cos(t5);
J54 = sin(t5)*sin(t7)*cos(t6) + cos(t5)*cos(t7);
J64 = -sin(t5)*sin(t6);

J15 = -D*sin(t7)*cos(t6);
J25 = -D*cos(t6)*cos(t7);
J35 = 0;
J45 = sin(t6)*cos(t7);
J55 = -sin(t6)*sin(t7);
J65 = -cos(t6);

J16 = 0;
J26 = 0;
J36 = -D;
J46 = sin(t7);
J56 = cos(t7);
J66 = 0;

J17 = 0;
J27 = 0;
J37 = 0;
J47 = 0;
J57 = 0;
J67 = 1;
  
 
	Eigen::MatrixXd geometric_jacobian(6,7);
	
	// this gives the jacobian matrix of the arm frame7 w.r.t arm frame0 
	geometric_jacobian << J11, J12, J13, J14, J15, J16, J17,
												J21, J22, J23, J24, J25, J26, J27,
												J31, J32, J33, J34, J35, J36, J37,
												J41, J42, J43, J44, J45, J46, J47,
												J51, J52, J53, J54, J55, J56, J57,
												J61, J62, J63, J64, J65, J66, J67;
	
	// screw transformation matrix relating frame7 to End effector
	Eigen::Vector3d r_7E, r_E7;
	Eigen::Matrix3d R_E7, r_7E_hat, product;
	Eigen::Matrix3d R_7E  = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d zero3 = Eigen::Matrix3d::Zero();
	Eigen::MatrixXd screw_TM_E7(6,6);
	r_7E << 0, 0, db;
	R_E7 = R_7E.transpose();
	r_7E_hat = skew_symm_matrix(r_7E);
	product = -R_E7*r_7E_hat;
	
	screw_TM_E7 << R_E7 , product,
								 zero3,    R_E7;
	
	// screw transformation matrix relating End effector to base frame
	Eigen::Matrix4d T_bE;
	Eigen::Matrix3d R_bE;
	Eigen::MatrixXd screw_TM_bE(6,6);
	T_bE = arm_direct_geometric_model(joint_state);
	R_bE << T_bE(0,0), T_bE(0,1), T_bE(0,2),
				  T_bE(1,0), T_bE(1,1), T_bE(1,2),
				  T_bE(2,0), T_bE(2,1), T_bE(2,2);
	
	screw_TM_bE << R_bE , zero3,
								 zero3,  R_bE;
	
	if(reference_frame=="base")
		return screw_TM_bE*screw_TM_E7*geometric_jacobian;
	else if(reference_frame=="effector")
		return screw_TM_E7*geometric_jacobian;
}
















/*

Eigen::MatrixXd geometric_jacobian_derivative(Eigen::MatrixXd jacobian_past(6,7), double time_now, double time_past){
	Eigen::MatrixXd jacobian_now(6,7);
	Eigen::MatrixXd jacobian_derivative(6,7);
	
	jacobian_derivative = (jacobian_now-jacobian_past)/(time_now-time_past);
	return jacobian_derivative;
}
*/
/*
Eigen::MatrixXd geometric_jacobian_derivative(Eigen::VectorXd joint_state){
	double t1, t2, t3, t4, t5, t6, t7;
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	t5 = joint_state(4);
	t6 = joint_state(5);
	t7 = joint_state(6);

double C1 = cos(t1);
double S1 = sin(t1);
double C2 = cos(t2);
double S2 = sin(t2);
double C3 = cos(t3);
double S3 = sin(t3);
double C4 = cos(t4);
double S4 = sin(t4);
double C5 = cos(t5);
double S5 = sin(t5);
double C6 = cos(t6);
double S6 = sin(t6);
double C7 = cos(t7);
double S7 = sin(t7);
double WPJ11 = 0;
double WPJ21 = 0;
double WPJ31 = 0;
double DV61 = qd1**2;
double VPJ11 = 0;
double VPJ21 = 0;
double VPJ31 = 0;
double W12 = S2*qd1;
double W22 = C2*qd1;
double WPJ12 = C2*WPJ11 + S2*WPJ31 + W22*qd2;
double WPJ22 = C2*WPJ31 - S2*WPJ11 - W12*qd2;
double WPJ32 = -WPJ21;
double DV12 = W12**2;
double DV22 = W12*W22;
double DV32 = W12*qd2;
double DV42 = W22**2;
double DV52 = W22*qd2;
double DV62 = qd2**2;
double U112 = -DV42 - DV62;
double U212 = DV22 + WPJ32;
double U312 = DV32 - WPJ22;
double U122 = DV22 - WPJ32;
double U222 = -DV12 - DV62;
double U322 = DV52 + WPJ12;
double U132 = DV32 + WPJ22;
double U232 = DV52 - WPJ12;
double U332 = -DV12 - DV42;
double VPJ12 = C2*VPJ11 + S2*VPJ31;
double VPJ22 = C2*VPJ31 - S2*VPJ11;
double VPJ32 = -VPJ21;
double W13 = C3*W12 - S3*qd2;
double W23 = -C3*qd2 - S3*W12;
double W33 = W22 + qd3;
double WPJ13 = C3*WPJ12 - S3*WPJ32 + W23*qd3;
double WPJ23 = -C3*WPJ32 - S3*WPJ12 - W13*qd3;
double WPJ33 = WPJ22;
double DV13 = W13**2;
double DV23 = W13*W23;
double DV33 = W13*W33;
double DV43 = W23**2;
double DV53 = W23*W33;
double DV63 = W33**2;
double U113 = -DV43 - DV63;
double U213 = DV23 + WPJ33;
double U313 = DV33 - WPJ23;
double U123 = DV23 - WPJ33;
double U223 = -DV13 - DV63;
double U323 = DV53 + WPJ13;
double U133 = DV33 + WPJ23;
double U233 = DV53 - WPJ13;
double U333 = -DV13 - DV43;
double VSP13 = RL3*U122 + VPJ12;
double VSP23 = RL3*U222 + VPJ22;
double VSP33 = RL3*U322 + VPJ32;
double VPJ13 = C3*VSP13 - S3*VSP33;
double VPJ23 = -C3*VSP33 - S3*VSP13;
double VPJ33 = VSP23;
double W14 = C4*W13 - S4*W33;
double W24 = -C4*W33 - S4*W13;
double W34 = W23 + qd4;
double WPJ14 = C4*WPJ13 - S4*WPJ33 + W24*qd4;
double WPJ24 = -C4*WPJ33 - S4*WPJ13 - W14*qd4;
double WPJ34 = WPJ23;
DV14 = W14**2;
DV24 = W14*W24;
DV34 = W14*W34;
DV44 = W24**2;
DV54 = W24*W34;
DV64 = W34**2;
U114 = -DV44 - DV64;
U214 = DV24 + WPJ34;
U314 = DV34 - WPJ24;
U124 = DV24 - WPJ34;
U224 = -DV14 - DV64;
U324 = DV54 + WPJ14;
U134 = DV34 + WPJ24;
U234 = DV54 - WPJ14;
U334 = -DV14 - DV44;
VSP14 = -D*U113 + VPJ13;
VSP24 = -D*U213 + VPJ23;
VSP34 = -D*U313 + VPJ33;
VPJ14 = C4*VSP14 - S4*VSP34;
VPJ24 = -C4*VSP34 - S4*VSP14;
VPJ34 = VSP24;
W15 = -C5*W14 - S5*W34;
W25 = -C5*W34 + S5*W14;
W35 = -W24 + qd5;
WPJ15 = -C5*WPJ14 - S5*WPJ34 + W25*qd5;
WPJ25 = -C5*WPJ34 + S5*WPJ14 - W15*qd5;
WPJ35 = -WPJ24;
DV15 = W15**2;
DV25 = W15*W25;
DV35 = W15*W35;
DV45 = W25**2;
DV55 = W25*W35;
DV65 = W35**2;
U115 = -DV45 - DV65;
U215 = DV25 + WPJ35;
U315 = DV35 - WPJ25;
U125 = DV25 - WPJ35;
U225 = -DV15 - DV65;
U325 = DV55 + WPJ15;
U135 = DV35 + WPJ25;
U235 = DV55 - WPJ15;
U335 = -DV15 - DV45;
VSP15 = D*U114 - RL5*U124 + VPJ14;
VSP25 = D*U214 - RL5*U224 + VPJ24;
VSP35 = D*U314 - RL5*U324 + VPJ34;
VPJ15 = -C5*VSP15 - S5*VSP35;
VPJ25 = -C5*VSP35 + S5*VSP15;
VPJ35 = -VSP25;
W16 = C6*W15 + S6*W35;
W26 = C6*W35 - S6*W15;
W36 = -W25 + qd6;
WPJ16 = C6*WPJ15 + S6*WPJ35 + W26*qd6;
WPJ26 = C6*WPJ35 - S6*WPJ15 - W16*qd6;
WPJ36 = -WPJ25;
DV16 = W16**2;
DV26 = W16*W26;
DV36 = W16*W36;
DV46 = W26**2;
DV56 = W26*W36;
DV66 = W36**2;
U116 = -DV46 - DV66;
U216 = DV26 + WPJ36;
U316 = DV36 - WPJ26;
U126 = DV26 - WPJ36;
U226 = -DV16 - DV66;
U326 = DV56 + WPJ16;
U136 = DV36 + WPJ26;
U236 = DV56 - WPJ16;
U336 = -DV16 - DV46;
VPJ16 = C6*VPJ15 + S6*VPJ35;
VPJ26 = C6*VPJ35 - S6*VPJ15;
VPJ36 = -VPJ25;
W17 = C7*W16 + S7*W36;
W27 = C7*W36 - S7*W16;
W37 = -W26 + qd7;
WPJ17 = C7*WPJ16 + S7*WPJ36 + W27*qd7;
WPJ27 = C7*WPJ36 - S7*WPJ16 - W17*qd7;
WPJ37 = -WPJ26;
DV17 = W17**2;
DV27 = W17*W27;
DV37 = W17*W37;
DV47 = W27**2;
DV57 = W27*W37;
DV67 = W37**2;
U117 = -DV47 - DV67;
U217 = DV27 + WPJ37;
U317 = DV37 - WPJ27;
U127 = DV27 - WPJ37;
U227 = -DV17 - DV67;
U327 = DV57 + WPJ17;
U137 = DV37 + WPJ27;
U237 = DV57 - WPJ17;
U337 = -DV17 - DV47;
VSP17 = D*U116 + VPJ16;
VSP27 = D*U216 + VPJ26;
VSP37 = D*U316 + VPJ36;
VPJ17 = C7*VSP17 + S7*VSP37;
VPJ27 = C7*VSP37 - S7*VSP17;
VPJ37 = -VSP27;
}

*/







// Vector of Gravity Compensation Torques
Eigen::VectorXd arm_gravity_compensation_torque(Eigen::VectorXd joint_state){
	double C1 = cos(joint_state(0));
	double S1 = sin(joint_state(0));
	double C2 = cos(joint_state(1));
	double S2 = sin(joint_state(1));
	double C3 = cos(joint_state(2));
	double S3 = sin(joint_state(2));
	double C4 = cos(joint_state(3));
	double S4 = sin(joint_state(3));
	double C5 = cos(joint_state(4));
	double S5 = sin(joint_state(4));
	double C6 = cos(joint_state(5));
	double S6 = sin(joint_state(5));
	double C7 = cos(joint_state(6));
	double S7 = sin(joint_state(6));
	
	Eigen::VectorXd gravity_compensation_torque(7);
	
	double GZ=9.81;
	
	double VP12 = GZ*S2;
	double VP22 = C2*GZ;
	double VSP13 = VP12;
	double VSP23 = VP22;
	double VP13 = C3*VSP13;
	double VP23 = -S3*VSP13;
	double VSP14 = VP13;
	double VSP24 = VP23;
	double VSP34 = VSP23;
	double VP14 = C4*VSP14 - S4*VSP34;
	double VP24 = -C4*VSP34 - S4*VSP14;
	double VSP15 = VP14;
	double VSP25 = VP24;
	double VSP35 = VSP24;
	double VP15 = -C5*VSP15 - S5*VSP35;
	double VP25 = -C5*VSP35 + S5*VSP15;
	double VP16 = C6*VP15 - S6*VSP25;
	double VP26 = -C6*VSP25 - S6*VP15;
	double VSP17 = VP16;
	double VSP27 = VP26;
	double VSP37 = -VP25;
	double VP17 = C7*VSP17 + S7*VSP37;
	double VP27 = C7*VSP37 - S7*VSP17;
	double F31 = GZ*M1;
	double F12 = M2*VP12;
	double F22 = M2*VP22;
	double F13 = M3*VP13;
	double F23 = M3*VP23;
	double F33 = M3*VSP23;
	double F14 = M4*VP14;
	double F24 = M4*VP24;
	double F34 = M4*VSP24;
	double F15 = M5*VP15;
	double F25 = M5*VP25;
	double F35 = -M5*VSP25;
	double F16 = M6*VP16;
	double F26 = M6*VP26;
	double F36 = -M6*VP25;
	double F17 = M7*VP17;
	double F27 = M7*VP27;
	double F37 = -M7*VSP27;
	double E17 = F17 + FX7;
	double E27 = F27 + FY7;
	double E37 = F37 + FZ7;
	double N17 = CX7 - VP27*Z7 - VSP27*Y7;
	double N27 = CY7 + VP17*Z7 + VSP27*X7;
	double N37 = CZ7 - VP17*Y7 + VP27*X7;
	double FDI17 = C7*E17 - E27*S7;
	double FDI37 = C7*E27 + E17*S7;
	double E16 = F16 + FDI17;
	double E26 = -E37 + F26;
	double E36 = F36 + FDI37;
	double N16 = C7*N17 - N27*S7 - VP25*Y6 - VP26*Z6;
	double N26 = -D*FDI37 - N37 + VP16*Z6 + VP25*X6;
	double N36 = C7*N27 - D*E37 + N17*S7 - VP16*Y6 + VP26*X6;
	double FDI16 = C6*E16 - E26*S6;
	double FDI36 = C6*E26 + E16*S6;
	double E15 = F15 + FDI16;
	double E25 = -E36 + F25;
	double E35 = F35 + FDI36;
	double N15 = C6*N16 - N26*S6 - VP25*Z5 - VSP25*Y5;
	double N25 = -N36 + VP15*Z5 + VSP25*X5;
	double N35 = C6*N26 + N16*S6 - VP15*Y5 + VP25*X5;
	double FDI15 = -C5*E15 + E25*S5;
	double FDI35 = -C5*E25 - E15*S5;
	double E14 = F14 + FDI15;
	double E24 = -E35 + F24;
	double E34 = F34 + FDI35;
	double N14 = -C5*N15 - FDI35*RL5 + N25*S5 - VP24*Z4 + VSP24*Y4;
	double N24 = -D*FDI35 - N35 + VP14*Z4 - VSP24*X4;
	double N34 = -C5*N25 - D*E35 + FDI15*RL5 - N15*S5 - VP14*Y4 + VP24*X4;
	double FDI14 = C4*E14 - E24*S4;
	double FDI34 = -C4*E24 - E14*S4;
	double E13 = F13 + FDI14;
	double E23 = E34 + F23;
	double E33 = F33 + FDI34;
	double N13 = C4*N14 - N24*S4 + VP23*Z3 + VSP23*Y3;
	double N23 = D*FDI34 + N34 + VP13*Z3 - VSP23*X3;
	double N33 = -C4*N24 - D*E34 - N14*S4 - VP13*Y3 + VP23*X3;
	double FDI13 = C3*E13 - E23*S3;
	double FDI33 = -C3*E23 - E13*S3;
	double E12 = F12 + FDI13;
	double E22 = E33 + F22;
	double E32 = FDI33;
	double N12 = C3*N13 + FDI33*RL3 - N23*S3 - VP22*Z2;
	double N22 = N33 + VP12*Z2;
	double N32 = -C3*N23 - FDI13*RL3 - N13*S3 - VP12*Y2 + VP22*X2;
	double FDI12 = C2*E12 - E22*S2;
	double FDI32 = C2*E22 + E12*S2;
	double E11 = FDI12;
	double E21 = -E32;
	double E31 = F31 + FDI32;
	double N11 = C2*N12 + GZ*Y1 - N22*S2;
	double N21 = -GZ*X1 - N32;
	double N31 = C2*N22 + N12*S2;
	double FDI11 = C1*E11 - E21*S1;
	double FDI21 = C1*E21 + E11*S1;
	
	gravity_compensation_torque << N31, N32, N33, N34, N35, N36, N37;
	
	return gravity_compensation_torque;
}






///////////////// this function returns the 7x7 inertia matrix
Eigen::MatrixXd arm_inertia_matrix(Eigen::VectorXd joint_state){
	double t1, t2, t3, t4, t5, t6, t7;
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	t5 = joint_state(4);
	t6 = joint_state(5);
	t7 = joint_state(6);
	
	double C1 = cos(t1);
	double S1 = sin(t1);
	double C2 = cos(t2);
	double S2 = sin(t2);
	double C3 = cos(t3);
	double S3 = sin(t3);
	double C4 = cos(t4);
	double S4 = sin(t4);
	double C5 = cos(t5);
	double S5 = sin(t5);
	double C6 = cos(t6);
	double S6 = sin(t6);
	double C7 = cos(t7);
	double S7 = sin(t7);
	double AS17 = C7*X7 - S7*Y7;
	double AS37 = C7*Y7 + S7*X7;
	double AJ117 = C7*IXX7 - IXY7*S7;
	double AJ317 = C7*IXY7 + IXX7*S7;
	double AJ127 = C7*IXY7 - IYY7*S7;
	double AJ327 = C7*IYY7 + IXY7*S7;
	double AJ137 = C7*IXZ7 - IYZ7*S7;
	double AJ337 = C7*IYZ7 + IXZ7*S7;
	double AJA117 = AJ117*C7 - AJ127*S7;
	double AJA317 = AJ317*C7 - AJ327*S7;
	double AJA137 = AJ117*S7 + AJ127*C7;
	double AJA337 = AJ317*S7 + AJ327*C7;
	double PAS217 = -D*Z7;
	double PAS317 = AS37*D;
	double PAS227 = -AS17*D;
	double JP116 = AJA117 + IXX6;
	double JP216 = -AJ137 + IXY6 - PAS217;
	double JP316 = AJA317 + IXZ6 - PAS317;
	double JP226 = D*D*M7 + IYY6 + IZZ7 - 2*PAS227;
	double JP326 = -AJ337 + IYZ6;
	double JP136 = AJA137 + IXZ6 - PAS317;
	double JP336 = AJA337 + D*D*M7 + IZZ6 - 2*PAS227;
	double MSP16 = AS17 + D*M7 + X6;
	double MSP26 = Y6 - Z7;
	double MSP36 = AS37 + Z6;
	double MP6 = M6 + M7;
	double AS16 = C6*MSP16 - MSP26*S6;
	double AS36 = C6*MSP26 + MSP16*S6;
	double AJ116 = C6*JP116 - JP216*S6;
	double AJ316 = C6*JP216 + JP116*S6;
	double AJ126 = C6*JP216 - JP226*S6;
	double AJ326 = C6*JP226 + JP216*S6;
	double AJ136 = C6*JP136 - JP326*S6;
	double AJ336 = C6*JP326 + JP136*S6;
	double AJA116 = AJ116*C6 - AJ126*S6;
	double AJA216 = -C6*JP316 + JP326*S6;
	double AJA316 = AJ316*C6 - AJ326*S6;
	double AJA136 = AJ116*S6 + AJ126*C6;
	double AJA236 = -C6*JP326 - JP316*S6;
	double AJA336 = AJ316*S6 + AJ326*C6;
	double JP115 = AJA116 + IXX5;
	double JP215 = AJA216 + IXY5;
	double JP315 = AJA316 + IXZ5;
	double JP125 = -AJ136 + IXY5;
	double JP225 = IYY5 + JP336;
	double JP325 = -AJ336 + IYZ5;
	double JP135 = AJA136 + IXZ5;
	double JP235 = AJA236 + IYZ5;
	double JP335 = AJA336 + IZZ5;
	double MSP15 = AS16 + X5;
	double MSP25 = -MSP36 + Y5;
	double MSP35 = AS36 + Z5;
	double MP5 = M5 + MP6;
	double AS15 = -C5*MSP15 + MSP25*S5;
	double AS35 = -C5*MSP25 - MSP15*S5;
	double AJ115 = -C5*JP115 + JP215*S5;
	double AJ315 = -C5*JP215 - JP115*S5;
	double AJ125 = -C5*JP125 + JP225*S5;
	double AJ325 = -C5*JP225 - JP125*S5;
	double AJ135 = -C5*JP135 + JP235*S5;
	double AJ335 = -C5*JP235 - JP135*S5;
	double AJA115 = -AJ115*C5 + AJ125*S5;
	double AJA215 = C5*JP315 - JP325*S5;
	double AJA315 = -AJ315*C5 + AJ325*S5;
	double AJA135 = -AJ115*S5 - AJ125*C5;
	double AJA235 = C5*JP325 + JP315*S5;
	double AJA335 = -AJ315*S5 - AJ325*C5;
	double PAS115 = -MSP35*RL5;
	double PAS215 = -D*MSP35;
	double PAS315 = AS35*D;
	double PAS125 = -AS15*RL5;
	double PAS225 = -AS15*D;
	double PAS325 = -AS35*RL5;
	double PAS335 = -AS15*D - MSP35*RL5;
	double JP114 = AJA115 + IXX4 + MP5*RL5*RL5 - 2*PAS115;
	double JP214 = AJA215 + D*MP5*RL5 + IXY4 - PAS125 - PAS215;
	double JP314 = AJA315 + IXZ4 - PAS315;
	double JP124 = -AJ135 + D*MP5*RL5 + IXY4 - PAS125 - PAS215;
	double JP224 = D*D*MP5 + IYY4 + JP335 - 2*PAS225;
	double JP324 = -AJ335 + IYZ4 - PAS325;
	double JP134 = AJA135 + IXZ4 - PAS315;
	double JP234 = AJA235 + IYZ4 - PAS325;
	double JP334 = AJA335 + D*D*MP5 + IZZ4 + MP5*RL5*RL5 - 2*PAS335;
	double MSP14 = AS15 + D*MP5 + X4;
	double MSP24 = -MP5*RL5 - MSP35 + Y4;
	double MSP34 = AS35 + Z4;
	double MP4 = M4 + MP5;
	double AS14 = C4*MSP14 - MSP24*S4;
	double AS34 = -C4*MSP24 - MSP14*S4;
	double AJ114 = C4*JP114 - JP214*S4;
	double AJ314 = -C4*JP214 - JP114*S4;
	double AJ124 = C4*JP124 - JP224*S4;
	double AJ324 = -C4*JP224 - JP124*S4;
	double AJ134 = C4*JP134 - JP234*S4;
	double AJ334 = -C4*JP234 - JP134*S4;
	double AJA114 = AJ114*C4 - AJ124*S4;
	double AJA214 = C4*JP314 - JP324*S4;
	double AJA314 = AJ314*C4 - AJ324*S4;
	double AJA134 = -AJ114*S4 - AJ124*C4;
	double AJA234 = -C4*JP324 - JP314*S4;
	double AJA334 = -AJ314*S4 - AJ324*C4;
	double PAS214 = -D*MSP34;
	double PAS314 = -AS34*D;
	double PAS224 = AS14*D;
	double JP113 = AJA114 + IXX3;
	double JP213 = AJA214 + IXY3 - PAS214;
	double JP313 = AJA314 + IXZ3 - PAS314;
	double JP123 = AJ134 + IXY3 - PAS214;
	double JP223 = D*D*MP4 + IYY3 + JP334 - 2*PAS224;
	double JP323 = AJ334 + IYZ3;
	double JP133 = AJA134 + IXZ3 - PAS314;
	double JP233 = AJA234 + IYZ3;
	double JP333 = AJA334 + D*D*MP4 + IZZ3 - 2*PAS224;
	double MSP13 = AS14 - D*MP4 + X3;
	double MSP23 = MSP34 + Y3;
	double MSP33 = AS34 + Z3;
	double MP3 = M3 + MP4;
	double AS13 = C3*MSP13 - MSP23*S3;
	double AS33 = -C3*MSP23 - MSP13*S3;
	double AJ113 = C3*JP113 - JP213*S3;
	double AJ313 = -C3*JP213 - JP113*S3;
	double AJ123 = C3*JP123 - JP223*S3;
	double AJ323 = -C3*JP223 - JP123*S3;
	double AJ133 = C3*JP133 - JP233*S3;
	double AJ333 = -C3*JP233 - JP133*S3;
	double AJA113 = AJ113*C3 - AJ123*S3;
	double AJA213 = C3*JP313 - JP323*S3;
	double AJA313 = AJ313*C3 - AJ323*S3;
	double AJA133 = -AJ113*S3 - AJ123*C3;
	double AJA233 = -C3*JP323 - JP313*S3;
	double AJA333 = -AJ313*S3 - AJ323*C3;
	double PAS113 = -MSP33*RL3;
	double PAS123 = AS13*RL3;
	double PAS323 = AS33*RL3;
	double JP112 = AJA113 + IXX2 + MP3*RL3*RL3 - 2*PAS113;
	double JP212 = AJA213 + IXY2 - PAS123;
	double JP312 = AJA313 + IXZ2;
	double JP122 = AJ133 + IXY2 - PAS123;
	double JP222 = IYY2 + JP333;
	double JP322 = AJ333 + IYZ2 - PAS323;
	double JP132 = AJA133 + IXZ2;
	double JP232 = AJA233 + IYZ2 - PAS323;
	double JP332 = AJA333 + IZZ2 + MP3*RL3*RL3 - 2*PAS113;
	double MSP12 = AS13 + X2;
	double MSP22 = MP3*RL3 + MSP33 + Y2;
	double MSP32 = AS33 + Z2;
	double MP2 = M2 + MP3;
	double AS12 = C2*MSP12 - MSP22*S2;
	double AS32 = C2*MSP22 + MSP12*S2;
	double AJ112 = C2*JP112 - JP212*S2;
	double AJ312 = C2*JP212 + JP112*S2;
	double AJ122 = C2*JP122 - JP222*S2;
	double AJ322 = C2*JP222 + JP122*S2;
	double AJ132 = C2*JP132 - JP232*S2;
	double AJ332 = C2*JP232 + JP132*S2;
	double AJA112 = AJ112*C2 - AJ122*S2;
	double AJA212 = -C2*JP312 + JP322*S2;
	double AJA312 = AJ312*C2 - AJ322*S2;
	double AJA132 = AJ112*S2 + AJ122*C2;
	double AJA232 = -C2*JP322 - JP312*S2;
	double AJA332 = AJ312*S2 + AJ322*C2;
	double JP111 = AJA112 + IXX1;
	double JP211 = AJA212 + IXY1;
	double JP311 = AJA312 + IXZ1;
	double JP121 = -AJ132 + IXY1;
	double JP221 = IYY1 + JP332;
	double JP321 = -AJ332 + IYZ1;
	double JP131 = AJA132 + IXZ1;
	double JP231 = AJA232 + IYZ1;
	double JP331 = AJA332 + IZZ1;
	double MSP11 = AS12 + X1;
	double MSP21 = -MSP32 + Y1;
	double MSP31 = AS32 + Z1;
	double MP1 = M1 + MP2;
	double Nd12 = AJ133 - C3*MSP13*RL3 + MSP23*RL3*S3;
	double Nd32 = AJ333 + C3*MSP23*RL3 + MSP13*RL3*S3;
	double Ed11 = AS33*C2;
	double Ed31 = AS33*S2;
	double Nd11 = C2*Nd12 - JP333*S2;
	double Nd31 = C2*JP333 + Nd12*S2;
	double Ne23 = -C4*D*MSP14 + D*MSP24*S4 + JP334;
	double Ee12 = AS34*C3;
	double Ee32 = -AS34*S3;
	double Ne12 = AJ134*C3 - AS34*RL3*S3 - Ne23*S3;
	double Ne32 = -AJ134*S3 - AS34*C3*RL3 - C3*Ne23;
	double Ee11 = AS14*S2 + C2*Ee12;
	double Ee31 = -AS14*C2 + Ee12*S2;
	double Ne11 = -AJ334*S2 + C2*Ne12;
	double Ne31 = AJ334*C2 + Ne12*S2;
	double Nf14 = AJ135 + C5*MSP15*RL5 - MSP25*RL5*S5;
	double Nf24 = C5*D*MSP15 - D*MSP25*S5 - JP335;
	double Nf34 = AJ335 + C5*MSP25*RL5 + MSP15*RL5*S5;
	double Ef13 = -AS35*C4;
	double Ef33 = AS35*S4;
	double Nf13 = C4*Nf14 - Nf24*S4;
	double Nf23 = AS35*D*S4 + Nf34;
	double Nf33 = -AS15*D - C4*Nf24 - Nf14*S4;
	double Ef12 = -AS15*S3 + C3*Ef13;
	double Ef32 = -AS15*C3 - Ef13*S3;
	double Nf12 = -AS15*C3*RL3 + C3*Nf13 - Ef13*RL3*S3 - Nf23*S3;
	double Nf32 = AS15*RL3*S3 - C3*Ef13*RL3 - C3*Nf23 - Nf13*S3;
	double Ef11 = C2*Ef12 - Ef33*S2;
	double Ef31 = C2*Ef33 + Ef12*S2;
	double Nf11 = C2*Nf12 - Nf33*S2;
	double Nf31 = C2*Nf33 + Nf12*S2;
	double Eg14 = AS36*C5;
	double Eg34 = AS36*S5;
	double Ng14 = -AJ136*C5 - AS36*RL5*S5 - JP336*S5;
	double Ng24 = -AJ336 - AS36*D*S5;
	double Ng34 = -AJ136*S5 - AS16*D + AS36*C5*RL5 + C5*JP336;
	double Eg13 = AS16*S4 + C4*Eg14;
	double Eg33 = AS16*C4 - Eg14*S4;
	double Ng13 = C4*Ng14 - Ng24*S4;
	double Ng23 = AS16*C4*D - D*Eg14*S4 + Ng34;
	double Ng33 = -C4*Ng24 - D*Eg34 - Ng14*S4;
	double Eg12 = C3*Eg13 - Eg34*S3;
	double Eg32 = -C3*Eg34 - Eg13*S3;
	double Ng12 = -C3*Eg34*RL3 + C3*Ng13 - Eg13*RL3*S3 - Ng23*S3;
	double Ng32 = -C3*Eg13*RL3 - C3*Ng23 + Eg34*RL3*S3 - Ng13*S3;
	double Eg11 = C2*Eg12 - Eg33*S2;
	double Eg31 = C2*Eg33 + Eg12*S2;
	double Ng11 = C2*Ng12 - Ng33*S2;
	double Ng31 = C2*Ng33 + Ng12*S2;
	double Nh26 = -C7*D*X7 + D*S7*Y7 - IZZ7;
	double Eh15 = -AS37*C6;
	double Eh35 = -AS37*S6;
	double Nh15 = AJ137*C6 - Nh26*S6;
	double Nh35 = AJ137*S6 + C6*Nh26;
	double Eh14 = -AS17*S5 - C5*Eh15;
	double Eh34 = AS17*C5 - Eh15*S5;
	double Nh14 = -AJ337*S5 - AS17*C5*RL5 - C5*Nh15 + Eh15*RL5*S5;
	double Nh24 = -AS17*C5*D + D*Eh15*S5 - Nh35;
	double Nh34 = AJ337*C5 - AS17*RL5*S5 - C5*Eh15*RL5 - D*Eh35 - Nh15*S5;
	double Eh13 = C4*Eh14 + Eh35*S4;
	double Eh33 = C4*Eh35 - Eh14*S4;
	double Nh13 = C4*Nh14 - Nh24*S4;
	double Nh23 = C4*D*Eh35 - D*Eh14*S4 + Nh34;
	double Nh33 = -C4*Nh24 - D*Eh34 - Nh14*S4;
	double Eh12 = C3*Eh13 - Eh34*S3;
	double Eh32 = -C3*Eh34 - Eh13*S3;
	double Nh12 = -C3*Eh34*RL3 + C3*Nh13 - Eh13*RL3*S3 - Nh23*S3;
	double Nh32 = -C3*Eh13*RL3 - C3*Nh23 + Eh34*RL3*S3 - Nh13*S3;
	double Eh11 = C2*Eh12 - Eh33*S2;
	double Eh31 = C2*Eh33 + Eh12*S2;
	double Nh11 = C2*Nh12 - Nh33*S2;
	double Nh31 = C2*Nh33 + Nh12*S2;
	double A11 = Ia1 + JP331;
	double A21 = AJ332;
	double A31 = Nd31;
	double A41 = Ne31;
	double A51 = Nf31;
	double A61 = Ng31;
	double A71 = Nh31;
	double A22 = Ia2 + JP332;
	double A32 = Nd32;
	double A42 = Ne32;
	double A52 = Nf32;
	double A62 = Ng32;
	double A72 = Nh32;
	double A33 = Ia3 + JP333;
	double A43 = AJ334;
	double A53 = Nf33;
	double A63 = Ng33;
	double A73 = Nh33;
	double A44 = Ia4 + JP334;
	double A54 = Nf34;
	double A64 = Ng34;
	double A74 = Nh34;
	double A55 = Ia5 + JP335;
	double A65 = AJ336;
	double A75 = Nh35;
	double A66 = Ia6 + JP336;
	double A76 = AJ337;
	double A77 = IZZ7 + Ia7;
	
	Eigen::MatrixXd inertia_matrix(7,7);
	
	// this gives the jacobian matrix of the arm frame7 w.r.t arm frame0 
	inertia_matrix << A11, A21, A31, A41, A51, A61, A71,
										A21, A22, A32, A42, A52, A62, A72,
										A31, A32, A33, A43, A53, A63, A73,
										A41, A42, A43, A44, A54, A64, A74,
										A51, A52, A53, A54, A55, A65, A75,
										A61, A62, A63, A64, A65, A66, A76,
										A71, A72, A73, A74, A75, A76, A77;
	return inertia_matrix;
}



/////////////// this function returns the vector of viscous friction torques
Eigen::VectorXd arm_viscous_friction_torque(Eigen::VectorXd joint_velocity){
	Eigen::MatrixXd viscous_friction_coefficient_matrix(7,7);
	viscous_friction_coefficient_matrix << Fv1, 0, 0, 0, 0, 0, 0,
																				 0, Fv2, 0, 0, 0, 0, 0,
																				 0, 0, Fv3, 0, 0, 0, 0,
																				 0, 0, 0, Fv4, 0, 0, 0,
																				 0, 0, 0, 0, Fv5, 0, 0,
																				 0, 0, 0, 0, 0, Fv6, 0,
																				 0, 0, 0, 0, 0, 0, Fv7;
	return viscous_friction_coefficient_matrix*joint_velocity;
}




/////////////// this function returns the vector of static friction torques
Eigen::VectorXd arm_static_friction_torque(Eigen::VectorXd joint_velocity){
	Eigen::VectorXd joint_velocity_sign(joint_velocity.size());
	for(int k=0;k<joint_velocity.size();k++){
		if(joint_velocity(k)>=0)
			joint_velocity_sign(k) = 1;
		else
			joint_velocity_sign(k) = 0;
	}
	Eigen::MatrixXd static_friction_coefficient_matrix(7,7);
	static_friction_coefficient_matrix << Fs1, 0, 0, 0, 0, 0, 0,
																				 0, Fs2, 0, 0, 0, 0, 0,
																				 0, 0, Fs3, 0, 0, 0, 0,
																				 0, 0, 0, Fs4, 0, 0, 0,
																				 0, 0, 0, 0, Fs5, 0, 0,
																				 0, 0, 0, 0, 0, Fs6, 0,
																				 0, 0, 0, 0, 0, 0, Fs7;
	return static_friction_coefficient_matrix*joint_velocity_sign;
}








Eigen::VectorXd arm_coriolis_centrifugal_torque(Eigen::VectorXd joint_state, Eigen::VectorXd joint_velocity){

	double t1, t2, t3, t4, t5, t6, t7;
	double qd1, qd2, qd3, qd4, qd5, qd6, qd7;
	
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	t5 = joint_state(4);
	t6 = joint_state(5);
	t7 = joint_state(6);
	
	qd1 = joint_velocity(0);
	qd2 = joint_velocity(1);
	qd3 = joint_velocity(2);
	qd4 = joint_velocity(3);
	qd5 = joint_velocity(4);
	qd6 = joint_velocity(5);
	qd7 = joint_velocity(6);
	
	double G=0.0;
	
	double C1 = cos(t1);
	double S1 = sin(t1);
	double C2 = cos(t2);
	double S2 = sin(t2);
	double C3 = cos(t3);
	double S3 = sin(t3);
	double C4 = cos(t4);
	double S4 = sin(t4);
	double C5 = cos(t5);
	double S5 = sin(t5);
	double C6 = cos(t6);
	double S6 = sin(t6);
	double C7 = cos(t7);
	double S7 = sin(t7);
	double DV61 = qd1*qd1;
	double W12 = S2*qd1;
	double W22 = C2*qd1;
	double WP12 = W22*qd2;
	double WP22 = -W12*qd2;
	double DV12 = W12*W12;
	double DV22 = W12*W22;
	double DV42 = W22*W22;
	double DV62 = qd2*qd2;
	double U112 = -DV42 - DV62;
	double U312 = -2*WP22;
	double U222 = -DV12 - DV62;
	double U322 = 2*WP12;
	double U332 = -DV12 - DV42;
	double VP12 = -G*S2;
	double VP22 = -C2*G;
	double W13 = C3*W12 - S3*qd2;
	double W23 = -C3*qd2 - S3*W12;
	double W33 = W22 + qd3;
	double WP13 = C3*WP12 + W23*qd3;
	double WP23 = -S3*WP12 - W13*qd3;
	double DV13 = W13*W13;
	double DV23 = W13*W23;
	double DV33 = W13*W33;
	double DV43 = W23*W23;
	double DV53 = W23*W33;
	double DV63 = W33*W33;
	double U113 = -DV43 - DV63;
	double U213 = DV23 + WP22;
	double U313 = DV33 - WP23;
	double U123 = DV23 - WP22;
	double U223 = -DV13 - DV63;
	double U323 = DV53 + WP13;
	double U133 = DV33 + WP23;
	double U233 = DV53 - WP13;
	double U333 = -DV13 - DV43;
	double VSP13 = DV22*RL3 + VP12;
	double VSP23 = RL3*U222 + VP22;
	double VSP33 = RL3*U322;
	double VP13 = C3*VSP13 - S3*VSP33;
	double VP23 = -C3*VSP33 - S3*VSP13;
	double W14 = C4*W13 - S4*W33;
	double W24 = -C4*W33 - S4*W13;
	double W34 = W23 + qd4;
	double WP14 = C4*WP13 - S4*WP22 + W24*qd4;
	double WP24 = -C4*WP22 - S4*WP13 - W14*qd4;
	double DV14 = W14*W14;
	double DV24 = W14*W24;
	double DV34 = W14*W34;
	double DV44 = W24*W24;
	double DV54 = W24*W34;
	double DV64 = W34*W34;
	double U114 = -DV44 - DV64;
	double U214 = DV24 + WP23;
	double U314 = DV34 - WP24;
	double U124 = DV24 - WP23;
	double U224 = -DV14 - DV64;
	double U324 = DV54 + WP14;
	double U134 = DV34 + WP24;
	double U234 = DV54 - WP14;
	double U334 = -DV14 - DV44;
	double VSP14 = -D*U113 + VP13;
	double VSP24 = -D*U213 + VP23;
	double VSP34 = -D*U313 + VSP23;
	double VP14 = C4*VSP14 - S4*VSP34;
	double VP24 = -C4*VSP34 - S4*VSP14;
	double W15 = -C5*W14 - S5*W34;
	double W25 = -C5*W34 + S5*W14;
	double W35 = -W24 + qd5;
	double WP15 = -C5*WP14 - S5*WP23 + W25*qd5;
	double WP25 = -C5*WP23 + S5*WP14 - W15*qd5;
	double DV15 = W15*W15;
	double DV25 = W15*W25;
	double DV35 = W15*W35;
	double DV45 = W25*W25;
	double DV55 = W25*W35;
	double DV65 = W35*W35;
	double U115 = -DV45 - DV65;
	double U215 = DV25 - WP24;
	double U315 = DV35 - WP25;
	double U125 = DV25 + WP24;
	double U225 = -DV15 - DV65;
	double U325 = DV55 + WP15;
	double U135 = DV35 + WP25;
	double U235 = DV55 - WP15;
	double U335 = -DV15 - DV45;
	double VSP15 = D*U114 - RL5*U124 + VP14;
	double VSP25 = D*U214 - RL5*U224 + VP24;
	double VSP35 = D*U314 - RL5*U324 + VSP24;
	double VP15 = -C5*VSP15 - S5*VSP35;
	double VP25 = -C5*VSP35 + S5*VSP15;
	double W16 = C6*W15 + S6*W35;
	double W26 = C6*W35 - S6*W15;
	double W36 = -W25 + qd6;
	double WP16 = C6*WP15 - S6*WP24 + W26*qd6;
	double WP26 = -C6*WP24 - S6*WP15 - W16*qd6;
	double DV16 = W16*W16;
	double DV26 = W16*W26;
	double DV36 = W16*W36;
	double DV46 = W26*W26;
	double DV56 = W26*W36;
	double DV66 = W36*W36;
	double U116 = -DV46 - DV66;
	double U216 = DV26 - WP25;
	double U316 = DV36 - WP26;
	double U126 = DV26 + WP25;
	double U226 = -DV16 - DV66;
	double U326 = DV56 + WP16;
	double U136 = DV36 + WP26;
	double U236 = DV56 - WP16;
	double U336 = -DV16 - DV46;
	double VP16 = C6*VP15 - S6*VSP25;
	double VP26 = -C6*VSP25 - S6*VP15;
	double W17 = C7*W16 + S7*W36;
	double W27 = C7*W36 - S7*W16;
	double W37 = -W26 + qd7;
	double WP17 = C7*WP16 - S7*WP25 + W27*qd7;
	double WP27 = -C7*WP25 - S7*WP16 - W17*qd7;
	double DV17 = W17*W17;
	double DV27 = W17*W27;
	double DV37 = W17*W37;
	double DV47 = W27*W27;
	double DV57 = W27*W37;
	double DV67 = W37*W37;
	double U117 = -DV47 - DV67;
	double U217 = DV27 - WP26;
	double U317 = DV37 - WP27;
	double U127 = DV27 + WP26;
	double U227 = -DV17 - DV67;
	double U327 = DV57 + WP17;
	double U137 = DV37 + WP27;
	double U237 = DV57 - WP17;
	double U337 = -DV17 - DV47;
	double VSP17 = D*U116 + VP16;
	double VSP27 = D*U216 + VP26;
	double VSP37 = D*U316 - VP25;
	double VP17 = C7*VSP17 + S7*VSP37;
	double VP27 = C7*VSP37 - S7*VSP17;
	double F11 = -DV61*X1;
	double F21 = -DV61*Y1;
	double F31 = -G*M1;
	double PSI11 = IXZ1*qd1;
	double PSI21 = IYZ1*qd1;
	double PSI31 = IZZ1*qd1;
	double No11 = -PSI21*qd1;
	double No21 = PSI11*qd1;
	double F12 = DV22*Y2 + M2*VP12 + U112*X2;
	double F22 = DV22*X2 + M2*VP22 + U222*Y2;
	double F32 = U312*X2 + U322*Y2 + U332*Z2;
	double PSI12 = IXX2*W12 + IXY2*W22 + IXZ2*qd2;
	double PSI22 = IXY2*W12 + IYY2*W22 + IYZ2*qd2;
	double PSI32 = IXZ2*W12 + IYZ2*W22 + IZZ2*qd2;
	double No12 = IXX2*WP12 + IXY2*WP22 - PSI22*qd2 + PSI32*W22;
	double No22 = IXY2*WP12 + IYY2*WP22 + PSI12*qd2 - PSI32*W12;
	double No32 = IXZ2*WP12 + IYZ2*WP22 - PSI12*W22 + PSI22*W12;
	double F13 = M3*VP13 + U113*X3 + U123*Y3 + U133*Z3;
	double F23 = M3*VP23 + U213*X3 + U223*Y3 + U233*Z3;
	double F33 = M3*VSP23 + U313*X3 + U323*Y3 + U333*Z3;
	double PSI13 = IXX3*W13 + IXY3*W23 + IXZ3*W33;
	double PSI23 = IXY3*W13 + IYY3*W23 + IYZ3*W33;
	double PSI33 = IXZ3*W13 + IYZ3*W23 + IZZ3*W33;
	double No13 = IXX3*WP13 + IXY3*WP23 + IXZ3*WP22 - PSI23*W33 + PSI33*W23;
	double No23 = IXY3*WP13 + IYY3*WP23 + IYZ3*WP22 + PSI13*W33 - PSI33*W13;
	double No33 = IXZ3*WP13 + IYZ3*WP23 + IZZ3*WP22 - PSI13*W23 + PSI23*W13;
	double F14 = M4*VP14 + U114*X4 + U124*Y4 + U134*Z4;
	double F24 = M4*VP24 + U214*X4 + U224*Y4 + U234*Z4;
	double F34 = M4*VSP24 + U314*X4 + U324*Y4 + U334*Z4;
	double PSI14 = IXX4*W14 + IXY4*W24 + IXZ4*W34;
	double PSI24 = IXY4*W14 + IYY4*W24 + IYZ4*W34;
	double PSI34 = IXZ4*W14 + IYZ4*W24 + IZZ4*W34;
	double No14 = IXX4*WP14 + IXY4*WP24 + IXZ4*WP23 - PSI24*W34 + PSI34*W24;
	double No24 = IXY4*WP14 + IYY4*WP24 + IYZ4*WP23 + PSI14*W34 - PSI34*W14;
	double No34 = IXZ4*WP14 + IYZ4*WP24 + IZZ4*WP23 - PSI14*W24 + PSI24*W14;
	double F15 = M5*VP15 + U115*X5 + U125*Y5 + U135*Z5;
	double F25 = M5*VP25 + U215*X5 + U225*Y5 + U235*Z5;
	double F35 = -M5*VSP25 + U315*X5 + U325*Y5 + U335*Z5;
	double PSI15 = IXX5*W15 + IXY5*W25 + IXZ5*W35;
	double PSI25 = IXY5*W15 + IYY5*W25 + IYZ5*W35;
	double PSI35 = IXZ5*W15 + IYZ5*W25 + IZZ5*W35;
	double No15 = IXX5*WP15 + IXY5*WP25 - IXZ5*WP24 - PSI25*W35 + PSI35*W25;
	double No25 = IXY5*WP15 + IYY5*WP25 - IYZ5*WP24 + PSI15*W35 - PSI35*W15;
	double No35 = IXZ5*WP15 + IYZ5*WP25 - IZZ5*WP24 - PSI15*W25 + PSI25*W15;
	double F16 = M6*VP16 + U116*X6 + U126*Y6 + U136*Z6;
	double F26 = M6*VP26 + U216*X6 + U226*Y6 + U236*Z6;
	double F36 = -M6*VP25 + U316*X6 + U326*Y6 + U336*Z6;
	double PSI16 = IXX6*W16 + IXY6*W26 + IXZ6*W36;
	double PSI26 = IXY6*W16 + IYY6*W26 + IYZ6*W36;
	double PSI36 = IXZ6*W16 + IYZ6*W26 + IZZ6*W36;
	double No16 = IXX6*WP16 + IXY6*WP26 - IXZ6*WP25 - PSI26*W36 + PSI36*W26;
	double No26 = IXY6*WP16 + IYY6*WP26 - IYZ6*WP25 + PSI16*W36 - PSI36*W16;
	double No36 = IXZ6*WP16 + IYZ6*WP26 - IZZ6*WP25 - PSI16*W26 + PSI26*W16;
	double F17 = M7*VP17 + U117*X7 + U127*Y7 + U137*Z7;
	double F27 = M7*VP27 + U217*X7 + U227*Y7 + U237*Z7;
	double F37 = -M7*VSP27 + U317*X7 + U327*Y7 + U337*Z7;
	double PSI17 = IXX7*W17 + IXY7*W27 + IXZ7*W37;
	double PSI27 = IXY7*W17 + IYY7*W27 + IYZ7*W37;
	double PSI37 = IXZ7*W17 + IYZ7*W27 + IZZ7*W37;
	double No17 = IXX7*WP17 + IXY7*WP27 - IXZ7*WP26 - PSI27*W37 + PSI37*W27;
	double No27 = IXY7*WP17 + IYY7*WP27 - IYZ7*WP26 + PSI17*W37 - PSI37*W17;
	double No37 = IXZ7*WP17 + IYZ7*WP27 - IZZ7*WP26 - PSI17*W27 + PSI27*W17;
	double E17 = F17 + FX7;
	double E27 = F27 + FY7;
	double E37 = F37 + FZ7;
	double N17 = CX7 + No17 - VP27*Z7 - VSP27*Y7;
	double N27 = CY7 + No27 + VP17*Z7 + VSP27*X7;
	double N37 = CZ7 + No37 - VP17*Y7 + VP27*X7;
	double FDI17 = C7*E17 - E27*S7;
	double FDI37 = C7*E27 + E17*S7;
	double E16 = F16 + FDI17;
	double E26 = -E37 + F26;
	double E36 = F36 + FDI37;
	double N16 = C7*N17 - N27*S7 + No16 - VP25*Y6 - VP26*Z6;
	double N26 = -D*FDI37 - N37 + No26 + VP16*Z6 + VP25*X6;
	double N36 = C7*N27 - D*E37 + N17*S7 + No36 - VP16*Y6 + VP26*X6;
	double FDI16 = C6*E16 - E26*S6;
	double FDI36 = C6*E26 + E16*S6;
	double E15 = F15 + FDI16;
	double E25 = -E36 + F25;
	double E35 = F35 + FDI36;
	double N15 = C6*N16 - N26*S6 + No15 - VP25*Z5 - VSP25*Y5;
	double N25 = -N36 + No25 + VP15*Z5 + VSP25*X5;
	double N35 = C6*N26 + N16*S6 + No35 - VP15*Y5 + VP25*X5;
	double FDI15 = -C5*E15 + E25*S5;
	double FDI35 = -C5*E25 - E15*S5;
	double E14 = F14 + FDI15;
	double E24 = -E35 + F24;
	double E34 = F34 + FDI35;
	double N14 = -C5*N15 - FDI35*RL5 + N25*S5 + No14 - VP24*Z4 + VSP24*Y4;
	double N24 = -D*FDI35 - N35 + No24 + VP14*Z4 - VSP24*X4;
	double N34 = -C5*N25 - D*E35 + FDI15*RL5 - N15*S5 + No34 - VP14*Y4 + VP24*X4;
	double FDI14 = C4*E14 - E24*S4;
	double FDI34 = -C4*E24 - E14*S4;
	double E13 = F13 + FDI14;
	double E23 = E34 + F23;
	double E33 = F33 + FDI34;
	double N13 = C4*N14 - N24*S4 + No13 - VP23*Z3 + VSP23*Y3;
	double N23 = D*FDI34 + N34 + No23 + VP13*Z3 - VSP23*X3;
	double N33 = -C4*N24 - D*E34 - N14*S4 + No33 - VP13*Y3 + VP23*X3;
	double FDI13 = C3*E13 - E23*S3;
	double FDI33 = -C3*E23 - E13*S3;
	double E12 = F12 + FDI13;
	double E22 = E33 + F22;
	double E32 = F32 + FDI33;
	double N12 = C3*N13 + FDI33*RL3 - N23*S3 + No12 - VP22*Z2;
	double N22 = N33 + No22 + VP12*Z2;
	double N32 = -C3*N23 - FDI13*RL3 - N13*S3 + No32 - VP12*Y2 + VP22*X2;
	double FDI12 = C2*E12 - E22*S2;
	double FDI32 = C2*E22 + E12*S2;
	double E11 = F11 + FDI12;
	double E21 = -E32 + F21;
	double E31 = F31 + FDI32;
	double N11 = C2*N12 - G*Y1 - N22*S2 + No11;
	double N21 = G*X1 - N32 + No21;
	double N31 = C2*N22 + N12*S2;
	
	Eigen::VectorXd coriolis_centrifugal_torque(7);
	coriolis_centrifugal_torque << N31, N32, N33, N34, N35, N36, N37;
	return coriolis_centrifugal_torque;
}

















// franka panda arm constant transformation matrices
Eigen::Matrix4d base_to_arm_frame0(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R;
	Eigen::Vector3d P;
	R = Rotz(pi);
	P	 << 0.0, 0.0, db;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Eigen::Matrix4d arm_frame7_to_tip(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d P;
	P	 << 0.0, 0.0, de;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Eigen::Matrix4d arm_tip_to_panda_gripper(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R;
	Eigen::Vector3d rpy;
	rpy << 0,0,-pi/4;
	R = rpy_to_direction_cosines(rpy);
	Eigen::Vector3d P;
	P	 << 0.0, 0.0, 0.05;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}























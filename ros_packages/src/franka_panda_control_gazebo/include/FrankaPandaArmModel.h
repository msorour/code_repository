#include"Eigen/Dense"
#include <iostream>
#include <fstream>

#define D   0.088
#define RL3 0.316
#define RL5 0.384

#define pi  3.14159

using namespace std;
using namespace Eigen;
using namespace franka_panda_gazebo_controller;

Matrix3d Rotx(double t);
Matrix3d Roty(double t);
Matrix3d Rotz(double t);
Matrix4d base_to_arm_frame0(void);
Matrix4d arm_frame7_to_tip(void);



Matrix4d arm_direct_geometric_model(VectorXd joint_state){
	double t1, t2, t3, t4, t5, t6, t7;
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	t5 = joint_state(4);
	t6 = joint_state(5);
	t7 = joint_state(6);
	double T0T711,T0T721,T0T731, T0T712,T0T722,T0T732, T0T713,T0T723,T0T733, T0T714,T0T724,T0T734;
	
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
	
	Matrix4d DGM;
	DGM << T0T711, T0T712, T0T713, T0T714,
				 T0T721, T0T722, T0T723, T0T724,
				 T0T731, T0T732, T0T733, T0T734,
				 			0, 			0, 			0, 		  1;
	return base_to_arm_frame0()*DGM*arm_frame7_to_tip();
}









// thumb finger constant transformation matrices
Matrix4d base_to_arm_frame0(void){
	Matrix4d TM;
	Matrix3d R;
	Vector3d P;
	R = Rotz(pi);
	P	 << 0.0, 0.0, 333.0/1000;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Matrix4d arm_frame7_to_tip(void){
	Matrix4d TM;
	Matrix3d R = Matrix3d::Identity();
	Vector3d P;
	P	 << 0.0, 0.0, 107.0/1000;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}









// Useful Implementations
Matrix3d Rotx(double t){
	Matrix3d R;
	R << 1, 		 0,			 	0,
			 0, cos(t), -sin(t),
			 0, sin(t),  cos(t);
	return R;
}
Matrix3d Roty(double t){
	Matrix3d R;
	R <<  cos(t), 0,	sin(t),
			 			 0, 1, 			 0,
			 -sin(t), 0,  cos(t);
	return R;
}
Matrix3d Rotz(double t){
	Matrix3d R;
	R << cos(t), -sin(t), 0,
			 sin(t),  cos(t), 0,
			  	 	0, 			 0, 1;
	return R;
}






// This function calculates the Motion Profile online. It takes the initial and final conditions together with the current time instant (tn) and returns a 3D vector of Position, Velocity and Acceleration commands.
Vector3d OnlineMP_L5B( double ti, double tf, double tn, double Pi, double Pf ){
  Vector3d MPM;
  
  double t1, t2, P1, P2, V1, V2, A1, A2;
  double Vi, Vf, Ai, Af;
  double T, diff, a0, a1, a2, a3, a4, a5;

  // CONSTANTS NEEDED
  t1 = (tf-ti)*0.1 + ti;
  t2 = tf - (tf-ti)*0.1;
  
  P1 = (Pf-Pi)*0.05 + Pi;
  P2 = Pf - (Pf-Pi)*0.05;
  
  //t1 = (tf-ti)*0.3 + ti;
  //t2 = tf - (tf-ti)*0.3;

  //P1 = (Pf-Pi)*0.15 + Pi;
  //P2 = Pf - (Pf-Pi)*0.15;

  Vi = 0;
  V1 = (P2 - P1)/(t2 - t1);
  V2 = V1;
  Vf = 0;

  Ai = 0;
  A1 = 0;
  A2 = 0;
  Af = 0;

  
  
  
  // GENERATING THE CURRENT MOTION PROFILE VECTOR   
  
  // Interval [ti,t1[ : 5th order motion profile (accelerating)
  // ---------------------------------------------------------
  if (tn >= ti && tn < t1)
  {
    T = t1 - ti;
    diff = P1 - Pi;

    a0 = Pi;
    a1 = Vi;
    a2 = 0.5*Ai;
    a3 = (0.5/(T*T*T))*( 20*diff - ( 8*V1 + 12*Vi )*T - ( 3*Ai - A1 )*(T*T) );
    a4 = (0.5/(T*T*T*T))*( -30*diff + ( 14*V1 + 16*Vi )*T + ( 3*Ai - 2*A1 )*(T*T) );
    a5 = (0.5/(T*T*T*T*T))*( 12*diff - 6*( V1 + Vi )*T + ( A1 - Ai )*(T*T) );

    // DISPLACEMENT PROFILE
    MPM(0) = a0 + a1*(tn-ti) + a2*((tn-ti)*(tn-ti)) + a3*((tn-ti)*(tn-ti)*(tn-ti)) + a4*((tn-ti)*(tn-ti)*(tn-ti)*(tn-ti)) + a5*((tn-ti)*(tn-ti)*(tn-ti)*(tn-ti)*(tn-ti));   
      
    // VELOCITY PROFILE
    MPM(1) = a1 + 2*a2*(tn-ti) + 3*a3*((tn-ti)*(tn-ti)) + 4*a4*((tn-ti)*(tn-ti)*(tn-ti)) + 5*a5*((tn-ti)*(tn-ti)*(tn-ti)*(tn-ti));

    // ACCELERATION PROFILE
    MPM(2) = 2*a2 + 6*a3*(tn-ti) + 12*a4*((tn-ti)*(tn-ti)) + 20*a5*((tn-ti)*(tn-ti)*(tn-ti));
  }
    
  
  // Interval [t1,t2] : 1st order motion profile (constant velocity)
  // ---------------------------------------------------------------
  else if (t1 <= tn && tn <= t2)
  {
    T = t2 - t1;
    diff = P2 - P1;

    a0 = P1;
    a1 = diff/T;

    // DISPLACEMENT PROFILE
    MPM(0) = a0 + a1*(tn-t1) ;

    // VELOCITY PROFILE
    MPM(1) = a1;

    // ACCELERATION PROFILE
    MPM(2) = 0;
  }
    
    
  // Interval ]t2,tf] : 5th order motion profile (decelerating)
  // ----------------------------------------------------------
  else if (tn > t2 && tn <= tf)
  {
    T = tf - t2;
    diff = Pf - P2;

    a0 = P2;
    a1 = V2;
    a2 = 0.5*A2;
    a3 = (0.5/(T*T*T))*( 20*diff - ( 8*Vf + 12*V2 )*T - ( 3*A2 - Af )*(T*T) );
    a4 = (0.5/(T*T*T*T))*( -30*diff + ( 14*Vf + 16*V2 )*T + ( 3*A2 - 2*Af )*(T*T) );
    a5 = (0.5/(T*T*T*T*T))*( 12*diff - 6*( Vf + V2 )*T + ( A2 - Af )*(T*T) );

    // DISPLACEMENT PROFILE
    MPM(0) = a0 + a1*(tn-t2) + a2*((tn-t2)*(tn-t2)) + a3*((tn-t2)*(tn-t2)*(tn-t2)) + a4*((tn-t2)*(tn-t2)*(tn-t2)*(tn-t2)) + a5*((tn-t2)*(tn-t2)*(tn-t2)*(tn-t2)*(tn-t2));   

    // VELOCITY PROFILE
    MPM(1) = a1 + 2*a2*(tn-t2) + 3*a3*((tn-t2)*(tn-t2)) + 4*a4*((tn-t2)*(tn-t2)*(tn-t2)) + 5*a5*((tn-t2)*(tn-t2)*(tn-t2)*(tn-t2));

    // ACCELERATION PROFILE
    MPM(2) = 2*a2 + 6*a3*(tn-t2) + 12*a4*((tn-t2)*(tn-t2)) + 20*a5*((tn-t2)*(tn-t2)*(tn-t2));
  }
  
  
  else if(tn > tf)
  {
    // DISPLACEMENT PROFILE
    MPM(0) = Pf;   

    // VELOCITY PROFILE
    MPM(1) = 0;

    // ACCELERATION PROFILE
    MPM(2) = 0;
  }
  
  return MPM;

}






void log_data(void){
	joint_position_command_log << time_now << " " << joint_position_traj.transpose() <<  endl;
	joint_position_response_log << time_now << " " << joint_position.transpose() <<  endl;
	joint_position_error_log << time_now << " " << joint_position_error.transpose() <<  endl;
}

void open_logs(){
	joint_position_command_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/joint_position_command_log.txt");
	joint_position_response_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/joint_position_response_log.txt");
	joint_position_error_log.open("/home/work/code_repository/ros_packages/src/franka_panda_control_gazebo/logs/joint_position_error_log.txt");
}
void close_logs(void){
	joint_position_command_log.close();
	joint_position_response_log.close();
	joint_position_error_log.close();
}







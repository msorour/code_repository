#include "allegro_hand_parameters.h"

Eigen::Matrix4d palm_to_thumb_frame0(void);
Eigen::Matrix4d thumb_frame4_to_tip(void);
Eigen::Matrix4d palm_to_index_frame0(void);
Eigen::Matrix4d index_frame4_to_tip(void);
Eigen::Matrix4d palm_to_middle_frame0(void);
Eigen::Matrix4d middle_frame4_to_tip(void);
Eigen::Matrix4d palm_to_pinky_frame0(void);
Eigen::Matrix4d pinky_frame4_to_tip(void);
Eigen::Matrix4d finger_direct_geometric_model(std::string finger_name, Eigen::Vector4d joint_state);
Eigen::MatrixXd finger_geometric_jacobian_matrix(std::string finger_name, Eigen::Vector4d joint_state);
Eigen::Matrix4d finger_task_jacobian(std::string finger_name, Eigen::Vector4d joint_state);


Eigen::Matrix4d finger_direct_geometric_model(std::string finger_name, Eigen::Vector4d joint_state){
	
	double T0T411,T0T421,T0T431, T0T412,T0T422,T0T432, T0T413,T0T423,T0T433, T0T414,T0T424,T0T434;
	
	if(finger_name=="thumb"){
	  double t1t, t2t, t3t, t4t;
	  t1t = joint_state(0);
	  t2t = joint_state(1);
	  t3t = joint_state(2);
	  t4t = joint_state(3);
	  
	  T0T411 = sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t);
    T0T421 = sin(t1t)*sin(t2t)*sin(t3t + t4t) - cos(t1t)*cos(t3t + t4t);
    T0T431 = -sin(t3t + t4t)*cos(t2t);
    
    T0T412 = -sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t);
    T0T422 = sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t);
    T0T432 = -cos(t2t)*cos(t3t + t4t);
    
    T0T413 = cos(t1t)*cos(t2t);
    T0T423 = sin(t1t)*cos(t2t);
    T0T433 = sin(t2t);
    
    T0T414 = D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t);
    T0T424 = D2t*sin(t1t) + D4t*(sin(t1t)*sin(t2t)*sin(t3t) - cos(t1t)*cos(t3t)) - RL2t*cos(t1t);
    T0T434 = -D4t*sin(t3t)*cos(t2t);
    
	  /*
		T0T411 = sin(t1)*cos(t3 + t4) + sin(t2)*sin(t3 + t4)*cos(t1);
		T0T421 = sin(t1)*sin(t2)*sin(t3 + t4) - cos(t1)*cos(t3 + t4);
		T0T431 = -sin(t3 + t4)*cos(t2);
		
		T0T412 = -sin(t1)*sin(t3 + t4) + sin(t2)*cos(t1)*cos(t3 + t4);
		T0T422 = sin(t1)*sin(t2)*cos(t3 + t4) + sin(t3 + t4)*cos(t1);
		T0T432 = -cos(t2)*cos(t3 + t4);
		
		T0T413 = cos(t1)*cos(t2);
		T0T423 = sin(t1)*cos(t2);
		T0T433 = sin(t2);
		
		T0T414 = D2t*cos(t1) + D4t*(sin(t1)*cos(t3) + sin(t2)*sin(t3)*cos(t1)) + RL2t*sin(t1);
		T0T424 = D2t*sin(t1) + D4t*(sin(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t3)) - RL2t*cos(t1);
		T0T434 = -D4t*sin(t3)*cos(t2);
		*/
	}
	else if(finger_name=="index" or finger_name=="middle" or finger_name=="pinky"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
	  T0T411 = -sin(t2i + t3i + t4i)*cos(t1i);
    T0T421 = -sin(t1i)*sin(t2i + t3i + t4i);
    T0T431 = cos(t2i + t3i + t4i);
    
    T0T412 = -cos(t1i)*cos(t2i + t3i + t4i);
    T0T422 = -sin(t1i)*cos(t2i + t3i + t4i);
    T0T432 = -sin(t2i + t3i + t4i);
    
    T0T413 = sin(t1i);
    T0T423 = -cos(t1i);
    T0T433 = 0;
    
    T0T414 = -D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i);
    T0T424 = -D3i*sin(t1i)*sin(t2i) - D4i*sin(t1i)*sin(t2i + t3i);
    T0T434 = D3i*cos(t2i) + D4i*cos(t2i + t3i);
    /*
		T0T411 = -sin(t2 + t3 + t4)*cos(t1);
		T0T421 = -sin(t1)*sin(t2 + t3 + t4);
		T0T431 = cos(t2 + t3 + t4);
		
		T0T412 = -cos(t1)*cos(t2 + t3 + t4);
		T0T422 = -sin(t1)*cos(t2 + t3 + t4);
		T0T432 = -sin(t2 + t3 + t4);
		
		T0T413 = sin(t1);
		T0T423 = -cos(t1);
		T0T433 = 0;
		
		T0T414 = -D3*sin(t2)*cos(t1) - D4*sin(t2 + t3)*cos(t1);
		T0T424 = -D3*sin(t1)*sin(t2) - D4*sin(t1)*sin(t2 + t3);
		T0T434 = D3*cos(t2) + D4*cos(t2 + t3);
		*/
	}
	
	Eigen::Matrix4d DGM;
	DGM << T0T411, T0T412, T0T413, T0T414,
				 T0T421, T0T422, T0T423, T0T424,
				 T0T431, T0T432, T0T433, T0T434,
				 			0, 			0, 			0, 		  1;
	
	if(finger_name=="thumb")
		return palm_to_thumb_frame0()*DGM*thumb_frame4_to_tip();
	if(finger_name=="index")
		return palm_to_index_frame0()*DGM*index_frame4_to_tip();
	if(finger_name=="middle")
		return palm_to_middle_frame0()*DGM*middle_frame4_to_tip();
	if(finger_name=="pinky")
		return palm_to_pinky_frame0()*DGM*pinky_frame4_to_tip();

  return DGM;
}





Eigen::MatrixXd finger_geometric_jacobian_matrix(std::string finger_name, Eigen::Vector4d joint_state){
	double J11,J12,J13,J14;
	double J21,J22,J23,J24;
	double J31,J32,J33,J34;
	double J41,J42,J43,J44;
	double J51,J52,J53,J54;
	double J61,J62,J63,J64;
	
	if(finger_name=="thumb"){
	  double t1t, t2t, t3t, t4t;
	  t1t = joint_state(0);
	  t2t = joint_state(1);
	  t3t = joint_state(2);
	  t4t = joint_state(3);
	  
		J11 = -D2t*cos(t3t + t4t) + D4t*sin(t2t)*sin(t4t) + RL2t*sin(t2t)*sin(t3t + t4t);
    J21 = D2t*sin(t3t + t4t) + D4t*sin(t2t)*cos(t4t) + RL2t*sin(t2t)*cos(t3t + t4t);
    J31 = (D4t*cos(t3t) + RL2t)*cos(t2t);
    J41 = -sin(t3t + t4t)*cos(t2t);
    J51 = -cos(t2t)*cos(t3t + t4t);
    J61 = sin(t2t);

    J12 = 0;
    J22 = 0;
    J32 = D4t*sin(t3t);
    J42 = cos(t3t + t4t);
    J52 = -sin(t3t + t4t);
    J62 = 0;

    J13 = D4t*sin(t4t);
    J23 = D4t*cos(t4t);
    J33 = 0;
    J43 = 0;
    J53 = 0;
    J63 = 1;

    J14 = 0;
    J24 = 0;
    J34 = 0;
    J44 = 0;
    J54 = 0;
    J64 = 1;
	}
	else if(finger_name=="index" or finger_name=="middle" or finger_name=="pinky"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		J11 = 0;
    J21 = 0;
    J31 = D3i*sin(t2i) + D4i*sin(t2i + t3i);
    J41 = cos(t2i + t3i + t4i);
    J51 = -sin(t2i + t3i + t4i);
    J61 = 0;

    J12 = D3i*sin(t3i + t4i) + D4i*sin(t4i);
    J22 = D3i*cos(t3i + t4i) + D4i*cos(t4i);
    J32 = 0;
    J42 = 0;
    J52 = 0;
    J62 = 1;

    J13 = D4i*sin(t4i);
    J23 = D4i*cos(t4i);
    J33 = 0;
    J43 = 0;
    J53 = 0;
    J63 = 1;

    J14 = 0;
    J24 = 0;
    J34 = 0;
    J44 = 0;
    J54 = 0;
    J64 = 1;
	}
	
	Eigen::MatrixXd geometric_jacobian(6,4);
	
	// this gives the jacobian matrix of the arm frame4 w.r.t arm frame0 
	geometric_jacobian << J11, J12, J13, J14,
												J21, J22, J23, J24,
												J31, J32, J33, J34,
												J41, J42, J43, J44,
												J51, J52, J53, J54,
												J61, J62, J63, J64;
	
	
	Eigen::Matrix3d zero3 = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d product;
	
	// screw transformation matrix relating thumb frame4 to finger tip
	Eigen::Vector3d r_4tt, r_tt4;
	Eigen::Matrix3d R_tt4, r_4tt_hat;
	Eigen::Matrix3d R_4tt  = Eigen::Matrix3d::Identity();
	Eigen::MatrixXd screw_TM_tt4(6,6);
	r_4tt << rx_4tt, 0, 0;
	R_tt4 = R_4tt.transpose();
	r_4tt_hat = skew_symm_matrix(r_4tt);
	product = -R_tt4*r_4tt_hat;
	screw_TM_tt4 << R_tt4 , product,
								  zero3,    R_tt4;
	
	// screw transformation matrix relating thumb finger tip to palm frame
	Eigen::Matrix4d T_Ptt;
	Eigen::Matrix3d R_Ptt;
	Eigen::MatrixXd screw_TM_Ptt(6,6);
	T_Ptt = finger_direct_geometric_model("thumb",joint_state);
	R_Ptt << T_Ptt(0,0), T_Ptt(0,1), T_Ptt(0,2),
				   T_Ptt(1,0), T_Ptt(1,1), T_Ptt(1,2),
				   T_Ptt(2,0), T_Ptt(2,1), T_Ptt(2,2);
	screw_TM_Ptt << R_Ptt , zero3,
								  zero3,  R_Ptt;
	
	if(finger_name=="thumb")
		return screw_TM_Ptt*screw_TM_tt4*geometric_jacobian;
	else if(finger_name=="index")
		return geometric_jacobian;
	else if(finger_name=="middle")
		return geometric_jacobian;
	else if(finger_name=="pinky")
		return geometric_jacobian;
	
}





Eigen::Matrix4d finger_task_jacobian(std::string finger_name, Eigen::Vector4d joint_state){
	Eigen::Matrix4d task_jacobian;
	double J11,J12,J13,J14;
	double J21,J22,J23,J24;
	double J31,J32,J33,J34;
	double J41,J42,J43,J44;
	
	if(finger_name=="thumb"){
	  double t1t, t2t, t3t, t4t;
	  t1t = joint_state(0);
	  t2t = joint_state(1);
	  t3t = joint_state(2);
	  t4t = joint_state(3);
	  
		//Thumb Differential Jacobian
    //column 1
    J11 = -D2t*sin(t1t) + D4t*(-sin(t1t)*sin(t2t)*sin(t3t) + cos(t1t)*cos(t3t)) + RL2t*cos(t1t) + rx_4tt*(-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*sin(t2t)*cos(t3t + t4t) - sin(t3t + t4t)*cos(t1t)) - rz_4tt*sin(t1t)*cos(t2t);
    J21 = rx_4tt*(sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t))*cos(35*pi/36) + ry_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t))*cos(35*pi/36) + rz_4tt*cos(35*pi/36)*cos(t1t)*cos(t2t) + (D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t))*cos(35*pi/36);
    J31 = rx_4tt*(sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t))*sin(35*pi/36) + ry_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t))*sin(35*pi/36) + rz_4tt*sin(35*pi/36)*cos(t1t)*cos(t2t) + (D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t))*sin(35*pi/36);
    J41 = 0;

    //column 2
    J12 = D4t*sin(t3t)*cos(t1t)*cos(t2t) + rx_4tt*sin(t3t + t4t)*cos(t1t)*cos(t2t) + ry_4tt*cos(t1t)*cos(t2t)*cos(t3t + t4t) - rz_4tt*sin(t2t)*cos(t1t);
    J22 = -D4t*sin(35*pi/36)*sin(t2t)*sin(t3t) + D4t*sin(t1t)*sin(t3t)*cos(35*pi/36)*cos(t2t) + rx_4tt*(-sin(35*pi/36)*sin(t2t)*sin(t3t + t4t) + sin(t1t)*sin(t3t + t4t)*cos(35*pi/36)*cos(t2t)) + ry_4tt*(-sin(35*pi/36)*sin(t2t)*cos(t3t + t4t) + sin(t1t)*cos(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + rz_4tt*(-sin(35*pi/36)*cos(t2t) - sin(t1t)*sin(t2t)*cos(35*pi/36));
    J32 = D4t*sin(35*pi/36)*sin(t1t)*sin(t3t)*cos(t2t) + D4t*sin(t2t)*sin(t3t)*cos(35*pi/36) + rx_4tt*(sin(35*pi/36)*sin(t1t)*sin(t3t + t4t)*cos(t2t) + sin(t2t)*sin(t3t + t4t)*cos(35*pi/36)) + ry_4tt*(sin(35*pi/36)*sin(t1t)*cos(t2t)*cos(t3t + t4t) + sin(t2t)*cos(35*pi/36)*cos(t3t + t4t)) + rz_4tt*(-sin(35*pi/36)*sin(t1t)*sin(t2t) + cos(35*pi/36)*cos(t2t));
    J42 = 1;

    //column 3
    J13 = D4t*(-sin(t1t)*sin(t3t) + sin(t2t)*cos(t1t)*cos(t3t)) + rx_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*cos(t3t + t4t) - sin(t2t)*sin(t3t + t4t)*cos(t1t));
    J23 = D4t*(sin(t1t)*sin(t2t)*cos(t3t) + sin(t3t)*cos(t1t))*cos(35*pi/36) + D4t*sin(35*pi/36)*cos(t2t)*cos(t3t) + rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*cos(35*pi/36) + sin(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*cos(35*pi/36) - sin(35*pi/36)*sin(t3t + t4t)*cos(t2t));
    J33 = D4t*(sin(t1t)*sin(t2t)*cos(t3t) + sin(t3t)*cos(t1t))*sin(35*pi/36) - D4t*cos(35*pi/36)*cos(t2t)*cos(t3t) + rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*sin(35*pi/36) - cos(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*sin(35*pi/36) + sin(t3t + t4t)*cos(35*pi/36)*cos(t2t));
    J43 = 1;

    //column 4
    J14 = rx_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*cos(t3t + t4t) - sin(t2t)*sin(t3t + t4t)*cos(t1t));
    J24 = rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*cos(35*pi/36) + sin(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*cos(35*pi/36) - sin(35*pi/36)*sin(t3t + t4t)*cos(t2t));
    J34 = rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*sin(35*pi/36) - cos(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*sin(35*pi/36) + sin(t3t + t4t)*cos(35*pi/36)*cos(t2t));
    J44 = 1;
	}
	else if(finger_name=="index"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Index Differential Jacobian
    //column 1
    J11 = rx_4it*(sin(pi)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + ry_4it*(sin(pi)*cos(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + rz_4it*(-sin(pi)*sin(t1i)*cos(pi/36) + cos(pi)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(pi) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi)*cos(pi/36);
    J21 = rx_4it*(sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)*cos(t1i)) + ry_4it*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi/36)*cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4it*(sin(pi)*cos(t1i) + sin(t1i)*cos(pi/36)*cos(pi)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(pi) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(pi/36)*cos(pi);
    J31 = -rx_4it*sin(pi/36)*sin(t2i + t3i + t4i)*cos(t1i) - ry_4it*sin(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) + rz_4it*sin(pi/36)*sin(t1i) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi/36);
    J41 = 0;

    //column 2
    J12 = rx_4it*(-sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*sin(pi) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi)*cos(pi/36) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(pi);
    J22 = rx_4it*(sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4it*(sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)) - (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*cos(pi) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(pi/36)*cos(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(pi);
    J32 = rx_4it*(-sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4it*(sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*cos(pi/36) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi/36);
    J42 = 1;

    //column 3
    J13 = -D4i*sin(pi/36)*sin(pi)*sin(t2i + t3i) + D4i*sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i) - D4i*cos(pi)*cos(t1i)*cos(t2i + t3i) + rx_4it*(-sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J23 = D4i*sin(pi/36)*sin(t2i + t3i)*cos(pi) - D4i*sin(pi)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i) + rx_4it*(sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4it*(sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J33 = -D4i*sin(pi/36)*sin(t1i)*cos(t2i + t3i) - D4i*sin(t2i + t3i)*cos(pi/36) + rx_4it*(-sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4it*(sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));
    J43 = 1;

    //column 4
    J14 = rx_4it*(-sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J24 = rx_4it*(sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4it*(sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J34 = rx_4it*(-sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4it*(sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));
    J44 = 1;
	}
	else if(finger_name=="middle"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Middle Differential Jacobian
    //column 1
    J11 = rx_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + ry_4mt*(sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + rz_4mt*(-sin(pi)*sin(t1i) + cos(pi)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(pi) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi);
    J21 = rx_4mt*(sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) + ry_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4mt*(sin(pi)*cos(t1i) + sin(t1i)*cos(pi)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(pi) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(pi);
    J31 = 0;
    J41 = 0;

    //column 2
    J12 = rx_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(pi);
    J22 = rx_4mt*(-sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(pi);
    J32 = -D3i*sin(t2i) - D4i*sin(t2i + t3i) - rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);
    J42 = 1;

    //column 3
    J13 = D4i*sin(pi)*sin(t1i)*cos(t2i + t3i) - D4i*cos(pi)*cos(t1i)*cos(t2i + t3i) + rx_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J23 = -D4i*sin(pi)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(pi)*cos(t2i + t3i) + rx_4mt*(-sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi));
    J33 = -D4i*sin(t2i + t3i) - rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);
    J43 = 1;

    //column 4
    J14 = rx_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J24 = rx_4mt*(-sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi));
    J34 = -rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);
    J44 = 1;
	}
	else if(finger_name=="pinky"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Pinky Differential Jacobian
    //column 1
    J11 = rx_4pt*(sin(pi)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + ry_4pt*(sin(pi)*cos(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + rz_4pt*(-sin(pi)*sin(t1i)*cos(pi/36) + cos(pi)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(pi) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi)*cos(pi/36);
    J21 = rx_4pt*(sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)*cos(t1i)) + ry_4pt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi/36)*cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4pt*(sin(pi)*cos(t1i) + sin(t1i)*cos(pi/36)*cos(pi)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(pi) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(pi/36)*cos(pi);
    J31 = rx_4pt*sin(pi/36)*sin(t2i + t3i + t4i)*cos(t1i) + ry_4pt*sin(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) - rz_4pt*sin(pi/36)*sin(t1i) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi/36);
    J41 = 0;

    //column 2
    J12 = rx_4pt*(sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) - (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*sin(pi) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi)*cos(pi/36) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(pi);
    J22 = rx_4pt*(-sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*cos(pi) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(pi/36)*cos(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(pi);
    J32 = rx_4pt*(sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4pt*(-sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*cos(pi/36) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi/36);
    J42 = 1;

    //column 3
    J13 = D4i*sin(pi/36)*sin(pi)*sin(t2i + t3i) + D4i*sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i) - D4i*cos(pi)*cos(t1i)*cos(t2i + t3i) + rx_4pt*(sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J23 = -D4i*sin(pi/36)*sin(t2i + t3i)*cos(pi) - D4i*sin(pi)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i) + rx_4pt*(-sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J33 = D4i*sin(pi/36)*sin(t1i)*cos(t2i + t3i) - D4i*sin(t2i + t3i)*cos(pi/36) + rx_4pt*(sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4pt*(-sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));
    J43 = 1;

    //column 4
    J14 = rx_4pt*(sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J24 = rx_4pt*(-sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J34 = rx_4pt*(sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4pt*(-sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));
    J44 = 1;
	}
	
	// this gives the jacobian matrix of the finger-tip frame w.r.t hand-Palm frame 
	task_jacobian << J11, J12, J13, J14,
								   J21, J22, J23, J24,
									 J31, J32, J33, J34,
									 J41, J42, J43, J44;
	
	return task_jacobian;
}






Eigen::MatrixXd finger_position_jacobian(std::string finger_name, Eigen::Vector4d joint_state){
	Eigen::MatrixXd position_jacobian(3,4);
	double J11,J12,J13,J14;
	double J21,J22,J23,J24;
	double J31,J32,J33,J34;
	
	if(finger_name=="thumb"){
	  double t1t, t2t, t3t, t4t;
	  t1t = joint_state(0);
	  t2t = joint_state(1);
	  t3t = joint_state(2);
	  t4t = joint_state(3);
	  
		//Thumb Differential Jacobian
    //column 1
    J11 = -D2t*sin(t1t) + D4t*(-sin(t1t)*sin(t2t)*sin(t3t) + cos(t1t)*cos(t3t)) + RL2t*cos(t1t) + rx_4tt*(-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*sin(t2t)*cos(t3t + t4t) - sin(t3t + t4t)*cos(t1t)) - rz_4tt*sin(t1t)*cos(t2t);
    J21 = rx_4tt*(sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t))*cos(35*pi/36) + ry_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t))*cos(35*pi/36) + rz_4tt*cos(35*pi/36)*cos(t1t)*cos(t2t) + (D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t))*cos(35*pi/36);
    J31 = rx_4tt*(sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t))*sin(35*pi/36) + ry_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t))*sin(35*pi/36) + rz_4tt*sin(35*pi/36)*cos(t1t)*cos(t2t) + (D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t))*sin(35*pi/36);

    //column 2
    J12 = D4t*sin(t3t)*cos(t1t)*cos(t2t) + rx_4tt*sin(t3t + t4t)*cos(t1t)*cos(t2t) + ry_4tt*cos(t1t)*cos(t2t)*cos(t3t + t4t) - rz_4tt*sin(t2t)*cos(t1t);
    J22 = -D4t*sin(35*pi/36)*sin(t2t)*sin(t3t) + D4t*sin(t1t)*sin(t3t)*cos(35*pi/36)*cos(t2t) + rx_4tt*(-sin(35*pi/36)*sin(t2t)*sin(t3t + t4t) + sin(t1t)*sin(t3t + t4t)*cos(35*pi/36)*cos(t2t)) + ry_4tt*(-sin(35*pi/36)*sin(t2t)*cos(t3t + t4t) + sin(t1t)*cos(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + rz_4tt*(-sin(35*pi/36)*cos(t2t) - sin(t1t)*sin(t2t)*cos(35*pi/36));
    J32 = D4t*sin(35*pi/36)*sin(t1t)*sin(t3t)*cos(t2t) + D4t*sin(t2t)*sin(t3t)*cos(35*pi/36) + rx_4tt*(sin(35*pi/36)*sin(t1t)*sin(t3t + t4t)*cos(t2t) + sin(t2t)*sin(t3t + t4t)*cos(35*pi/36)) + ry_4tt*(sin(35*pi/36)*sin(t1t)*cos(t2t)*cos(t3t + t4t) + sin(t2t)*cos(35*pi/36)*cos(t3t + t4t)) + rz_4tt*(-sin(35*pi/36)*sin(t1t)*sin(t2t) + cos(35*pi/36)*cos(t2t));

    //column 3
    J13 = D4t*(-sin(t1t)*sin(t3t) + sin(t2t)*cos(t1t)*cos(t3t)) + rx_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*cos(t3t + t4t) - sin(t2t)*sin(t3t + t4t)*cos(t1t));
    J23 = D4t*(sin(t1t)*sin(t2t)*cos(t3t) + sin(t3t)*cos(t1t))*cos(35*pi/36) + D4t*sin(35*pi/36)*cos(t2t)*cos(t3t) + rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*cos(35*pi/36) + sin(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*cos(35*pi/36) - sin(35*pi/36)*sin(t3t + t4t)*cos(t2t));
    J33 = D4t*(sin(t1t)*sin(t2t)*cos(t3t) + sin(t3t)*cos(t1t))*sin(35*pi/36) - D4t*cos(35*pi/36)*cos(t2t)*cos(t3t) + rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*sin(35*pi/36) - cos(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*sin(35*pi/36) + sin(t3t + t4t)*cos(35*pi/36)*cos(t2t));

    //column 4
    J14 = rx_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*cos(t3t + t4t) - sin(t2t)*sin(t3t + t4t)*cos(t1t));
    J24 = rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*cos(35*pi/36) + sin(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*cos(35*pi/36) - sin(35*pi/36)*sin(t3t + t4t)*cos(t2t));
    J34 = rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*sin(35*pi/36) - cos(35*pi/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*sin(35*pi/36) + sin(t3t + t4t)*cos(35*pi/36)*cos(t2t));
	}
	else if(finger_name=="index"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Index Differential Jacobian
    //column 1
    J11 = rx_4it*(sin(pi)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + ry_4it*(sin(pi)*cos(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + rz_4it*(-sin(pi)*sin(t1i)*cos(pi/36) + cos(pi)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(pi) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi)*cos(pi/36);
    J21 = rx_4it*(sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)*cos(t1i)) + ry_4it*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi/36)*cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4it*(sin(pi)*cos(t1i) + sin(t1i)*cos(pi/36)*cos(pi)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(pi) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(pi/36)*cos(pi);
    J31 = -rx_4it*sin(pi/36)*sin(t2i + t3i + t4i)*cos(t1i) - ry_4it*sin(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) + rz_4it*sin(pi/36)*sin(t1i) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi/36);

    //column 2
    J12 = rx_4it*(-sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*sin(pi) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi)*cos(pi/36) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(pi);
    J22 = rx_4it*(sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4it*(sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)) - (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*cos(pi) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(pi/36)*cos(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(pi);
    J32 = rx_4it*(-sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4it*(sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*cos(pi/36) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi/36);

    //column 3
    J13 = -D4i*sin(pi/36)*sin(pi)*sin(t2i + t3i) + D4i*sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i) - D4i*cos(pi)*cos(t1i)*cos(t2i + t3i) + rx_4it*(-sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J23 = D4i*sin(pi/36)*sin(t2i + t3i)*cos(pi) - D4i*sin(pi)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i) + rx_4it*(sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4it*(sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J33 = -D4i*sin(pi/36)*sin(t1i)*cos(t2i + t3i) - D4i*sin(t2i + t3i)*cos(pi/36) + rx_4it*(-sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4it*(sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));

    //column 4
    J14 = rx_4it*(-sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J24 = rx_4it*(sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4it*(sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J34 = rx_4it*(-sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4it*(sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));
	}
	else if(finger_name=="middle"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Middle Differential Jacobian
    //column 1
    J11 = rx_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + ry_4mt*(sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + rz_4mt*(-sin(pi)*sin(t1i) + cos(pi)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(pi) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi);
    J21 = rx_4mt*(sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) + ry_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4mt*(sin(pi)*cos(t1i) + sin(t1i)*cos(pi)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(pi) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(pi);
    J31 = 0;

    //column 2
    J12 = rx_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(pi);
    J22 = rx_4mt*(-sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(pi);
    J32 = -D3i*sin(t2i) - D4i*sin(t2i + t3i) - rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);

    //column 3
    J13 = D4i*sin(pi)*sin(t1i)*cos(t2i + t3i) - D4i*cos(pi)*cos(t1i)*cos(t2i + t3i) + rx_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J23 = -D4i*sin(pi)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(pi)*cos(t2i + t3i) + rx_4mt*(-sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi));
    J33 = -D4i*sin(t2i + t3i) - rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);

    //column 4
    J14 = rx_4mt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J24 = rx_4mt*(-sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi));
    J34 = -rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);
	}
	else if(finger_name=="pinky"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Pinky Differential Jacobian
    //column 1
    J11 = rx_4pt*(sin(pi)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi)) + ry_4pt*(sin(pi)*cos(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(pi)*cos(t2i + t3i + t4i)) + rz_4pt*(-sin(pi)*sin(t1i)*cos(pi/36) + cos(pi)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(pi) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi)*cos(pi/36);
    J21 = rx_4pt*(sin(pi)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)*cos(t1i)) + ry_4pt*(sin(pi)*sin(t1i)*cos(t2i + t3i + t4i) - cos(pi/36)*cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4pt*(sin(pi)*cos(t1i) + sin(t1i)*cos(pi/36)*cos(pi)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(pi) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(pi/36)*cos(pi);
    J31 = rx_4pt*sin(pi/36)*sin(t2i + t3i + t4i)*cos(t1i) + ry_4pt*sin(pi/36)*cos(t1i)*cos(t2i + t3i + t4i) - rz_4pt*sin(pi/36)*sin(t1i) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(pi/36);

    //column 2
    J12 = rx_4pt*(sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i)) - (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*sin(pi) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi)*cos(pi/36) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(pi);
    J22 = rx_4pt*(-sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(pi/36)*cos(pi) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(pi/36)*cos(pi) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(pi);
    J32 = rx_4pt*(sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4pt*(-sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*cos(pi/36) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(pi/36);

    //column 3
    J13 = D4i*sin(pi/36)*sin(pi)*sin(t2i + t3i) + D4i*sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i) - D4i*cos(pi)*cos(t1i)*cos(t2i + t3i) + rx_4pt*(sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J23 = -D4i*sin(pi/36)*sin(t2i + t3i)*cos(pi) - D4i*sin(pi)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i) + rx_4pt*(-sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J33 = D4i*sin(pi/36)*sin(t1i)*cos(t2i + t3i) - D4i*sin(t2i + t3i)*cos(pi/36) + rx_4pt*(sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4pt*(-sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));

    //column 4
    J14 = rx_4pt*(sin(pi/36)*sin(pi)*sin(t2i + t3i + t4i) + sin(pi)*sin(t1i)*cos(pi/36)*cos(t2i + t3i + t4i) - cos(pi)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(pi/36)*sin(pi)*cos(t2i + t3i + t4i) - sin(pi)*sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36) + sin(t2i + t3i + t4i)*cos(pi)*cos(t1i));
    J24 = rx_4pt*(-sin(pi/36)*sin(t2i + t3i + t4i)*cos(pi) - sin(pi)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(pi/36)*cos(pi)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(pi/36)*cos(pi)*cos(t2i + t3i + t4i) + sin(pi)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(pi/36)*cos(pi));
    J34 = rx_4pt*(sin(pi/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(pi/36)) + ry_4pt*(-sin(pi/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(pi/36)*cos(t2i + t3i + t4i));
	}
	
	// this gives the jacobian matrix of the finger-tip frame w.r.t hand-Palm frame 
	position_jacobian << J11, J12, J13, J14,
								   J21, J22, J23, J24,
									 J31, J32, J33, J34;
	
	return position_jacobian;
}






// thumb finger constant transformation matrices
Eigen::Matrix4d palm_to_thumb_frame0(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R;
	Eigen::Vector3d P;
	R = Rotx(pi-pi/36);
	P	 << rx_Pt0, ry_Pt0, rz_Pt0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Eigen::Matrix4d thumb_frame4_to_tip(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d P;
	P	 << rx_4tt, ry_4tt, rz_4tt;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}

// index finger constant transformation matrices
Eigen::Matrix4d palm_to_index_frame0(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R;
	Eigen::Vector3d P;
	R = Rotz(pi)*Rotx(pi/36);
	P	 << rx_Pi0, ry_Pi0, rz_Pi0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Eigen::Matrix4d index_frame4_to_tip(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d P;
	P	 << rx_4it, ry_4it, rz_4it;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}

// middle finger constant transformation matrices
Eigen::Matrix4d palm_to_middle_frame0(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R;
	Eigen::Vector3d P;
	R = Rotz(pi);
	P	 << rx_Pm0, ry_Pm0, rz_Pm0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Eigen::Matrix4d middle_frame4_to_tip(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d P;
	P	 << rx_4mt, ry_4mt, rz_4mt;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}

// pinky finger constant transformation matrices
Eigen::Matrix4d palm_to_pinky_frame0(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R;
	Eigen::Vector3d P;
	R = Rotz(pi)*Rotx(-pi/36);
	P	 << rx_Pp0, ry_Pp0, rz_Pp0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Eigen::Matrix4d pinky_frame4_to_tip(void){
	Eigen::Matrix4d TM;
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d P;
	P	 << rx_4pt, ry_4pt, rz_4pt;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}



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
Eigen::MatrixXd finger_position_jacobian(std::string finger_name, Eigen::Vector4d joint_state);

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
    J21 = rx_4tt*(sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t))*cos(35*PI/36) + ry_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t))*cos(35*PI/36) + rz_4tt*cos(35*PI/36)*cos(t1t)*cos(t2t) + (D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t))*cos(35*PI/36);
    J31 = rx_4tt*(sin(t1t)*cos(t3t + t4t) + sin(t2t)*sin(t3t + t4t)*cos(t1t))*sin(35*PI/36) + ry_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t))*sin(35*PI/36) + rz_4tt*sin(35*PI/36)*cos(t1t)*cos(t2t) + (D2t*cos(t1t) + D4t*(sin(t1t)*cos(t3t) + sin(t2t)*sin(t3t)*cos(t1t)) + RL2t*sin(t1t))*sin(35*PI/36);

    //column 2
    J12 = D4t*sin(t3t)*cos(t1t)*cos(t2t) + rx_4tt*sin(t3t + t4t)*cos(t1t)*cos(t2t) + ry_4tt*cos(t1t)*cos(t2t)*cos(t3t + t4t) - rz_4tt*sin(t2t)*cos(t1t);
    J22 = -D4t*sin(35*PI/36)*sin(t2t)*sin(t3t) + D4t*sin(t1t)*sin(t3t)*cos(35*PI/36)*cos(t2t) + rx_4tt*(-sin(35*PI/36)*sin(t2t)*sin(t3t + t4t) + sin(t1t)*sin(t3t + t4t)*cos(35*PI/36)*cos(t2t)) + ry_4tt*(-sin(35*PI/36)*sin(t2t)*cos(t3t + t4t) + sin(t1t)*cos(35*PI/36)*cos(t2t)*cos(t3t + t4t)) + rz_4tt*(-sin(35*PI/36)*cos(t2t) - sin(t1t)*sin(t2t)*cos(35*PI/36));
    J32 = D4t*sin(35*PI/36)*sin(t1t)*sin(t3t)*cos(t2t) + D4t*sin(t2t)*sin(t3t)*cos(35*PI/36) + rx_4tt*(sin(35*PI/36)*sin(t1t)*sin(t3t + t4t)*cos(t2t) + sin(t2t)*sin(t3t + t4t)*cos(35*PI/36)) + ry_4tt*(sin(35*PI/36)*sin(t1t)*cos(t2t)*cos(t3t + t4t) + sin(t2t)*cos(35*PI/36)*cos(t3t + t4t)) + rz_4tt*(-sin(35*PI/36)*sin(t1t)*sin(t2t) + cos(35*PI/36)*cos(t2t));

    //column 3
    J13 = D4t*(-sin(t1t)*sin(t3t) + sin(t2t)*cos(t1t)*cos(t3t)) + rx_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*cos(t3t + t4t) - sin(t2t)*sin(t3t + t4t)*cos(t1t));
    J23 = D4t*(sin(t1t)*sin(t2t)*cos(t3t) + sin(t3t)*cos(t1t))*cos(35*PI/36) + D4t*sin(35*PI/36)*cos(t2t)*cos(t3t) + rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*cos(35*PI/36) + sin(35*PI/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*cos(35*PI/36) - sin(35*PI/36)*sin(t3t + t4t)*cos(t2t));
    J33 = D4t*(sin(t1t)*sin(t2t)*cos(t3t) + sin(t3t)*cos(t1t))*sin(35*PI/36) - D4t*cos(35*PI/36)*cos(t2t)*cos(t3t) + rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*sin(35*PI/36) - cos(35*PI/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*sin(35*PI/36) + sin(t3t + t4t)*cos(35*PI/36)*cos(t2t));

    //column 4
    J14 = rx_4tt*(-sin(t1t)*sin(t3t + t4t) + sin(t2t)*cos(t1t)*cos(t3t + t4t)) + ry_4tt*(-sin(t1t)*cos(t3t + t4t) - sin(t2t)*sin(t3t + t4t)*cos(t1t));
    J24 = rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*cos(35*PI/36) + sin(35*PI/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*cos(35*PI/36) - sin(35*PI/36)*sin(t3t + t4t)*cos(t2t));
    J34 = rx_4tt*((sin(t1t)*sin(t2t)*cos(t3t + t4t) + sin(t3t + t4t)*cos(t1t))*sin(35*PI/36) - cos(35*PI/36)*cos(t2t)*cos(t3t + t4t)) + ry_4tt*((-sin(t1t)*sin(t2t)*sin(t3t + t4t) + cos(t1t)*cos(t3t + t4t))*sin(35*PI/36) + sin(t3t + t4t)*cos(35*PI/36)*cos(t2t));
	}
	else if(finger_name=="index"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Index Differential Jacobian
    //column 1
    J11 = rx_4it*(sin(PI)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI)) + ry_4it*(sin(PI)*cos(PI/36)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(PI)*cos(t2i + t3i + t4i)) + rz_4it*(-sin(PI)*sin(t1i)*cos(PI/36) + cos(PI)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(PI) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(PI)*cos(PI/36);
    J21 = rx_4it*(sin(PI)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI)*cos(t1i)) + ry_4it*(sin(PI)*sin(t1i)*cos(t2i + t3i + t4i) - cos(PI/36)*cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4it*(sin(PI)*cos(t1i) + sin(t1i)*cos(PI/36)*cos(PI)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(PI) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(PI/36)*cos(PI);
    J31 = -rx_4it*sin(PI/36)*sin(t2i + t3i + t4i)*cos(t1i) - ry_4it*sin(PI/36)*cos(t1i)*cos(t2i + t3i + t4i) + rz_4it*sin(PI/36)*sin(t1i) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(PI/36);

    //column 2
    J12 = rx_4it*(-sin(PI/36)*sin(PI)*sin(t2i + t3i + t4i) + sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(PI/36)*sin(PI)*cos(t2i + t3i + t4i) - sin(PI)*sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(PI/36)*sin(PI) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(PI)*cos(PI/36) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(PI);
    J22 = rx_4it*(sin(PI/36)*sin(t2i + t3i + t4i)*cos(PI) - sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4it*(sin(PI/36)*cos(PI)*cos(t2i + t3i + t4i) + sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI)) - (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(PI/36)*cos(PI) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(PI/36)*cos(PI) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(PI);
    J32 = rx_4it*(-sin(PI/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)) + ry_4it*(sin(PI/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(PI/36)*cos(t2i + t3i + t4i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*cos(PI/36) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(PI/36);

    //column 3
    J13 = -D4i*sin(PI/36)*sin(PI)*sin(t2i + t3i) + D4i*sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i) - D4i*cos(PI)*cos(t1i)*cos(t2i + t3i) + rx_4it*(-sin(PI/36)*sin(PI)*sin(t2i + t3i + t4i) + sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(PI/36)*sin(PI)*cos(t2i + t3i + t4i) - sin(PI)*sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i));
    J23 = D4i*sin(PI/36)*sin(t2i + t3i)*cos(PI) - D4i*sin(PI)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i) + rx_4it*(sin(PI/36)*sin(t2i + t3i + t4i)*cos(PI) - sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4it*(sin(PI/36)*cos(PI)*cos(t2i + t3i + t4i) + sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI));
    J33 = -D4i*sin(PI/36)*sin(t1i)*cos(t2i + t3i) - D4i*sin(t2i + t3i)*cos(PI/36) + rx_4it*(-sin(PI/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)) + ry_4it*(sin(PI/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(PI/36)*cos(t2i + t3i + t4i));

    //column 4
    J14 = rx_4it*(-sin(PI/36)*sin(PI)*sin(t2i + t3i + t4i) + sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4it*(-sin(PI/36)*sin(PI)*cos(t2i + t3i + t4i) - sin(PI)*sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i));
    J24 = rx_4it*(sin(PI/36)*sin(t2i + t3i + t4i)*cos(PI) - sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4it*(sin(PI/36)*cos(PI)*cos(t2i + t3i + t4i) + sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI));
    J34 = rx_4it*(-sin(PI/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)) + ry_4it*(sin(PI/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(PI/36)*cos(t2i + t3i + t4i));
	}
	else if(finger_name=="middle"){
	  double t1i, t2i, t3i, t4i;
	  t1i = joint_state(0);
	  t2i = joint_state(1);
	  t3i = joint_state(2);
	  t4i = joint_state(3);
	  
		//Middle Differential Jacobian
    //column 1
    J11 = rx_4mt*(sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI)) + ry_4mt*(sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(PI)*cos(t2i + t3i + t4i)) + rz_4mt*(-sin(PI)*sin(t1i) + cos(PI)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(PI) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(PI);
    J21 = rx_4mt*(sin(PI)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI)*cos(t1i)) + ry_4mt*(sin(PI)*sin(t1i)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4mt*(sin(PI)*cos(t1i) + sin(t1i)*cos(PI)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(PI) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(PI);
    J31 = 0;

    //column 2
    J12 = rx_4mt*(sin(PI)*sin(t1i)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(PI)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i)) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(PI) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(PI);
    J22 = rx_4mt*(-sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI)) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(PI) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(PI);
    J32 = -D3i*sin(t2i) - D4i*sin(t2i + t3i) - rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);

    //column 3
    J13 = D4i*sin(PI)*sin(t1i)*cos(t2i + t3i) - D4i*cos(PI)*cos(t1i)*cos(t2i + t3i) + rx_4mt*(sin(PI)*sin(t1i)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(PI)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i));
    J23 = -D4i*sin(PI)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(PI)*cos(t2i + t3i) + rx_4mt*(-sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI));
    J33 = -D4i*sin(t2i + t3i) - rx_4mt*sin(t2i + t3i + t4i) - ry_4mt*cos(t2i + t3i + t4i);

    //column 4
    J14 = rx_4mt*(sin(PI)*sin(t1i)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4mt*(-sin(PI)*sin(t1i)*sin(t2i + t3i + t4i) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i));
    J24 = rx_4mt*(-sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4mt*(sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI));
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
    J11 = rx_4pt*(sin(PI)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI)) + ry_4pt*(sin(PI)*cos(PI/36)*cos(t1i)*cos(t2i + t3i + t4i) + sin(t1i)*cos(PI)*cos(t2i + t3i + t4i)) + rz_4pt*(-sin(PI)*sin(t1i)*cos(PI/36) + cos(PI)*cos(t1i)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*cos(PI) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(PI)*cos(PI/36);
    J21 = rx_4pt*(sin(PI)*sin(t1i)*sin(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI)*cos(t1i)) + ry_4pt*(sin(PI)*sin(t1i)*cos(t2i + t3i + t4i) - cos(PI/36)*cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + rz_4pt*(sin(PI)*cos(t1i) + sin(t1i)*cos(PI/36)*cos(PI)) + (D3i*sin(t1i)*sin(t2i) + D4i*sin(t1i)*sin(t2i + t3i))*sin(PI) + (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*cos(PI/36)*cos(PI);
    J31 = rx_4pt*sin(PI/36)*sin(t2i + t3i + t4i)*cos(t1i) + ry_4pt*sin(PI/36)*cos(t1i)*cos(t2i + t3i + t4i) - rz_4pt*sin(PI/36)*sin(t1i) - (-D3i*sin(t2i)*cos(t1i) - D4i*sin(t2i + t3i)*cos(t1i))*sin(PI/36);

    //column 2
    J12 = rx_4pt*(sin(PI/36)*sin(PI)*sin(t2i + t3i + t4i) + sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(PI/36)*sin(PI)*cos(t2i + t3i + t4i) - sin(PI)*sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i)) - (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(PI/36)*sin(PI) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(PI)*cos(PI/36) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*cos(PI);
    J22 = rx_4pt*(-sin(PI/36)*sin(t2i + t3i + t4i)*cos(PI) - sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(PI/36)*cos(PI)*cos(t2i + t3i + t4i) + sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*sin(PI/36)*cos(PI) + (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*cos(PI/36)*cos(PI) + (-D3i*cos(t1i)*cos(t2i) - D4i*cos(t1i)*cos(t2i + t3i))*sin(PI);
    J32 = rx_4pt*(sin(PI/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)) + ry_4pt*(-sin(PI/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(PI/36)*cos(t2i + t3i + t4i)) + (-D3i*sin(t2i) - D4i*sin(t2i + t3i))*cos(PI/36) - (-D3i*sin(t1i)*cos(t2i) - D4i*sin(t1i)*cos(t2i + t3i))*sin(PI/36);

    //column 3
    J13 = D4i*sin(PI/36)*sin(PI)*sin(t2i + t3i) + D4i*sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i) - D4i*cos(PI)*cos(t1i)*cos(t2i + t3i) + rx_4pt*(sin(PI/36)*sin(PI)*sin(t2i + t3i + t4i) + sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(PI/36)*sin(PI)*cos(t2i + t3i + t4i) - sin(PI)*sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i));
    J23 = -D4i*sin(PI/36)*sin(t2i + t3i)*cos(PI) - D4i*sin(PI)*cos(t1i)*cos(t2i + t3i) - D4i*sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i) + rx_4pt*(-sin(PI/36)*sin(t2i + t3i + t4i)*cos(PI) - sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(PI/36)*cos(PI)*cos(t2i + t3i + t4i) + sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI));
    J33 = D4i*sin(PI/36)*sin(t1i)*cos(t2i + t3i) - D4i*sin(t2i + t3i)*cos(PI/36) + rx_4pt*(sin(PI/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)) + ry_4pt*(-sin(PI/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(PI/36)*cos(t2i + t3i + t4i));

    //column 4
    J14 = rx_4pt*(sin(PI/36)*sin(PI)*sin(t2i + t3i + t4i) + sin(PI)*sin(t1i)*cos(PI/36)*cos(t2i + t3i + t4i) - cos(PI)*cos(t1i)*cos(t2i + t3i + t4i)) + ry_4pt*(sin(PI/36)*sin(PI)*cos(t2i + t3i + t4i) - sin(PI)*sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36) + sin(t2i + t3i + t4i)*cos(PI)*cos(t1i));
    J24 = rx_4pt*(-sin(PI/36)*sin(t2i + t3i + t4i)*cos(PI) - sin(PI)*cos(t1i)*cos(t2i + t3i + t4i) - sin(t1i)*cos(PI/36)*cos(PI)*cos(t2i + t3i + t4i)) + ry_4pt*(-sin(PI/36)*cos(PI)*cos(t2i + t3i + t4i) + sin(PI)*sin(t2i + t3i + t4i)*cos(t1i) + sin(t1i)*sin(t2i + t3i + t4i)*cos(PI/36)*cos(PI));
    J34 = rx_4pt*(sin(PI/36)*sin(t1i)*cos(t2i + t3i + t4i) - sin(t2i + t3i + t4i)*cos(PI/36)) + ry_4pt*(-sin(PI/36)*sin(t1i)*sin(t2i + t3i + t4i) - cos(PI/36)*cos(t2i + t3i + t4i));
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
	R = Rotx(PI-PI/36);
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
	R = Rotz(PI)*Rotx(PI/36);
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
	R = Rotz(PI);
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
	R = Rotz(PI)*Rotx(-PI/36);
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



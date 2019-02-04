


#include "../include/interface.h"

extern BHand* pBHand;
extern double q_des[MAX_DOF];
extern double q[MAX_DOF];
extern double tau_des[MAX_DOF];

static double q_mean[] = {
	(index_joint_0_max+index_joint_0_min)/2,   (index_joint_1_max+index_joint_1_min)/2,   (index_joint_2_max+index_joint_2_min)/2,   (index_joint_3_max+index_joint_3_min)/2,     // index
	(middle_joint_0_max+middle_joint_0_min)/2, (middle_joint_1_max+middle_joint_1_min)/2, (middle_joint_2_max+middle_joint_2_min)/2, (middle_joint_3_max+middle_joint_3_min)/2,   // middle
	(pinky_joint_0_max+pinky_joint_0_min)/2,   (pinky_joint_1_max+pinky_joint_1_min)/2,   (pinky_joint_2_max+pinky_joint_2_min)/2,   (pinky_joint_3_max+pinky_joint_3_min)/2,     // pinky
	(thumb_joint_0_max+thumb_joint_0_min)/2,   (thumb_joint_1_max+thumb_joint_1_min)/2,   (thumb_joint_2_max+thumb_joint_2_min)/2,   (thumb_joint_3_max+thumb_joint_3_min)/2};    // thumb

static double q_min[] = {
	index_joint_0_min+joint_safety_margin,  index_joint_1_min+joint_safety_margin,  index_joint_2_min+joint_safety_margin,  index_joint_3_min+joint_safety_margin,     // index
	middle_joint_0_min+joint_safety_margin, middle_joint_1_min+joint_safety_margin, middle_joint_2_min+joint_safety_margin, middle_joint_3_min+joint_safety_margin,   // middle
	pinky_joint_0_min+joint_safety_margin,  pinky_joint_1_min+joint_safety_margin,  pinky_joint_2_min+joint_safety_margin,  pinky_joint_3_min+joint_safety_margin,     // pinky
	thumb_joint_0_min+joint_safety_margin,  thumb_joint_1_min+joint_safety_margin,  thumb_joint_2_min+joint_safety_margin,  thumb_joint_3_min+joint_safety_margin};    // thumb

static double q_max[] = {
	index_joint_0_max,  index_joint_1_max,  index_joint_2_max,  index_joint_3_max,     // index
	middle_joint_0_max, middle_joint_1_max, middle_joint_2_max, middle_joint_3_max,   // middle
	pinky_joint_0_max,  pinky_joint_1_max,  pinky_joint_2_max,  pinky_joint_3_max,     // pinky
	thumb_joint_0_max,  thumb_joint_1_max,  thumb_joint_2_max,  thumb_joint_3_max};    // thumb

static void SetGains(){
	// This function should be called after the function SetMotionType() is called.
	// Once SetMotionType() function is called, all gains are reset using the default values.
	if (!pBHand) return;
	double kp[] = {
		500,  800, 900, 500,
		500,  800, 900, 500,
		500,  800, 900, 500,
		1000, 700, 600, 600
	};
	double kd[] = {
		25, 50, 55, 40,
		25, 50, 55, 40,
		25, 50, 55, 40,
		50, 50, 50, 40
	};
	pBHand->SetGainsEx(kp, kd);
}

void SetMeanJointValue(){
	for (int i=0; i<16; i++)
		q_des[i] = q_mean[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGains();
}
void SetMinJointValue(){
	for (int i=0; i<16; i++)
		q_des[i] = q_min[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGains();
}
void SetMaxJointValue(){
	for (int i=0; i<16; i++)
		q_des[i] = q_max[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGains();
}

void JointControl(Eigen::VectorXd q_desired){
  Eigen::VectorXd q_current(16), q_error(16), torque_desired(16);
	//for (int i=0; i<16; i++)
	//	q_des[i] = q_desired(i);
	//if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	//SetGains();
	
	if (pBHand) pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
	
  for(int i=0; i<16; i++){
    q_current(i) = q[i];
    //q_desired(i) = q_des[i];
  }
  //q_desired = q_mean;
  q_error = q_desired - q_current;
  torque_desired = 10.0*q_error;

  for(int i=0; i<16; i++){
    tau_des[i] = torque_desired(i);
  }
  pBHand->GetJointTorque(tau_des);
	
}


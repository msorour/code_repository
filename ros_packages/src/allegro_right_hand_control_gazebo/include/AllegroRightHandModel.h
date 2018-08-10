#include"Eigen/Dense"

#define D2t  0.005
#define D4t  0.0514
#define RL2t 0.0554

#define D3   0.054
#define D4   0.0384

#define pi   3.14159

using namespace std;
using namespace Eigen;

Matrix3d Rotx(double t);
Matrix3d Roty(double t);
Matrix3d Rotz(double t);
Matrix4d palm_to_thumb_frame0(void);
Matrix4d thumb_frame4_to_tip(void);
Matrix4d palm_to_index_frame0(void);
Matrix4d index_frame4_to_tip(void);
Matrix4d palm_to_middle_frame0(void);
Matrix4d middle_frame4_to_tip(void);
Matrix4d palm_to_pinky_frame0(void);
Matrix4d pinky_frame4_to_tip(void);


Matrix4d finger_direct_geometric_model(char* finger_name, Vector4d joint_state){
	double t1, t2, t3, t4;
	t1 = joint_state(0);
	t2 = joint_state(1);
	t3 = joint_state(2);
	t4 = joint_state(3);
	double T0T411,T0T421,T0T431, T0T412,T0T422,T0T432, T0T413,T0T423,T0T433, T0T414,T0T424,T0T434;
	
	if(finger_name=="thumb"){
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
	}
	else if(finger_name=="index" or finger_name=="middle" or finger_name=="pinky"){
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
	}
	
	Matrix4d DGM;
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
}




// thumb finger constant transformation matrices
Matrix4d palm_to_thumb_frame0(void){
	Matrix4d TM;
	Matrix3d R;
	Vector3d P;
	R = Rotx(pi-pi/36);
	P	 << -9.1/1000, 16.958/1000, 21.712/1000;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Matrix4d thumb_frame4_to_tip(void){
	Matrix4d TM;
	Matrix3d R = Matrix3d::Identity();
	Vector3d P;
	P	 << 59.3/1000, 0, 0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}

// index finger constant transformation matrices
Matrix4d palm_to_index_frame0(void){
	Matrix4d TM;
	Matrix3d R;
	Vector3d P;
	R = Rotz(pi)*Rotx(pi/36);
	P	 << 9.1/1000, 45.098/1000, 109.293/1000;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Matrix4d index_frame4_to_tip(void){
	Matrix4d TM;
	Matrix3d R = Matrix3d::Identity();
	Vector3d P;
	P	 << 43.7/1000, 0, 0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}

// middle finger constant transformation matrices
Matrix4d palm_to_middle_frame0(void){
	Matrix4d TM;
	Matrix3d R;
	Vector3d P;
	R = Rotz(pi);
	P	 << 9.1/1000, 0, 111.6/1000;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Matrix4d middle_frame4_to_tip(void){
	Matrix4d TM;
	Matrix3d R = Matrix3d::Identity();
	Vector3d P;
	P	 << 43.7/1000, 0, 0;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}

// pinky finger constant transformation matrices
Matrix4d palm_to_pinky_frame0(void){
	Matrix4d TM;
	Matrix3d R;
	Vector3d P;
	R = Rotz(pi)*Rotx(-pi/36);
	P	 << 9.1/1000, -45.098/1000, 109.293/1000;
	TM << R,		 P,
				0,0,0, 1;
	return TM;
}
Matrix4d pinky_frame4_to_tip(void){
	Matrix4d TM;
	Matrix3d R = Matrix3d::Identity();
	Vector3d P;
	P	 << 43.7/1000, 0, 0;
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





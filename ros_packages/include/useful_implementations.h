#include"Eigen/Dense"

Eigen::Matrix3d Rotx(double t);
Eigen::Matrix3d Roty(double t);
Eigen::Matrix3d Rotz(double t);
Eigen::Matrix3d skew_symm_matrix(Eigen::Vector3d vec);
Eigen::VectorXd transfer_matrix_to_rose_rpy(Eigen::Matrix4d TM);
Eigen::MatrixXd geometric_to_analytic_jacobian_rpy(Eigen::VectorXd pose);
Eigen::MatrixXd Pinv_damped(Eigen::MatrixXd J, double damping);
Eigen::Vector3d OnlineMP_L5B(double ti, double tf, double tn, double Pi, double Pf);

// Useful Implementations
Eigen::Matrix3d Rotx(double t){
	Eigen::Matrix3d R;
	R << 1, 		 0,			 	0,
			 0, cos(t), -sin(t),
			 0, sin(t),  cos(t);
	return R;
}
Eigen::Matrix3d Roty(double t){
	Eigen::Matrix3d R;
	R <<  cos(t), 0,	sin(t),
			 			 0, 1, 			 0,
			 -sin(t), 0,  cos(t);
	return R;
}
Eigen::Matrix3d Rotz(double t){
	Eigen::Matrix3d R;
	R << cos(t), -sin(t), 0,
			 sin(t),  cos(t), 0,
			  	 	0, 			 0, 1;
	return R;
}

Eigen::Matrix3d skew_symm_matrix(Eigen::Vector3d vec){
	Eigen::Matrix3d hat;
	hat <<			 0, -vec(2),  vec(1),
				  vec(2), 	 		0, -vec(0),
				 -vec(1),	 vec(0),			 0;
	return hat;
}

Eigen::VectorXd transfer_matrix_to_rose_rpy(Eigen::Matrix4d TM){
	Eigen::VectorXd POSE(6);
  
  double sx,sy,sz,nx,ny,nz,ax,ay,az,Px,Py,Pz;
  double alpha,beta,gamma;
  // alpha -> yaw : about z
  // beta  -> pitch : about y
  // gamma -> roll : about x
  
  sx = TM(0,0);		sy = TM(1,0);		sz = TM(2,0);
  nx = TM(0,1);		ny = TM(1,1);		nz = TM(2,1);
  ax = TM(0,2);		ay = TM(1,2);		az = TM(2,2);
  Px = TM(0,3);		Py = TM(1,3);		Pz = TM(2,3);
  
  alpha = atan2( sy , sx ); 
  beta  = atan2( -sz , sqrt( nz*nz + az*az ) );
  gamma = atan2( nz , az );
  
  POSE << Px, Py, Pz, gamma, beta, alpha;
  return POSE;
}

Eigen::MatrixXd geometric_to_analytic_jacobian_rpy(Eigen::VectorXd pose){ 
  Eigen::MatrixXd J(6,6), JInv(6,6);
  double alpha,beta,gamma;
  
  gamma = pose(3);		// gamma -> roll  : about x
  beta  = pose(4);		// beta  -> pitch : about y
  alpha = pose(5);		// alpha -> yaw   : about z
  
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d zero3 = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Omega;
  
  Omega << cos(alpha)*cos(beta), -sin(alpha), 0,
					 sin(alpha)*cos(beta),  cos(alpha),	0,
					 					 -sin(beta), 					 0, 1;
	
	/*
	Omega << cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1,
					 					-sin(gamma), cos(gamma)					 , 0,
					 cos(gamma)/cos(beta), sin(gamma)/cos(beta), 0;
	*/
	/*
	Omega << 					 cos(alpha)/cos(beta), 						sin(alpha)/cos(beta), 0,
					 										-sin(alpha),  										cos(alpha),	0,
					 cos(alpha)*sin(beta)/cos(beta), -sin(alpha)*sin(beta)/cos(beta), 1;
	*/
	J << I3, 		zero3,
			 zero3, Omega;
  
  JInv = Pinv_damped(J,0.001);
  return JInv;
}

Eigen::MatrixXd Pinv_damped( Eigen::MatrixXd J , double damping )    // Moore-Penrose Pseudo-inverse computing function
{
  int r = J.rows();
  int c = J.cols();
  Eigen::MatrixXd Jpinv( c , r );
  
  if( r >= c )   // tall matrix
  {
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(c,c);
    Jpinv = ( J.transpose()*J + damping*damping*I ).inverse() * J.transpose();
  }
  else           // fat matrix
  {
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(r,r);
    Jpinv = J.transpose() * ( J*J.transpose() + damping*damping*I ).inverse();
  }
  return Jpinv;
}






// This function calculates the Motion Profile online. It takes the initial and final conditions together with the current time instant (tn) and returns a 3D vector of Position, Velocity and Acceleration commands.
Eigen::Vector3d OnlineMP_L5B( double ti, double tf, double tn, double Pi, double Pf ){
  Eigen::Vector3d MPM;
  
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

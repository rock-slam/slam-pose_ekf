#include "KalmanFilter.hpp"
#include <Eigen/LU> 
#include <math.h>

using namespace KalmanFilter;

void PositionKF::update( Eigen::Vector3d v )
{

	// update the state and covariance matrices
	
	 
	
	 //State transition 
	//position
	x_kp.xi()=x_kp.xi()+R_n_n*v*d_t;
	
	//atitude
	x_kp.yaw()=x_kp.yaw();
	//x_kp.yaw()(0,0)=-28*3.141592653589793238462643383279/180;
	
	
	
	//Recalculates the new Rotation of yaw bias, takes from nav to new nav frame 
	R_n_n=Eigen::AngleAxisd(x_kp.yaw()(0,0), Eigen::Vector3d::UnitZ()); 
	
	//Jacobian of the state transition matrix 
	JacobianF(v); 
	
	//std::cout<<" JF = "<<std::endl<<JF<<std::endl;
	//covariance update 
	P_kp = JF*P_k*JF.transpose() + Q;
      

	// for now just copy the state, we could make 
	// this conditional depending on wether there will
	// be a correction or not.
	x_k.vector() = x_kp.vector();
	P_k = P_kp;
	
	
	
}

void PositionKF::correction_pos( Eigen::Vector3d p )
{
    // correct the estimate and covariance according to measurement
    
    Eigen::Vector3d y = p - H_pos*x_kp.vector();
    
    Eigen::Matrix3d S = H_pos*P_kp*H_pos.transpose()+R_pos;

    Eigen::Matrix<double, State::SIZE, MEASUREMENT_SIZE> K = P_kp*H_pos.transpose()*S.inverse();
   
    x_kp.vector() = x_kp.vector() + K*y;
    
    P_kp = (Eigen::Matrix<double, State::SIZE, State::SIZE>::Identity() - K*H_pos)*P_kp;
    
    // for now just copy the state, we could make 
    // this conditional depending on wether there will
    // be a correction or not.
    x_k.vector() = x_kp.vector();
    P_k = P_kp;
    
   
   // std::cout<<" erro "<<y.transpose()<<std::endl;
    //std::cout<<" *x_kp "<<x_kp.vector().transpose()<<std::endl;
    //std::cout<<" p "<<p.transpose()<<std::endl;
    //std::cout<<" Hpos "<<H_pos<<std::endl;
    //std::cout<<" k "<<std::endl<<K<<std::endl;
    std::cout<<" yaw = "<<x_kp.yaw()(0,0)*180/3.14<<std::endl;
}

void PositionKF::JacobianF ( Eigen::Vector3d v ){
    
    JF.setIdentity(); 
    
    //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
    dR_z<<-sin(x_kp.yaw()(0,0)), -cos(x_kp.yaw()(0,0)), 0,cos(x_kp.yaw()(0,0)),-sin(x_kp.yaw()(0,0)),0,0,0,0; 
    
 
    //position jacobian 
    JF.block<3,1>(0,3)
	  = dR_z*v*d_t;
   
  	
}










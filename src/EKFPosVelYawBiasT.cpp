#include "EKFPosVelYawBiasT.hpp"
#include <Eigen/LU> 
#include <math.h>
#include <fault_detection/FaultDetection.hpp>

using namespace pose_estimator;


/** CHECK */ 
EKFPosVelYawBiasT::EKFPosVelYawBiasT() 
{
    filter =  new ExtendedKalmanFilter::EKF<State::SIZE>;
    chi_square = new fault_detection::ChiSquared;
}

EKFPosVelYawBiasT::~EKFPosVelYawBiasT()
{
    delete filter; 
    delete chi_square;
}


/** update the filter */ 
void EKFPosVelYawBiasT::predict(const Eigen::Vector3d &acc_nav, double dt )
{	
  
  
    filter->P  = P; 
    filter->x  = x.vector(); 
    
      //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb = Eigen::AngleAxisd( x.yaw()(0,0), Eigen::Vector3d::UnitZ() ); 

    //sets the transition matrix 
    Eigen::Matrix<double, State::SIZE, 1> f;
    f.start<3>() = x.xi() + x.vi() * dt; 
    f.segment<3>(3) = x.vi() + R_w2wb * acc_nav * dt; 
    f.end<1>() = x.yaw();

    //sets the Jacobian of the state transition matrix 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F
	= jacobianF(acc_nav, dt );
  
	
    //updates the Kalman Filter 
    filter->prediction( f, J_F, Q ); 

    
    //get the updated values 
    x.vector()=filter->x; 
    P=filter->P; 
    

}	


/** calculates the process noise in world frame*/ 
void EKFPosVelYawBiasT::processNoise(const Eigen::Matrix<double, State::SIZE, State::SIZE> &Q)
{ 
    this->Q = Q; 

    // Rotate from world to world with bias 
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 

    Eigen::Matrix3d R_w2wb_ = Eigen::Matrix3d(R_w2wb);    

    this->Q.block<3,3>(0,0) = R_w2wb_*this->Q.block<3,3>(0,0) *R_w2wb_.transpose();
} 



/** Calculates the Jacobian of the transition matrix */ 
Eigen::Matrix<double, State::SIZE, State::SIZE> EKFPosVelYawBiasT::jacobianF( const Eigen::Vector3d &acc_nav, double dt )
{
    //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
    dR_z << -sin( x.yaw()(0,0) ), -cos( x.yaw()(0,0) ), 0, cos( x.yaw()(0,0) ), -sin( x.yaw()(0,0) ),0 ,0 ,0 ,0; 

    //jacobian 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F; 
    J_F.setIdentity(); 
    J_F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt; 
    J_F.block<3,1>(3,6)
	= dR_z * acc_nav * dt;

	
    std::cout << "Jacobian of f \n" << J_F << std::endl; 
    std::cout << "Jacobian of f \n" 
	<< "1 0 0 dt 0 0 0 \n" 
	<< "0 1 0 0 dt 0 0 \n"  
	<< "0 0 1 0 0 dt 0 \n"  
	<< "0 0 0 1 0 0 R*a*dt \n" 
	<< "0 0 0 0 1 0 R*a*dt \n"  
	<< "0 0 0 0 0 1 R*a*dt \n"  
	<< "0 0 0 0 0 0 1 \n" 
	<<std::endl; 
    
    return J_F;
}

/** configurarion hook */ 
void EKFPosVelYawBiasT::init(const Eigen::Matrix<double, State::SIZE, State::SIZE> &P, const Eigen::Matrix<double,State::SIZE,1> &x)
{
    Q.setZero(); 
    this->P = P;
    this->x.vector() = x; 

}


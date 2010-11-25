#include "EKFPosYawBiasT.hpp"
#include <Eigen/LU> 
#include <math.h>
#include "FaultDetection.hpp"

using namespace pose_ekf;


/** CHECK */ 
EKFPosYawBiasT::EKFPosYawBiasT() 
{
    filter =  new ExtendedKalmanFilter::EKF<StatePosYawBias::SIZE>;
    chi_square = new fault_detection::ChiSquared;
}

EKFPosYawBiasT::~EKFPosYawBiasT()
{
    delete filter; 
    delete chi_square;
}



/** update the filter */ 
void EKFPosYawBiasT::predict( const Eigen::Vector3d &translation_world )
{	
  
  
    filter->P  = P; 
    filter->x  = x.vector(); 
    
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb = Eigen::AngleAxisd( x.yaw()(0,0), Eigen::Vector3d::UnitZ() ); 

    //sets the transition matrix 
    Eigen::Matrix<double, StatePosYawBias::SIZE, 1> f;
    f.start<3>() = x.xi() + R_w2wb * translation_world;
    f.end<1>() = x.yaw();

    //sets the Jacobian of the state transition matrix 
    Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> J_F
	= jacobianF(translation_world );
  
	
    //updates the Kalman Filter 
    filter->prediction( f, J_F, Q ); 

    
    //get the updated values 
    x.vector()=filter->x; 
    P=filter->P; 
    

}	


/** calculates the process noise in world frame*/ 
void EKFPosYawBiasT::processNoise(const Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> &Q)
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
Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> EKFPosYawBiasT::jacobianF( const Eigen::Vector3d &translation_world )
{
    //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
    dR_z << -sin( x.yaw()(0,0) ), -cos( x.yaw()(0,0) ), 0, cos( x.yaw()(0,0) ), -sin( x.yaw()(0,0) ),0 ,0 ,0 ,0; 

    //jacobian 
    Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> J_F; 
    J_F.setIdentity(); 
    J_F.block<3,1>(0,3)
	= dR_z * translation_world;

    return J_F;
}

/** configurarion hook */ 
void EKFPosYawBiasT::init(const Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> &P, const Eigen::Matrix<double,StatePosYawBias::SIZE,1> &x)
{
    Q.setZero(); 
    this->P = P;
    this->x.vector() = x; 

}


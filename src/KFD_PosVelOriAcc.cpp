#include "KFD_PosVelOriAcc.hpp"
#include <Eigen/LU> 
#include <math.h>
#include <fault_detection/FaultDetection.hpp>

using namespace pose_estimator;


/** CHECK */ 
KFD_PosVelOriAcc::KFD_PosVelOriAcc() 
{
    filter =  new KalmanFilter::KF<StatePosVelOriAcc::SIZE>;
    chi_square = new fault_detection::ChiSquared;
}

KFD_PosVelOriAcc::~KFD_PosVelOriAcc()
{
    delete filter; 
    delete chi_square;
}


/** update the filter */ 
void KFD_PosVelOriAcc::predict(const Eigen::Vector3d &vel_inertial, double dt, Eigen::Matrix3d R_iertial_2_world  )
{	
  
  
    filter->P  = P; 
    filter->x  = x.vector(); 
    
    //sets the transition matrix 
    Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> F;
    F.setZero(); 
    F.block<3,3>(0,3) = R_iertial_2_world; 
    F.block<3,3>(0,9)  << 0, -vel_inertial(2), vel_inertial(1),
					      vel_inertial(2), 0, -vel_inertial(0),
					     -vel_inertial(1), vel_inertial(0), 0 ; 
    F.block<3,3>(3,6) = Eigen::Matrix3d::Identity(); 

  /*  std::cout << "F \n" << F << std::endl; 
    std::cout 
	<< "0 0 0 R R R 0 0 0 0 -vel_z vel_y\n" 
	<< "0 0 0 R R R 0 0 0 vel_z 0 -vel_x\n"  
	<< "0 0 0 R R R 0 0 0 -vel_y vel_x 0\n"  
	<< "0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 \n" 
	<< "0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 \n"  
	<< "0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 \n"  
	<< "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	<< "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	<< "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	<< "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	<< "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	<< "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	<<std::endl;*/
    
    //updates the Kalman Filter 
    filter->predictionDiscrete( F, Q, dt); 

    //get the updated values 
    x.vector()=filter->x; 
    P=filter->P; 
    

}	


/** calculates the process noise in world frame*/ 
void KFD_PosVelOriAcc::processNoise(const Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> &Q)
{ 
    this->Q = Q; 

} 

/** configurarion hook */ 
void KFD_PosVelOriAcc::init(const Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> &P, const Eigen::Matrix<double,StatePosVelOriAcc::SIZE,1> &x)
{
    Q.setZero(); 
    this->P = P;
    this->x.vector() = x; 

}


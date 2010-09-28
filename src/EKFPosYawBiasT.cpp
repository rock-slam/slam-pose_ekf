#include "EKFPosYawBiasT.hpp"
#include <Eigen/LU> 
#include <math.h>
#include <fault_detection/FaultDetection.hpp>

using namespace pose_estimator;


/** CHECK */ 
EKFPosYawBiasT::EKFPosYawBiasT() 
{
    filter =  new ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE>;
    chi_square = new fault_detection::ChiSquared<DEGREE_OF_FREEDOM>;
}

EKFPosYawBiasT::~EKFPosYawBiasT()
{
    delete filter; 
    delete chi_square;
}



/** update the filter */ 
void EKFPosYawBiasT::predict( const Eigen::Vector3d &translation_world )
{	
  
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb = Eigen::AngleAxisd( x.yaw()(0,0), Eigen::Vector3d::UnitZ() ); 

    //sets the transition matrix 
    Eigen::Matrix<double, State::SIZE, 1> f;
    f.start<3>() = x.xi() + R_w2wb * translation_world;
    f.end<1>() = x.yaw();

    //sets the Jacobian of the state transition matrix 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F
	= jacobianF(translation_world );
  
    //updates the Kalman Filter 
    filter->prediction( f, J_F, Q ); 

    //get the updated values 
    x.vector()=filter->x; 
    P=filter->P; 

}

bool EKFPosYawBiasT::correction(const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &p, 
			const Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> &R, 
			const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &h, 
			const Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE> &J_H, 
			float reject_threshold  )
{

   //innovation steps
    filter->innovation( p, h, J_H, R ); 

    bool reject_observation; 
    //test to reject data 
    if ( reject_threshold!=0 ) 
	reject_observation = chi_square->rejectData( filter->y.start<DEGREE_OF_FREEDOM>(), filter->S.block<DEGREE_OF_FREEDOM,DEGREE_OF_FREEDOM>(0,0), reject_threshold );
    else
	reject_observation = false;
    
    if(!reject_observation)
    {
    
	//Kalman Gain
	filter->gain( J_H ); 
	
	//update teh state 
	filter->update(J_H ); 

	//get the corrected values 
	x.vector() = filter->x; 
	P = filter->P; 
	
	return false; 

    }
    else
    { 
	
	std::cout<< " Rejected Observation " << std::endl; 
	return true; 
    
    }

}


/** Calculates the Jacobian of the transition matrix */ 
Eigen::Matrix<double, State::SIZE, State::SIZE> EKFPosYawBiasT::jacobianF( const Eigen::Vector3d &translation_world )
{
    //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
    dR_z << -sin( x.yaw()(0,0) ), -cos( x.yaw()(0,0) ), 0, cos( x.yaw()(0,0) ), -sin( x.yaw()(0,0) ),0 ,0 ,0 ,0; 

    //jacobian 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F; 
    J_F.setIdentity(); 
    J_F.block<3,1>(0,3)
	= dR_z * translation_world;

    return J_F;
}

/** configurarion hook */ 
void EKFPosYawBiasT::init(const Eigen::Matrix<double, State::SIZE, State::SIZE> &P, const Eigen::Matrix<double,State::SIZE,1> &x)
{
    Q.setZero(); 
    this->P = P;
    this->x.vector() = x; 

}


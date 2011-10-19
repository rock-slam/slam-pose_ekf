#include "KFD_PosVel.hpp"
#include <Eigen/LU> 
#include <math.h>
#include "FaultDetection.hpp"

using namespace pose_ekf;


/** CHECK */ 
KFD_PosVel::KFD_PosVel() 
{
    filter =  new KalmanFilter::KF<StatePosVel::SIZE>();
}

KFD_PosVel::~KFD_PosVel()
{
    delete filter; 
    
}

void KFD_PosVel::predict(Eigen::Quaterniond R_body_2_world, double dt, Eigen::Matrix<double, StatePosVel::SIZE, StatePosVel::SIZE> process_noise)
{
 	  
	  
	  /** Kalman Filter */ 
	  //sets the transition matrix 
	  Eigen::Matrix<double, StatePosVel::SIZE, StatePosVel::SIZE> F;
	  F.setZero();
	  F.block<3,3>(0,3) =  Eigen::Matrix3d( R_body_2_world );
	  
	 //std::cout << "F \n" << F << std::endl; 
	/* std::cout 
	      << "0 0 0 0 0 0 \n" 
	      << "0 0 0 0 0 0 \n"  
	      << "0 0 0 0 0 0 \n"  
	      << "0 0 0 R R R \n" 
	      << "0 0 0 R R R \n"  
	      << "0 0 0 R R R \n"  
	      <<std::endl;*/
	  
	  //updates the Kalman Filter 
	  filter->predictionDiscrete( F, process_noise, dt); 

	  //get the updated values 
	  x.vector()=filter->x; 
	  
}

bool KFD_PosVel::positionObservation(Eigen::Vector3d position, Eigen::Matrix3d covariance, float reject_position_threshol)
{

      Eigen::Matrix<double, _POS_MEASUREMENT_SIZE, StatePosVel::SIZE> H; 
      H.setZero(); 
      H.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); 
      
      //position_world = position;
      bool reject =  filter->correctionChiSquare<_POS_MEASUREMENT_SIZE,_POS_DEGREE_OF_FREEDOM>( position,  covariance,H, reject_position_threshol );
      
      x.vector() = filter->x;
      
      return reject; 
      
}

bool KFD_PosVel::velocityObservation(Eigen::Vector3d velocity, Eigen::Matrix3d covariance, float reject_velocity_threshol)
{
  

      Eigen::Matrix<double, _VEL_MEASUREMENT_SIZE, StatePosVel::SIZE> H; 
      H.setZero(); 
      H.block<3,3>(0,3) = Eigen::Matrix3d::Identity(); 
      //position_world = position;
      bool reject =  filter->correctionChiSquare<_VEL_MEASUREMENT_SIZE,_VEL_DEGREE_OF_FREEDOM>( velocity,  covariance, H, reject_velocity_threshol );
      x.vector() = filter->x;
      
      return reject; 
      
}

bool KFD_PosVel::positionZObservation(double z, double error, double rejection_threshold) 
{
    Eigen::Matrix<double,1,1>  position;
    position(0,0) =  z; 

    Eigen::Matrix<double,1 ,1> cov;
    cov(0,0) = error; 

    Eigen::Matrix<double, 1, StatePosVel::SIZE> H; 
    H.setZero(); 
    H(0,2) = 1; 
    
    bool reject =  filter->correctionChiSquare<1,1>( position,  cov, H, rejection_threshold );
    
    x.vector() = filter->x;
      
    return reject; 

}
void KFD_PosVel::setPosition( Eigen::Vector3d position, Eigen::Matrix3d covariance )
{
      //The initial covariance in position 
      filter->P.block<3,3>(0,0) = covariance ;
      
      x.pos_world() = position; 
      filter->x = x.vector();

}
void KFD_PosVel::setVelocity( Eigen::Vector3d velocity, Eigen::Matrix3d covariance )
{
      //The initial covariance in position 
      filter->P.block<3,3>(3,3) = covariance ;
      
      x.vel_body() = velocity; 
      filter->x = x.vector();

}

void KFD_PosVel::copyState ( const KFD_PosVel& kfd )
{
  
    x = kfd.x; 
    filter->x = kfd.filter->x;
    filter->P = kfd.filter->P;
    
}


Eigen::Vector3d KFD_PosVel::getPosition()
{
    return x.pos_world(); 
}
	
Eigen::Vector3d KFD_PosVel::getVelocity()
{
    return x.vel_body(); 
}

Eigen::Matrix3d KFD_PosVel::getPositionCovariance()
{
    return filter->P.block<3,3>(0,0);
}

Eigen::Matrix3d KFD_PosVel::getVelocityCovariance()
{
    return filter->P.block<3,3>(3,3);
}


/** configurarion hook */ 
void KFD_PosVel::init(const Eigen::Matrix<double, StatePosVel::SIZE, StatePosVel::SIZE> &P, const Eigen::Matrix<double,StatePosVel::SIZE,1> &x)
{
    Q.setZero(); 
    this->x.vector() = x; 
    filter->x = x; 
    filter->P = P; 

}

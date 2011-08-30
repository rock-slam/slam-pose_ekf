#include "KFD_PosVelAcc.hpp"
#include <Eigen/LU> 
#include <math.h>
#include "FaultDetection.hpp"

using namespace pose_ekf;


/** CHECK */ 
KFD_PosVelAcc::KFD_PosVelAcc() 
{
    filter =  new KalmanFilter::KF<StatePosVelAcc::SIZE>();
    position_world = Eigen::Vector3d::Zero();
    velocity_world = Eigen::Vector3d::Zero();
    
    std::cout << " Instanciando KFD_PosVelAcc " << std::endl; 
}

KFD_PosVelAcc::~KFD_PosVelAcc()
{
    delete filter; 
    
}

void KFD_PosVelAcc::predict(Eigen::Vector3d acc_intertial, double dt, Eigen::Matrix<double, StatePosVelAcc::SIZE, StatePosVelAcc::SIZE> process_noise)
{
 	  
	  /** Inertial Navigation */ 
	  
	  //gravity correction 
	  Eigen::Vector3d gravity_world = Eigen::Vector3d::Zero(); 
	  gravity_world.z() = 9.871; 
	  
	  //graviy in inertial frame  
	  Eigen::Vector3d gravity_inertial =  R_input_to_world.inverse() * gravity_world; 

	  //gravity correction 
	  acc_intertial = acc_intertial - gravity_inertial; 
	  
	  //inertial navigation 
	  velocity_world = velocity_world +   acc_intertial * dt; 
	  position_world = position_world  + R_input_to_world * velocity_world * dt; 
	  
	  /** Kalman Filter */ 
	  
	  //sets the transition matrix 
	  Eigen::Matrix<double, StatePosVelAcc::SIZE, StatePosVelAcc::SIZE> F;
	  F.setZero();
	  
// 	  F.block<3,3>(0,3) =  Eigen::Matrix3d::Identity(); 
// 	  F.block<3,3>(3,6) =  Eigen::Matrix3d( R_input_to_world ); 
	  F.block<3,3>(0,3) =  Eigen::Matrix3d( R_input_to_world );
	  F.block<3,3>(3,6) =  Eigen::Matrix3d::Identity(); 
	  
	  F(6,6) = - 0.00136142583242962/dt;
	  F(7,7) = - 0.00143792179010407/dt;
	  F(8,8) = - 0.000155846795306766/dt;

	 //std::cout << "F \n" << F << std::endl; 
	/* std::cout 
	      << "0 0 0 1 0 0 0 0 0 \n" 
	      << "0 0 0 0 1 0 0 0 0 \n"  
	      << "0 0 0 0 0 1 0 0 0 \n"  
	      << "0 0 0 0 0 0 R R R \n" 
	      << "0 0 0 0 0 0 R R R \n"  
	      << "0 0 0 0 0 0 R R R \n"  
	      << "0 0 0 0 0 0 0 0 0 \n" 
	      << "0 0 0 0 0 0 0 0 0 \n" 
	      << "0 0 0 0 0 0 0 0 0 \n" 
	      <<std::endl;*/
	  
	  //updates the Kalman Filter 
	  filter->predictionDiscrete( F, process_noise, dt); 

	  //get the updated values 
	  x.vector()=filter->x; 
	  
	 
	  
}

void KFD_PosVelAcc::setRotation(Eigen::Quaterniond R)
{
    R_input_to_world = R; 
}

bool KFD_PosVelAcc::positionObservation(Eigen::Vector3d position, Eigen::Matrix3d covariance, float reject_position_threshol)
{
  
      Eigen::Vector3d  position_diference = position_world - position; 

      Eigen::Matrix<double, _POS_MEASUREMENT_SIZE, StatePosVelAcc::SIZE> H; 
      H.setZero(); 
      H.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); 
      
      //position_world = position;
      bool reject =  filter->correctionChiSquare<_POS_MEASUREMENT_SIZE,_POS_DEGREE_OF_FREEDOM>( position_diference,  covariance,H, reject_position_threshol );
      
      x.vector() = filter->x;
      
       correct_state();
      
      return reject; 
      
}

bool KFD_PosVelAcc::velocityObservation(Eigen::Vector3d velocity, Eigen::Matrix3d covariance, float reject_velocity_threshol)
{
  
      Eigen::Vector3d  velocity_diference = velocity_world - velocity; 
      //std::cout << " Vel dif " << velocity_diference << std::endl; 
      Eigen::Matrix<double, _VEL_MEASUREMENT_SIZE, StatePosVelAcc::SIZE> H; 
      H.setZero(); 
      H.block<3,3>(0,3) = Eigen::Matrix3d::Identity(); 
      //position_world = position;
      bool reject =  filter->correctionChiSquare<_VEL_MEASUREMENT_SIZE,_VEL_DEGREE_OF_FREEDOM>( velocity_diference,  covariance, H, reject_velocity_threshol );
      x.vector() = filter->x;
      
      correct_state();
      
      return reject; 
      
}

bool KFD_PosVelAcc::positionZObservation(double z, double error, double rejection_threshold) 
{
    Eigen::Matrix<double,1,1>  dif;
    dif(0,0) = position_world(2) - z; 
    Eigen::Matrix<double,1 ,1> cov;
    cov(0,0) = error; 
    Eigen::Matrix<double, 1, StatePosVelAcc::SIZE> H; 
    H.setZero(); 
    H(0,2) = 1; 
    bool reject =  filter->correctionChiSquare<1,1>( dif,  cov, H, rejection_threshold );
    x.vector() = filter->x;
      
    correct_state();
      
    return reject; 

}
void KFD_PosVelAcc::setPosition( Eigen::Vector3d position, Eigen::Matrix3d covariance )
{
  
      //set the inertial navigation for the initial position 
      position_world = position; 
      
      //The initial covariance in position 
      filter->P.block<3,3>(0,0) = covariance ;
      
      x.pos_world() = Eigen::Vector3d::Zero(); 
      filter->x = x.vector();

}

void KFD_PosVelAcc::correct_state()
{

    position_world = position_world  -  x.pos_world(); 
    
    velocity_world = velocity_world -  x.vel_world(); 
    
    //once the state is corrected I can set the erro estimate to 0 
    x.pos_world() = Eigen::Vector3d::Zero(); 
    x.vel_world() = Eigen::Vector3d::Zero(); 
    
    filter->x = x.vector(); 
    
}

void KFD_PosVelAcc::copyState ( const KFD_PosVelAcc& kfd )
{
  
    x = kfd.x; 
    filter->x = kfd.filter->x;
    filter->P = kfd.filter->P;
    position_world = kfd.position_world; 
    velocity_world = kfd.velocity_world;
    R_input_to_world = kfd.R_input_to_world;
    
}


Eigen::Vector3d KFD_PosVelAcc::getPosition()
{
    return (position_world - x.pos_world()); 
}
	
Eigen::Vector3d KFD_PosVelAcc::getVelocity()
{
    return (velocity_world - x.vel_world()); 
}

Eigen::Matrix3d KFD_PosVelAcc::getPositionCovariance()
{
    return filter->P.block<3,3>(0,0);
}

Eigen::Matrix3d KFD_PosVelAcc::getVelocityCovariance()
{
    return filter->P.block<3,3>(3,3);
}

Eigen::Matrix3d KFD_PosVelAcc::getAccCovariance()
{
    return filter->P.block<3,3>(6,6);
}

Eigen::Vector3d KFD_PosVelAcc::getAccBias()
{
    return x.acc_inertial();
}


/** configurarion hook */ 
void KFD_PosVelAcc::init(const Eigen::Matrix<double, StatePosVelAcc::SIZE, StatePosVelAcc::SIZE> &P, const Eigen::Matrix<double,StatePosVelAcc::SIZE,1> &x)
{
    Q.setZero(); 
    this->x.vector() = x; 
    filter->x = x; 
    filter->P = P; 

}

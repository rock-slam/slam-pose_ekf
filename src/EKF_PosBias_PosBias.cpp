#include "EKF_PosBias_PosBias.hpp"
#include <Eigen/LU> 
#include <math.h>

using namespace poseEstimation;


/** CHECK */ 
EkfPosBias::EkfPosBias() 
   
{
  filter =  new ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE>;

}

EkfPosBias::~EkfPosBias(){
  delete filter; 
}
/** update the filter */ 
void EkfPosBias::update( Eigen::Vector3d v_w, double d_t )
{	
	
	//calculates the rotation from world to world frame corrected by the bias 
	Eigen::Quaterniond R_w2wb;
	    R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 

	//sets the transition matrix 
	Eigen::Matrix<double, State::SIZE, 1> f;
	    f.start<3>()=x.xi()+R_w2wb*v_w*d_t;
	    f.end<1>()=x.yaw();

	//sets the Jacobian of the state transition matrix 
	Eigen::Matrix<double, State::SIZE, State::SIZE> J_F
	   = jacobianF(v_w, d_t);
	   

	//updates the Kalman Filter 
	filter->update(f,J_F, Q ); 
	
	//get the updated values 
	x.vector()=filter->x; 
	P=filter->P; 
	
	
}

void EkfPosBias::correctionPos( Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p )
{
    
 	//jacobian of the observation function 
	Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE> J_H 
	    = jacobianH (); 
    
      
	//observation function (the observation is linear so the h matrix can be defined as
	Eigen::Matrix<double, MEASUREMENT_SIZE, 1> h
	    = J_H*x.vector(); 
	
	//correct the state 
	filter->correction( p,h, J_H, R); 
      
	//get the corrected values 
	x.vector()=filter->x; 
	P=filter->P; 

      std::cout<<" yaw 2= "<<x.yaw()(0,0)*180/3.14<<std::endl;
 
}

/** Calculates the Jacobian of the transition matrix */ 
Eigen::Matrix<double, State::SIZE, State::SIZE> EkfPosBias::jacobianF ( Eigen::Vector3d v_w, double d_t ){
    
    //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
      dR_z<<-sin(x.yaw()(0,0)), -cos(x.yaw()(0,0)), 0,cos(x.yaw()(0,0)),-sin(x.yaw()(0,0)),0,0,0,0; 
    
    //jacobian 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F; 
      J_F.setIdentity(); 
      J_F.block<3,1>(0,3)
	  = dR_z*v_w*d_t;
	  
    return J_F;
  	
}


/**jacobian observation model*/ 
Eigen::Matrix<double, EkfPosBias::MEASUREMENT_SIZE, State::SIZE>  EkfPosBias::jacobianH ( ){

  Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE>  J_H;
    J_H.setIdentity();
    
  return J_H; 

}

/** calculates the process noise in world frame*/ 
void EkfPosBias::processNoise(Eigen::Matrix<double, State::SIZE, State::SIZE> Q){ 
  
    this->Q = Q; 
    
    // Rotate from world to world with bias 
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
	R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 

    Eigen::Matrix3d R_w2wb_ = Eigen::Matrix3d(R_w2wb);    
    
    this->Q.block<3,3>(0,0) = R_w2wb_*this->Q.block<3,3>(0,0) *R_w2wb_.transpose();
    

} 

/** calculates the measurement noise */ 
void EkfPosBias::measurementNoise(Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R){
    this->R=R; 
}

/** configurarion hook */ 
void EkfPosBias::configure_hook(Eigen::Matrix<double, State::SIZE, State::SIZE> P, Eigen::Matrix<double,State::SIZE,1> x){

    Q.setZero(); 
    R.setZero(); 
    
   this->P=P;
   this->x.vector()=x; 
   filter->x=x; 
   filter->P=P;
    

}







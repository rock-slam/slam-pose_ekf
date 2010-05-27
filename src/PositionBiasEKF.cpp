#include "PositionBiasEKF.hpp"
#include <Eigen/LU> 
#include <math.h>

using namespace PositionBiasEKF;


/** CHECK */ 
PosBiasEKF::PosBiasEKF() 
   
{
  filter =  new ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE>;
  goodInitialPosition=false;
}

PosBiasEKF::~PosBiasEKF(){
  delete filter; 
}
/** update the filter */ 
void PosBiasEKF::update( Eigen::Vector3d v_w, double d_t )
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
	filter->update(f,J_F, Q_wb ); 
	
	//get the updated values 
	x.vector()=filter->x; 
	P=filter->P; 
	
	
}

void PosBiasEKF::correctionPos( Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p )
{
    
    //decides if there is or not a bad data 
    if(!rejectData(p)){
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
}

/** Calculates the Jacobian of the transition matrix */ 
Eigen::Matrix<double, State::SIZE, State::SIZE> PosBiasEKF::jacobianF ( Eigen::Vector3d v_w, double d_t ){
    
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
Eigen::Matrix<double, PosBiasEKF::MEASUREMENT_SIZE, State::SIZE>  PosBiasEKF::jacobianH ( ){

  Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE>  J_H;
    J_H.setZero();
    J_H.corner<3,3>(Eigen::TopLeft)= Eigen::Matrix3d::Identity();
 
  return J_H; 

}

/** calculates the process noise */ 
void PosBiasEKF::processNoise(Eigen::Matrix<double, 3, 3> Q_w){ 

    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
      R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 
  
    
    Eigen::Matrix3d R_w2wb_ = Eigen::Matrix3d(R_w2wb);
    
  
    Q_wb.block<3,3>(0,0)
      =R_w2wb_*Q_w *R_w2wb_.transpose();
    Q_wb(3,3)
	  =0.0001*M_PI/180.0;
} 

/** calculates the measurement noise */ 
void PosBiasEKF::measurementNoise(Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R_g){
    R=R_g; 
}

/** configurarion hook */ 
void PosBiasEKF::configure_hook(){

  goodInitialPosition=false;//true;

    Q_wb.setZero(); 
    R.setZero(); 
    
    filter->P.setIdentity();
    filter->P *= 1e10;
    
    filter->P(3,3)=50*M_PI/180.0; 
    filter->x.setZero();
    
    //get the initial values
    x.vector()=filter->x; 
    P=filter->P;
    

}

/** reject bad measurement data ? */ 
bool PosBiasEKF::rejectData(Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p){ 
	
    //turned off 
   // return false; 
    
    // Compute the distance between the estimated position and the GPS-provided
    // position
    double gps_to_estimate_x = p(0, 0) - x.xi().x();
    double gps_to_estimate_y = p(1, 0) - x.xi().y();
    double distance = sqrt(gps_to_estimate_x * gps_to_estimate_x + gps_to_estimate_y * gps_to_estimate_y);
   
    // Get an approximation of the error ellipses (just take the longest axis
    // length)
    double pose_var_r = sqrt(std::max(P(0,0), P(1,1))); 
    double gps_var_r  = sqrt(std::max(R(0,0), R(1,1)));
   
    if(!goodInitialPosition && gps_var_r < 0.1){ // was 0.1 
        goodInitialPosition =true; 
        std::cout << "got a good initial position with var=" << gps_var_r << std::endl; 
    }

    if(goodInitialPosition)
    {
        if (distance < (pose_var_r+gps_var_r) || gps_var_r <0.15){
            return false;
        }else{
            std::cout << "GPS position does not agree with estimated position, rejecting GPS data" << std::endl; 

            return true; 
        }
    }
    else
    {
        std::cout << "no good initial position yet, rejecting GPS data" << std::endl; 
        return true;
    }
}






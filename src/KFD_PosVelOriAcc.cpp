#include "KFD_PosVelOriAcc.hpp"
#include <Eigen/LU> 
#include <math.h>
#include <fault_detection/FaultDetection.hpp>

using namespace pose_estimator;


/** CHECK */ 
KFD_PosVelOriAcc::KFD_PosVelOriAcc() 
{
    filter =  new KalmanFilter::KF<StatePosVelOriAcc::SIZE>();
    position_world = Eigen::Vector3d::Zero();
    velocity_inertial = Eigen::Vector3d::Zero();
    
    std::cout << " Instanciando KFD_PosVelOriAcc " << std::endl; 
}

KFD_PosVelOriAcc::~KFD_PosVelOriAcc()
{
    delete filter; 
    
}

void KFD_PosVelOriAcc::predict(Eigen::Vector3d acc_intertial, Eigen::Vector3d angular_velocity, double dt, Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> process_noise)
{
  
	  /** Angular change calculation  */
	  //Quartenion differential equations (Notes on Quaternions - Simo Sarkka) 
	  //TODO SLERP INTERPOLATION WILL BLOW FOR ROTATION SPEED OVER 2PI per second 
	  Eigen::Quaterniond Q_angular_velocity = Eigen::AngleAxisd(angular_velocity[0], Eigen::Vector3d::UnitX()) * 
				      Eigen::AngleAxisd(angular_velocity[1], Eigen::Vector3d::UnitY()) * 
				      Eigen::AngleAxisd(angular_velocity[2], Eigen::Vector3d::UnitZ());
  
	  
	  
	  Eigen::Quaterniond ang_vel_10ms = Eigen::Quaterniond::Identity().slerp(dt, Q_angular_velocity);
	  R_inertial_2_world = R_inertial_2_world * ang_vel_10ms; 
	  
	  
	  /** Inertial Navigation */ 
	  
	  //gravity correction 
	  Eigen::Vector3d gravity_world = Eigen::Vector3d::Zero(); 
	  gravity_world.z() = 9.871; 
	  
	  //graviy in inertial frame  
	  Eigen::Vector3d gravity_inertial = ( angularCorrection() * R_inertial_2_world ).inverse() * gravity_world; 

	  //gravity correction 
	  acc_intertial = acc_intertial - gravity_inertial; 
	  
	  //inertial navigation 
	  position_world = position_world 
	      + R_inertial_2_world * velocity_inertial * dt 
	      + R_inertial_2_world * acc_intertial * dt * dt / 2; 
	  
	  velocity_inertial = velocity_inertial + acc_intertial * dt; 
	    
	  /** Kalman Filter */ 
	  //update the error 
	  //processNoise( process_noise );  
	  //predict(velocity_inertial, dt, Eigen::Matrix3d( R_inertial_2_world ), angular_velocity ); 
	  
	  //sets the transition matrix 
	  Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> F;
	  F.setZero(); 
	  F.block<3,3>(0,3) =  Eigen::Matrix3d( R_inertial_2_world ); 
	  F.block<3,3>(0,9)  << 0, -velocity_inertial(2), velocity_inertial(1),
			      velocity_inertial(2), 0, -velocity_inertial(0),
			      -velocity_inertial(1), velocity_inertial(0), 0 ; 
	  F.block<3,3>(3,6) = Eigen::Matrix3d::Identity(); 
	  F.block<3,3>(9,9) << 0, angular_velocity[2], -angular_velocity[1],
			      -angular_velocity[2], 0, angular_velocity[0],
			      angular_velocity[1], -angular_velocity[0], 0; 
	  F.block<3,3>(9,12) =  Eigen::Matrix3d( R_inertial_2_world );
	
	// std::cout << "F \n" << F << std::endl; 
	/* std::cout 
	      << "0 0 0 R R R 0 0 0 0 -vel_z vel_y 0 0 0\n" 
	      << "0 0 0 R R R 0 0 0 vel_z 0 -vel_x 0 0 0\n"  
	      << "0 0 0 R R R 0 0 0 -vel_y vel_x 0 0 0 0\n"  
	      << "0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0\n" 
	      << "0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0\n"  
	      << "0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0\n"  
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 wz -wy -R -R -R\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 -wz 0 wx -R -R -R\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 wy -wx 0 -R -R -R\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n" 
	      << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n"
	      <<std::endl;*/
	  
	  //updates the Kalman Filter 
	  filter->predictionDiscrete( F, process_noise, dt); 

	  //get the updated values 
	  x.vector()=filter->x; 
    
	  
}

bool KFD_PosVelOriAcc::positionObservation(Eigen::Vector3d position, Eigen::Matrix3d covariance, int reject_position_threshol)
{
  
      Eigen::Vector3d  position_diference = position_world - position; 

      Eigen::Matrix<double, _POS_MEASUREMENT_SIZE, StatePosVelOriAcc::SIZE> H; 
      H.setZero(); 
      H.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); 
      
      //position_world = position;
      bool reject =  filter->correctionChiSquare<_POS_MEASUREMENT_SIZE,_POS_DEGREE_OF_FREEDOM>( position_diference,  covariance,H, reject_position_threshol );
      
      x.vector() = filter->x;
      
      correct_state();
      
      return reject; 
      
}

bool KFD_PosVelOriAcc::orientationObservation( Eigen::Quaterniond orientation, Eigen::Matrix3d covariance, int reject_orientation_threshol)
{
  
      Eigen::Matrix<double, _ORI_MEASUREMENT_SIZE, StatePosVelOriAcc::SIZE> H; 
      H.setZero(); 
      H.block<3,3>(0,9) = Eigen::Matrix3d::Identity();
      
      Eigen::Quaterniond dif = orientation * R_inertial_2_world.inverse() ;
      Eigen::Vector3d _dif = Eigen::Matrix3d(dif).eulerAngles(0,1,2);
	
      bool reject = filter->correctionChiSquare<_ORI_MEASUREMENT_SIZE, _ORI_DEGREE_OF_FREEDOM>( _dif,  covariance , H, reject_orientation_threshol );
      
      x.vector() = filter->x;
      
      return reject; 

}

void KFD_PosVelOriAcc::setPosition( Eigen::Vector3d position, Eigen::Matrix3d covariance )
{
  
      //set the inertial navigation for the initial position 
      position_world = position; 
      
      //The initial covariance in position 
      filter->P.block<3,3>(0,0) = covariance ;
      
      x.pos_world() = Eigen::Vector3d::Zero(); 
      filter->x = x.vector();

}

void KFD_PosVelOriAcc::setOrientation( Eigen::Quaterniond orientation, Eigen::Matrix3d covariance )
{
  
      std::cout << " Set Orientation " << std::endl; 
      //sets again the initial orientation in case the system moved while waiting for a global position
      R_inertial_2_world = orientation; 
      
      //the variance of the xsens is 0.5 for pitch and roll and 1 heading STATIC 
      filter->P.block<3,3>(9,9) = covariance; 
      
       x.or_w() = Eigen::Vector3d::Zero(); 
      filter->x = x.vector(); 
      
}

void KFD_PosVelOriAcc::correct_state()
{
    
    R_inertial_2_world = angularCorrection() * R_inertial_2_world;
    
    position_world = position_world  -  x.pos_world(); 
    
    velocity_inertial = velocity_inertial -  x.vel_inertial(); 
    
    //once the state is corrected I can set the erro estimate to 0 
    x.pos_world() = Eigen::Vector3d::Zero(); 
    x.vel_inertial() = Eigen::Vector3d::Zero(); 
    x.or_w() = Eigen::Vector3d::Zero(); 

    filter->x = x.vector(); 
    
}

void KFD_PosVelOriAcc::copyState ( const KFD_PosVelOriAcc& kfd )
{
  
    x = kfd.x; 
    filter->x = kfd.filter->x;
    filter->P = kfd.filter->P;
    position_world = kfd.position_world; 
    velocity_inertial = kfd.velocity_inertial;
    R_inertial_2_world = kfd.R_inertial_2_world; 
    
}


Eigen::Vector3d KFD_PosVelOriAcc::getPosition()
{
    return (position_world - x.pos_world()); 
}
	
Eigen::Vector3d KFD_PosVelOriAcc::getVelocity()
{
    return (velocity_inertial - x.vel_inertial()); 
}

Eigen::Quaterniond KFD_PosVelOriAcc::getOrientation()
{  
    return (angularCorrection() * R_inertial_2_world) * tf.R_body_2_inertial;
}

Eigen::Matrix3d KFD_PosVelOriAcc::getPositionCovariance()
{
    return filter->P.block<3,3>(0,0);
}

Eigen::Matrix3d KFD_PosVelOriAcc::getVelocityCovariance()
{
    return filter->P.block<3,3>(3,3);
}

Eigen::Matrix3d KFD_PosVelOriAcc::getOrientationCovariance()
{
    return filter->P.block<3,3>(9,9);
}

Eigen::Quaterniond KFD_PosVelOriAcc::angularCorrection()
{    
    Eigen::Quaterniond correction;
    correction = Eigen::AngleAxisd(x.or_w()[0], Eigen::Vector3d::UnitX() ) * 
	Eigen::AngleAxisd(x.or_w()[1], Eigen::Vector3d::UnitY() ) * 
	Eigen::AngleAxisd(x.or_w()[2], Eigen::Vector3d::UnitZ() ); 
    return correction;
}


/** configurarion hook */ 
void KFD_PosVelOriAcc::init(const Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> &P, const Eigen::Matrix<double,StatePosVelOriAcc::SIZE,1> &x)
{
    Q.setZero(); 
    this->x.vector() = x; 
    filter->x = x; 
    filter->P = P; 

}

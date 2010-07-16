#include "EKFPosYawBias.hpp"
#include <Eigen/LU> 
#include <math.h>
#include <fault_detection/FaultDetection.hpp>

using namespace pose_estimator;


/** CHECK */ 
EKFPosYawBias::EKFPosYawBias() 
{
    filter_gps =  new ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE_GPS>;
    filter_scan_match =  new ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE_SCAN_MATCH>;
    chi_square = new fault_detection::ChiSquared();
}

EKFPosYawBias::~EKFPosYawBias()
{
    delete filter_gps; 
    delete filter_scan_match; 
    delete chi_square;
}

/** update the filter */ 
void EKFPosYawBias::predict( const Eigen::Vector3d &translation_world )
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

	
    /** EKF **/ 
    //sets the current state to the filter 
    filter_gps->x = x.vector(); 
    filter_gps->P = P;
    
    //updates the Kalman Filter 
    filter_gps->prediction( f, J_F, Q ); 

    //get the updated values 
    x.vector()=filter_gps->x; 
    P=filter_gps->P; 

}

void EKFPosYawBias::correctionGps(const Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, 1> &p, Eigen::Transform3d C_w2gw_without_bias, Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, MEASUREMENT_SIZE_GPS> R_GPS  )
{

    Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, State::SIZE> J_H;
    J_H.setIdentity();
    
    Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, 1> h;
    h = J_H * x.vector();

/*    //jacobian of the observation function 
    Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, State::SIZE> J_H 
	= jacobianGpsObservation ( C_w2gw_without_bias );
	
    Eigen::Transform3d R_w2wb;
    R_w2wb=Eigen::AngleAxisd( x.yaw()(0,0), Eigen::Vector3d::UnitZ() ); 
    
    Eigen::Transform3d R_w2gw( R_w2wb * C_w2gw_without_bias * R_w2wb.inverse() );
	
    //observation function
    Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, 1> h;
    h = R_w2gw * ( Eigen::Vector3d ( x.vector().start<3>() ) ); 
*/
    /** EKF  */
    //sets the current state to the filter 
    filter_gps->x = x.vector(); 
    filter_gps->P = P; 
    
   //innovation steps
    filter_gps->innovation( p, h, J_H, R_GPS ); 

    //test to reject data 
    reject_GPS_observation=chi_square->rejectData2D( filter_gps->y.start<2>(), filter_gps->S.block<2,2>(0,0), chi_square->THRESHOLD_2D_99 );
    //reject_GPS_observation=chi_square->rejectData3D( filter_gps->y.start<3>(), filter_gps->S.block<3,3>(0,0), chi_square->THRESHOLD_3D_95 );
  
    if(!reject_GPS_observation)
    {
    
	//Kalman Gain
	filter_gps->gain( J_H ); 
	
	//update teh state 
	filter_gps->update(J_H ); 

	//get the corrected values 
	x.vector() = filter_gps->x; 
	P = filter_gps->P; 

    }
    else
    { 
     
	std::cout<< " Rejected GPS data " << std::endl; 
    
    }

}

void EKFPosYawBias::correctionScanMatch(const Eigen::Matrix<double, MEASUREMENT_SIZE_SCAN_MATCH, 1> &p, Eigen::Matrix<double, MEASUREMENT_SIZE_SCAN_MATCH, MEASUREMENT_SIZE_SCAN_MATCH> R_scan_match  )
{
    //jacobian of the observation function 
    Eigen::Matrix<double, MEASUREMENT_SIZE_SCAN_MATCH, State::SIZE> J_H 
	= jacobianScanMatchObservation (); 

    //observation function (the observation is linear so the h matrix can be defined as
    Eigen::Matrix<double, MEASUREMENT_SIZE_SCAN_MATCH, 1> h
	= J_H*x.vector(); 


    /** EKF  */
    //sets the current state to the filter 
    filter_scan_match->x = x.vector(); 
    filter_scan_match->P = P; 
    
   //innovation steps
    filter_scan_match->innovation( p, h, J_H, R_scan_match ); 

    //test to reject data 
    reject_scan_match_observation=chi_square->rejectData2D( filter_scan_match->y.start<2>(), filter_scan_match->S.block<2,2>(0,0), chi_square->THRESHOLD_2D_99 );
    //reject_scan_match_observation=chi_square->rejectData3D( filter_scan_match->y.start<3>(), filter_scan_match->S.block<3,3>(0,0), chi_square->THRESHOLD_3D_95 );

    if(!reject_scan_match_observation)
    {
	
	//Kalman Gain
	filter_scan_match->gain( J_H ); 
	
	//update teh state 
	filter_scan_match->update( J_H ); 

	//get the corrected values 
	x.vector() = filter_scan_match->x; 
	P = filter_scan_match->P; 
    
    }
    else
    { 
     
	std::cout<< " Rejected SCAN Match data " << std::endl; 
    
    }

}

/** Calculates the Jacobian of the transition matrix */ 
Eigen::Matrix<double, State::SIZE, State::SIZE> EKFPosYawBias::jacobianF( const Eigen::Vector3d &translation_world )
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

/**jacobian observation model*/ 
Eigen::Matrix<double, EKFPosYawBias::MEASUREMENT_SIZE_SCAN_MATCH, State::SIZE> EKFPosYawBias::jacobianScanMatchObservation ()
{
    Eigen::Matrix<double, MEASUREMENT_SIZE_SCAN_MATCH, State::SIZE>  J_H;
    J_H.setIdentity();

    return J_H; 
}

/**jacobian of the GPS observation*/ 
Eigen::Matrix<double, EKFPosYawBias::MEASUREMENT_SIZE_GPS, State::SIZE> EKFPosYawBias::jacobianGpsObservation (Eigen::Transform3d C_w2gw_without_bias)
{
     //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
    dR_z << -sin( x.yaw()(0,0) ), -cos( x.yaw()(0,0) ), 0,cos( x.yaw()(0,0) ),-sin( x.yaw()(0,0) ), 0, 0, 0, 0; 

    //derivate of the inverse rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z_inv;
    dR_z_inv << -sin( -x.yaw()(0,0) ), -cos( -x.yaw()(0,0) ), 0, cos( -x.yaw()(0,0) ), -sin( -x.yaw()(0,0) ), 0, 0, 0, 0; 
    
    Eigen::Transform3d R_w2wb;
    R_w2wb = Eigen::AngleAxisd( x.yaw()(0,0), Eigen::Vector3d::UnitZ() ); 
    
    Eigen::Transform3d R_w2gw( R_w2wb * C_w2gw_without_bias * R_w2wb.inverse() );

    Eigen::Matrix<double, MEASUREMENT_SIZE_GPS, State::SIZE>  J_H;
    J_H.setZero();
    
    J_H.block<3,1>(0,0) = R_w2gw * Eigen::Vector3d::UnitX();
    J_H.block<3,1>(0,1) = R_w2gw * Eigen::Vector3d::UnitY();
    J_H.block<3,1>(0,2) = R_w2gw * Eigen::Vector3d::UnitZ();   
    J_H.block<3,1>(0,3) =  
	Eigen::Transform3d( dR_z * C_w2gw_without_bias * R_w2wb.inverse() ) * Eigen::Vector3d( x.vector().start<3>() )   
	+ Eigen::Transform3d( R_w2wb * C_w2gw_without_bias * dR_z_inv ) * Eigen::Vector3d( x.vector().start<3>() );

    return J_H; 
}

/** calculates the process noise in world frame*/ 
void EKFPosYawBias::processNoise(const Eigen::Matrix<double, State::SIZE, State::SIZE> &Q)
{ 
    this->Q = Q; 

    // Rotate from world to world with bias 
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 

    Eigen::Matrix3d R_w2wb_ = Eigen::Matrix3d(R_w2wb);    

    this->Q.block<3,3>(0,0) = R_w2wb_*this->Q.block<3,3>(0,0) *R_w2wb_.transpose();
} 


/** configurarion hook */ 
void EKFPosYawBias::init(const Eigen::Matrix<double, State::SIZE, State::SIZE> &P, const Eigen::Matrix<double,State::SIZE,1> &x)
{
    Q.setZero(); 
    this->P = P;
    this->x.vector() = x; 

}


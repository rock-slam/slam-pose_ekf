#ifndef __KFD_POS_VEL_ORI_ACC_HPP__
#define __KFD_POS_VEL_ORI_ACC_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "KalmanFilter.hpp"
#include "KalmanFilterTypes.hpp"

namespace pose_estimator {
    class StatePosVelOriAcc
      {
    public:
	static const int SIZE = 15;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _pos_w;
	Eigen::Block<vector_t, 3, 1> _vel_i;
	Eigen::Block<vector_t, 3, 1> _acc_i;
	Eigen::Block<vector_t, 3, 1> _or_w;
	Eigen::Block<vector_t, 3, 1> _w_i;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StatePosVelOriAcc() :
	    _x(vector_t::Zero()), _pos_w( _x.segment<3>(0) ), _vel_i( _x.segment<3>(3) ),_acc_i(_x.segment<3>(6)), _or_w(_x.segment<3>(9)), _w_i(_x.segment<3>(12)) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& pos_world() {return _pos_w;}
	Eigen::Block<vector_t, 3, 1>& vel_inertial() {return _vel_i;}	
	Eigen::Block<vector_t, 3, 1>& acc_inertial() {return _acc_i;}
	Eigen::Block<vector_t, 3, 1>& or_w() {return _or_w;}
	Eigen::Block<vector_t, 3, 1>& angular_velocity_inertial() {return _w_i;}
    };

    class KFD_PosVelOriAcc
    {
    public:
      	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	static const unsigned int _POS_MEASUREMENT_SIZE = 3; 
	static const unsigned int _POS_DEGREE_OF_FREEDOM = 3;
	static const unsigned int _ORI_MEASUREMENT_SIZE = 3; 
	static const unsigned int _ORI_DEGREE_OF_FREEDOM = 3; 
	
	/** state estimate */
	StatePosVelOriAcc x;
	
	/** covariance matrix */
	//Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> P;
	
    public:
	/** estimated position, velocityand orientation */ 
	Eigen::Vector3d velocity_inertial; 
	Eigen::Vector3d position_world; 
	Eigen::Quaterniond R_inertial_2_world; 
	

	
	/** Instance of the Extended Kalman filter*/
	KalmanFilter::KF<StatePosVelOriAcc::SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> Q;
	
	
    public:
	KFD_PosVelOriAcc();
	~KFD_PosVelOriAcc();
  
	/** Copy the Kalman Filter State from KFD*/ 
	void copyState ( const KFD_PosVelOriAcc& kfd ); 
	
	/** A 3D position observation */ 
	bool positionObservation( Eigen::Vector3d position, Eigen::Matrix3d covariance, int reject_position_threshol); 
	
	/** A 3D orientation observation */ 
	bool orientationObservation( Eigen::Quaterniond orientation, Eigen::Matrix3d covariance, int reject_orientation_threshol);
	
	/** set the position */  
	void setPosition( Eigen::Vector3d position, Eigen::Matrix3d covariance ); 

	/** set orientation */  
	void setOrientation( Eigen::Quaterniond orientation, Eigen::Matrix3d covariance );
	
	/** The current state estimation will be corrected by the Kalman Filter error calculation 
	than the states in the filter will be reseted. */ 
	void correct_state();
	
	/** input acc and angular velocity data to the filter updating the kalman filter and pose estimation */ 
	void predict(Eigen::Vector3d acc_intertial, Eigen::Vector3d angular_velocity, double dt,Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> process_noise);
	
	/** gets the estimated position */ 
	Eigen::Vector3d getPosition(); 
	
	/** gets the estimated velocity */ 
	Eigen::Vector3d getVelocity();
	
	/** gets the estimated pose */ 
	Eigen::Quaterniond getOrientationInertial2World(); 
	
	/** get the Position covariance */
	Eigen::Matrix3d getPositionCovariance(); 
	
	/** get the Velocity covariance */
	Eigen::Matrix3d getVelocityCovariance();
	
	/** get the Orientation covariance */
	Eigen::Matrix3d getOrientationCovariance();
	
	/** get  orientation correction calculated by the kalman filter */ 
	Eigen::Quaterniond angularCorrection();
      
	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> &P, const Eigen::Matrix<double,StatePosVelOriAcc::SIZE,1> &x); 


    
	
    };
}

#endif

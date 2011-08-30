#ifndef __KFD_POS_VEL_ACC_HPP__
#define __KFD_POS_VEL_ACC_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "KalmanFilter.hpp"

#include "FaultDetection.hpp"



namespace pose_ekf {
    class StatePosVelAcc
      {
    public:
	static const int SIZE = 9;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _pos_w;
	Eigen::Block<vector_t, 3, 1> _vel_i;
	Eigen::Block<vector_t, 3, 1> _acc_i;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StatePosVelAcc() :
	    _x(vector_t::Zero()), _pos_w( _x.segment<3>(0) ), _vel_i( _x.segment<3>(3) ),_acc_i(_x.segment<3>(6)) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& pos_world() {return _pos_w;}
	Eigen::Block<vector_t, 3, 1>& vel_world() {return _vel_i;}	
	Eigen::Block<vector_t, 3, 1>& acc_inertial() {return _acc_i;}

    };

    class KFD_PosVelAcc
    {
    public:
      	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	static const unsigned int _POS_MEASUREMENT_SIZE = 3; 
	static const unsigned int _VEL_MEASUREMENT_SIZE = 3; 
	static const unsigned int _POS_DEGREE_OF_FREEDOM = 3;
	static const unsigned int _VEL_DEGREE_OF_FREEDOM = 3;

	
	/** state estimate */
	StatePosVelAcc x;
	
    public:
	/** estimated position, velocity*/ 
	Eigen::Vector3d velocity_world; 
	Eigen::Vector3d position_world; 
	

	/** Instance of the Extended Kalman filter*/
	KalmanFilter::KF<StatePosVelAcc::SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, StatePosVelAcc::SIZE, StatePosVelAcc::SIZE> Q;
	
	Eigen::Quaterniond R_input_to_world; 
	
    public:
	KFD_PosVelAcc();
	~KFD_PosVelAcc();
  
	/** Copy the Kalman Filter State from KFD*/ 
	void copyState ( const KFD_PosVelAcc& kfd ); 
	
	/** A 3D position observation */ 
	bool positionObservation( Eigen::Vector3d position, Eigen::Matrix3d covariance, float reject_position_threshol); 
	
	/** A 3D velocity observation */ 
	bool velocityObservation(Eigen::Vector3d velocity, Eigen::Matrix3d covariance, float reject_velocity_threshol);
	
	/** A z position observation */
	bool positionZObservation(double z, double error, double rejection_threshold) ;
	
	/** set the position */  
	void setPosition( Eigen::Vector3d position, Eigen::Matrix3d covariance ); 

	/** The current state estimation will be corrected by the Kalman Filter error calculation 
	than the states in the filter will be reseted. */ 
	void correct_state();
	
	/** input acc and angular velocity data to the filter updating the kalman filter and pose estimation */ 
	void predict(Eigen::Vector3d acc, double dt, Eigen::Matrix<double, StatePosVelAcc::SIZE, StatePosVelAcc::SIZE> process_noise);
	
	void setRotation(Eigen::Quaterniond R); 
	
	Eigen::Quaterniond getRotation(){ return R_input_to_world; }
	
	/** gets the estimated position */ 
	Eigen::Vector3d getPosition(); 
	
	/** gets the estimated velocity */ 
	Eigen::Vector3d getVelocity();
	
	Eigen::Vector3d getAccBias(); 
	
	/** get the Position covariance */
	Eigen::Matrix3d getPositionCovariance(); 
	
	/** get the Velocity covariance */
	Eigen::Matrix3d getVelocityCovariance();
	
	/** get the Acceleration  covariance */
	Eigen::Matrix3d getAccCovariance();
	
	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, StatePosVelAcc::SIZE, StatePosVelAcc::SIZE> &P, const Eigen::Matrix<double,StatePosVelAcc::SIZE,1> &x); 


    
	
    };
}

#endif

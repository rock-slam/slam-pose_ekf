#ifndef __KFD_POS_VEL_HPP__
#define __KFD_POS_VEL_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "KalmanFilter.hpp"

#include "FaultDetection.hpp"

namespace pose_ekf {
    class StatePosVel
      {
    public:
	static const int SIZE = 6;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _pos_w;
	Eigen::Block<vector_t, 3, 1> _vel_i;
	

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StatePosVel() :
	    _x(vector_t::Zero()), _pos_w( _x.segment<3>(0) ), _vel_i( _x.segment<3>(3) ) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& pos_world() {return _pos_w;}
	Eigen::Block<vector_t, 3, 1>& vel_body() {return _vel_i;}	
	

    };

    class KFD_PosVel
    {
    public:
      	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	static const unsigned int _POS_MEASUREMENT_SIZE = 3; 
	static const unsigned int _VEL_MEASUREMENT_SIZE = 3; 
	static const unsigned int _POS_DEGREE_OF_FREEDOM = 3;
	static const unsigned int _VEL_DEGREE_OF_FREEDOM = 3;

	
	/** state estimate */
	StatePosVel x;
	
    public:

	/** Instance of the Extended Kalman filter*/
	KalmanFilter::KF<StatePosVel::SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, StatePosVel::SIZE, StatePosVel::SIZE> Q;
	
    public:
	KFD_PosVel();
	~KFD_PosVel();
  
	/** Copy the Kalman Filter State from KFD*/ 
	void copyState ( const KFD_PosVel& kfd ); 
	
	/** A 3D position observation */ 
	bool positionObservation( Eigen::Vector3d position, Eigen::Matrix3d covariance, float reject_position_threshol); 
	
	/** A 3D velocity observation */ 
	bool velocityObservation(Eigen::Vector3d velocity, Eigen::Matrix3d covariance, float reject_velocity_threshol);
	
	/** A z position observation */
	bool positionZObservation(double z, double error, double rejection_threshold) ;
	
	/** set the position */  
	void setPosition( Eigen::Vector3d position, Eigen::Matrix3d covariance ); 

	/** prediction step of the kalman filter */ 
	void predict(Eigen::Quaterniond R_body_2_world, double dt, Eigen::Matrix<double, StatePosVel::SIZE, StatePosVel::SIZE> process_noise);
	
	/** gets the estimated position */ 
	Eigen::Vector3d getPosition(); 
	
	/** gets the estimated velocity */ 
	Eigen::Vector3d getVelocity();
	
	/** get the Position covariance */
	Eigen::Matrix3d getPositionCovariance(); 
	
	/** get the Velocity covariance */
	Eigen::Matrix3d getVelocityCovariance();
	
	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, StatePosVel::SIZE, StatePosVel::SIZE> &P, const Eigen::Matrix<double,StatePosVel::SIZE,1> &x); 
    };
}

#endif

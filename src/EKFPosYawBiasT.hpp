#ifndef __EKF_POSITIONYAWBIAS_HPP__
#define __EKF_POSITIONYAWBIAS_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ExtendedKalmanFilter.hpp"



namespace pose_estimator {
    class StatePosYawBias
    {
    public:
	static const int SIZE = 4;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _xi;
	Eigen::Block<vector_t, 1, 1> _yaw;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StatePosYawBias() :
	    _x(vector_t::Zero()), _xi( _x.segment<3>(0) ),_yaw(_x.segment<1>(3)) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& xi() {return _xi;}	
	Eigen::Block<vector_t, 1, 1>& yaw() {return _yaw;}
    };

    class EKFPosYawBiasT
    {
    public:
      	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	/** state estimate */
	StatePosYawBias x;
	
    private:
	

	
	/** Instance of the Extended Kalman filter*/
	ExtendedKalmanFilter::EKF<StatePosYawBias::SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> Q;
	
	
    public:

	EKFPosYawBiasT();
	~EKFPosYawBiasT();
  
	/** Copy the Kalman Filter State from KFD*/ 
	void copyState ( const EKFPosYawBiasT& kfd ); 
	
	/** update step taking velocity in world frame without the bias correction */
	void predict(const Eigen::Vector3d &translation_world, const Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> &Q);

	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> &P, const Eigen::Matrix<double,StatePosYawBias::SIZE,1> &x); 
	
	/**gets the rotation from world to world corrected by the bias frame calculated by the filter  R_ImuWorld_2_world*/
	Eigen::Quaterniond getOrientationCorrection();
	Eigen::Matrix3d getCovariancePosition(); 
	Eigen::Vector3d getPosition();
	double getOrientationCorrectionCovariance();

	/** sets the initial position and position covariance  */
	void setInitialPosition( const Eigen::Vector3d &position, const Eigen::Matrix3d &covariance ); 
	
	/** Corrects th KF based on position observation */ 
	bool correctPosition( const Eigen::Vector3d &position, const Eigen::Matrix3d &covariance, float reject_threshold ); 

	/** Corrects th KF based on position and orientation correction observation */ 
	bool correctPositionOrientation( const Eigen::Vector4d &positionOrientation, const Eigen::Matrix4d &covariance, float reject_threshold ); 
	
      private: 
	
	/**jacobian state transition*/ 
	Eigen::Matrix<double, StatePosYawBias::SIZE, StatePosYawBias::SIZE> jacobianF( const Eigen::Vector3d &translation_world );
	
	
    };
}

#endif

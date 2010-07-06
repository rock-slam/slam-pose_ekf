#ifndef __EKF_POSITIONYAWBIAS_HPP__
#define __EKF_POSITIONYAWBIAS_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ExtendedKalmanFilter.hpp"

namespace pose_estimator {
    class State
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
	State() :
	    _x(vector_t::Zero()), _xi( _x.segment<3>(0) ),_yaw(_x.segment<1>(3)) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& xi() {return _xi;}	
	Eigen::Block<vector_t, 1, 1>& yaw() {return _yaw;}
    };

    class EKFPosYawBias
    {
    public:
	static const int INPUT_SIZE = 3;
	static const int POS_SIZE = 3;
	static const int MEASUREMENT_SIZE = 4;

	/** state estimate */
	State x;
	
	/** covariance matrix */
	Eigen::Matrix<double, State::SIZE, State::SIZE> P;
	
    private:
	/** Instance of the Extended Kalman filter*/
	ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, State::SIZE, State::SIZE> Q;
	
	/** measurement noise */ 
	Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R; 
	
    public:
	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EKFPosYawBias();
	~EKFPosYawBias();

	/** update step taking velocity in world frame without the bias correction */
	void update(const Eigen::Vector3d &v_w, double d_t );

	/** correction step, taking absolute position data */
	void correctionPos(const Eigen::Matrix<double, POS_SIZE, 1> &p, Eigen::Transform3d C_w2gw_without_bias );

	/** correction step, taking absolute position data */
	void correction(const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &p );

	/** set the process noise the frame should be world corrected by bias [velocity,  bias] */ 
	void processNoise(const Eigen::Matrix<double, State::SIZE, State::SIZE> &Q); 
	
	/** set the measurement noise in world frame [position] */ 
	void measurementNoise(const Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> &R); 

	/** set the measurement noise in world frame [position] */ 
	void measurementNoisePos(const Eigen::Matrix<double, POS_SIZE, POS_SIZE> &R); 

	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, State::SIZE, State::SIZE> &P, const Eigen::Matrix<double,State::SIZE,1> &x); 
	
	
      private: 
	/**jacobian state transition*/ 
	Eigen::Matrix<double, State::SIZE, State::SIZE> jacobianF( const Eigen::Vector3d &v_w, double d_t );
	
	/**jacobian observation model*/ 
	Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE>  jacobianH();

	/**jacobian of the GPS observation */ 
	Eigen::Matrix<double, POS_SIZE, State::SIZE>  jacobianH_GPS(Eigen::Transform3d C_w2gw_without_bias);	
	
	
    };
}

#endif

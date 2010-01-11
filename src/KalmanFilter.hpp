#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace KalmanFilter {
    class State
    {
    public:
	static const int SIZE = 6;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _xi;
	Eigen::Block<vector_t, 3, 1> _v;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	State() :
	    _x(vector_t::Zero()), _xi( _x.segment<3>(0) ), _v( _x.segment<3>(3) ) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& xi() {return _xi;}
	Eigen::Block<vector_t, 3, 1>& v() {return _v;}
    };

    class PositionKF
    {
    public:
	static const int INPUT_SIZE = 3;
	static const int MEASUREMENT_SIZE = 3;

	/** time step for measurements */
	double d_t;

	/** state estimate, at time k and k+ */
	State x_k, x_kp;
	/** covariance matrix at time k and kp */
	Eigen::Matrix<double, State::SIZE, State::SIZE> P_k, P_kp;

	/** input matrix */
	Eigen::Matrix<double, State::SIZE, INPUT_SIZE> B;

	/** state transition matrix */
	Eigen::Matrix<double, State::SIZE, State::SIZE> F;
	/** process noise covariance matrix */
	Eigen::Matrix<double, State::SIZE, State::SIZE> Q;

	/** observation model */
	Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE> H;
	/** measurement noise covariance matrix */
	Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R;

	/** these are just here for storage */
	Eigen::Quaternion<double> q;
	Eigen::Vector3d omega;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** update step taking acceleration in inertial frame as parameter */
	void update( Eigen::Vector3d a );

	/** correction step, taking absolute position data */
	void correction( Eigen::Vector3d p );
    };
}

#endif

#ifndef __EKF_POSBIAS_POSBIAS_HPP__
#define __EKF_POSBIAS_POSBIAS_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ExtendedKalmanFilter.hpp"



namespace poseEstimation {
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

    class EkfPosBias
    {
 	
    public:
	
	static const int INPUT_SIZE = 3;
	static const int MEASUREMENT_SIZE = 4;



	/** state estimate */
	State x;
	
	/** covariance matrix */
	Eigen::Matrix<double, State::SIZE, State::SIZE> P;
	
    private:
	/** Instance of the Extended Kalman filter*/
	ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, State::SIZE, State::SIZE> Q;
	
	/** measurement noise */ 
	Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R; 
	
    
    public:
	/** what is this line for ??? */ 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EkfPosBias();
	~EkfPosBias();

	/** update step taking velocity in world frame without the bias correction */
	void update( Eigen::Vector3d v_w, double d_t );

	/** correction step, taking absolute position data */
	void correctionPos(  Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p );
	

	/** calculates the process noise the frame should be world corrected by bias [velocity,  bias] */ 
	void processNoise(Eigen::Matrix<double, State::SIZE, State::SIZE> Q); 
	
	/** calculates the measurement noise in world frame [position] */ 
	void measurementNoise(Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R); 

	
	/** configurarion hook */ 
	void configure_hook(Eigen::Matrix<double, State::SIZE, State::SIZE> P, Eigen::Matrix<double,State::SIZE,1> x); 
	
	
	
      private: 
	
	/**jacobian state transition*/ 
	Eigen::Matrix<double, State::SIZE, State::SIZE> jacobianF ( Eigen::Vector3d v_w, double d_t );
	
	/**jacobian observation model*/ 
	Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE>  jacobianH ( );	


	
    };
}

#endif

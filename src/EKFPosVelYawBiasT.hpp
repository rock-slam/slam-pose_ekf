#ifndef __EKF_POSITIONVELYAWBIAS_HPP__
#define __EKF_POSITIONVELYAWBIAS_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ExtendedKalmanFilter.hpp"
#include <fault_detection/FaultDetection.hpp>


namespace pose_estimator {
    class StatePosVelYawBias
      {
    public:
	static const int SIZE = 7;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _xi;
	Eigen::Block<vector_t, 3, 1> _vi;
	Eigen::Block<vector_t, 1, 1> _yaw;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StatePosVelYawBias() :
	    _x(vector_t::Zero()), _xi( _x.segment<3>(0) ), _vi( _x.segment<3>(3) ),_yaw(_x.segment<1>(3)) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& xi() {return _xi;}
	Eigen::Block<vector_t, 3, 1>& vi() {return _vi;}	
	Eigen::Block<vector_t, 1, 1>& yaw() {return _yaw;}
    };

    class EKFPosVelYawBiasT
    {
    public:
      	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	/** state estimate */
	StatePosVelYawBias x;
	
	/** covariance matrix */
	Eigen::Matrix<double, StatePosVelYawBias::SIZE, StatePosVelYawBias::SIZE> P;
	
    private:
	
      	/**fault detection libary */ 
	fault_detection::ChiSquared* chi_square; 
	
	/** Instance of the Extended Kalman filter*/
	ExtendedKalmanFilter::EKF<StatePosVelYawBias::SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, StatePosVelYawBias::SIZE, StatePosVelYawBias::SIZE> Q;
	
	
    public:

	EKFPosVelYawBiasT();
	~EKFPosVelYawBiasT();
  
	
	/** update step taking velocity in world frame without the bias correction */
	void predict(const Eigen::Vector3d &acc_nav, double dt );

	/** set the process noise the frame should be world corrected by bias [velocity,  bias] */ 
	void processNoise(const Eigen::Matrix<double, StatePosVelYawBias::SIZE, StatePosVelYawBias::SIZE> &Q); 
	
	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, StatePosVelYawBias::SIZE, StatePosVelYawBias::SIZE> &P, const Eigen::Matrix<double,StatePosVelYawBias::SIZE,1> &x); 
	
	template < unsigned int MEASUREMENT_SIZE, unsigned int DEGREE_OF_FREEDOM >
	bool correction(const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &p, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> &R, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &h, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, StatePosVelYawBias::SIZE> &J_H, 
		      float reject_threshold  ) 
	{
	    filter->P  = P; 
	    filter->x  = x.vector(); 
	    //innovation steps
	    // innovation 
	    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y = filter->innovation<MEASUREMENT_SIZE>( p, h );

	    // innovation covariance matrix
	    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = filter->innovationCovariance<MEASUREMENT_SIZE>(J_H, R); 

	    bool reject_observation; 
	    //test to reject data 
	    if ( reject_threshold!=0 ) 
	    {
	      
		reject_observation = chi_square->rejectData<DEGREE_OF_FREEDOM>(y.start(DEGREE_OF_FREEDOM), S.block(0,0,DEGREE_OF_FREEDOM,DEGREE_OF_FREEDOM) ,reject_threshold );
		
	    }  
	    else
		reject_observation = false;
	    
	    if(!reject_observation)
	    {
	    
		//Kalman Gain
		Eigen::Matrix<double, StatePosVelYawBias::SIZE, MEASUREMENT_SIZE> K = filter->gain<MEASUREMENT_SIZE>( J_H, S );

		//update teh state 
		filter-> update<MEASUREMENT_SIZE>( J_H, K, y); 

		//get the corrected values 
		x.vector() = filter->x; 
		P = filter->P; 
		
		std::cout <<  std::endl;
		return false; 

	    }
	    else
	    { 
		
		std::cout<< " Rejected Observation " << std::endl; 
		return true; 
	    
	    }

	}
      private: 
	
	/**jacobian state transition*/ 
	Eigen::Matrix<double, StatePosVelYawBias::SIZE, StatePosVelYawBias::SIZE> jacobianF( const Eigen::Vector3d &acc_nav, double dt );
	
	
    };
}

#endif

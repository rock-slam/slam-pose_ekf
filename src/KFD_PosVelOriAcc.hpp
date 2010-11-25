#ifndef __KFD_POS_VEL_ORI_ACC_HPP__
#define __KFD_POS_VEL_ORI_ACC_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "KalmanFilter.hpp"
#include "FaultDetection.hpp"


namespace pose_ekf {
    class StatePosVelOriAcc
      {
    public:
	static const int SIZE = 12;
	typedef Eigen::Matrix<double,SIZE,1> vector_t;

    protected:
	vector_t _x; 
	Eigen::Block<vector_t, 3, 1> _pos_w;
	Eigen::Block<vector_t, 3, 1> _vel_b;
	Eigen::Block<vector_t, 3, 1> _acc_b;
	Eigen::Block<vector_t, 3, 1> _or_w;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StatePosVelOriAcc() :
	    _x(vector_t::Zero()), _pos_w( _x.segment<3>(0) ), _vel_b( _x.segment<3>(3) ),_acc_b(_x.segment<3>(6)), _or_w(_x.segment<3>(9)) {};

	vector_t& vector() {return _x;};
	Eigen::Block<vector_t, 3, 1>& pos_world() {return _pos_w;}
	Eigen::Block<vector_t, 3, 1>& vel_inertial() {return _vel_b;}	
	Eigen::Block<vector_t, 3, 1>& acc_inertial() {return _acc_b;}
	Eigen::Block<vector_t, 3, 1>& or_w() {return _or_w;}
    };

    class KFD_PosVelOriAcc
    {
    public:
      	/** ensure alignment for eigen vector types */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	/** state estimate */
	StatePosVelOriAcc x;
	
	/** covariance matrix */
	Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> P;
	
    private:
	
      	/**fault detection libary */ 
	fault_detection::ChiSquared* chi_square; 
	
	/** Instance of the Extended Kalman filter*/
	KalmanFilter::KF<StatePosVelOriAcc::SIZE>* filter;
	
	/** process noise */
	Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> Q;
	
	
    public:

	KFD_PosVelOriAcc();
	~KFD_PosVelOriAcc();
  
	
	/** update step taking velocity in world frame without the bias correction */
	void predict(const Eigen::Vector3d &acc_nav, double dt, Eigen::Matrix3d R_iertial_2_world );

	/** set the process noise the frame should be world corrected by bias [velocity,  bias] */ 
	void processNoise(const Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> &Q); 
	
	/** set the inital values for state x and covariance P */
	void init(const Eigen::Matrix<double, StatePosVelOriAcc::SIZE, StatePosVelOriAcc::SIZE> &P, const Eigen::Matrix<double,StatePosVelOriAcc::SIZE,1> &x); 
	
	template < unsigned int MEASUREMENT_SIZE, unsigned int DEGREE_OF_FREEDOM >
	bool correction(const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &p, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> &R, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, StatePosVelOriAcc::SIZE> &H, 
		      float reject_threshold  ) 
	{
	    filter->P  = P; 
	    filter->x  = x.vector(); 
	    //innovation steps
	    // innovation 
	    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y = filter->innovation<MEASUREMENT_SIZE>( p, H );

	    // innovation covariance matrix
	    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = filter->innovationCovariance<MEASUREMENT_SIZE>(H, R); 

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
		Eigen::Matrix<double, StatePosVelOriAcc::SIZE, MEASUREMENT_SIZE> K = filter->gain<MEASUREMENT_SIZE>( H, S );

		//update teh state 
		filter-> update<MEASUREMENT_SIZE>( H, K, y); 

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
    
	
    };
}

#endif

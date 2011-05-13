#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "FaultDetection.hpp"

#include <iostream>

namespace KalmanFilter {
 
template <unsigned int SIZE>
class KF
    {
      
      public:
	
	/** do i need to declare the size as dynamic ? */ 
	
	/** state estimate */
	Eigen::Matrix<double,SIZE,1>  x;
	
	/** covariance matrix */
	Eigen::Matrix<double, SIZE, SIZE> P;

      	/**fault detection libary */ 
	pose_ekf::ChiSquared* chi_square; 
	
      public:
	/** what is this line */ 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	KF()
	{
	      
	      chi_square = new pose_ekf::ChiSquared;
	      x = Eigen::Matrix<double,SIZE,1>::Zero(); 
	      P = Eigen::Matrix<double, SIZE, SIZE>::Zero();
	      
	}
	
	~KF()
	{
	    delete chi_square; 
	}
	
	/** prediction step 
	    F - state transition
	    Q - process noise covariance matrix, 
	    dt - discrete time time step 
	    */
	void predictionDiscrete(Eigen::Matrix<double, SIZE, SIZE> F,
	      Eigen::Matrix<double, SIZE, SIZE> Q,
	      double dt)
	{

		//State transition 
		x = ( Eigen::Matrix<double, SIZE, SIZE>::Identity() + F * dt + F * dt * dt / 2 ) * x; 
		
		//covariance update 
		P = P + (F * P + P * F.transpose()) * dt + F * P * F.transpose() * dt * dt + Q * dt; 
		
	}
 
	/** update step 
	    u - Input 
	    B - relation between input and state 
	    F - state transition
	    Q - process noise covariance matrix, 
	    */
	template<unsigned int INPUT_SIZE> 
	void prediction(Eigen::Matrix<double, INPUT_SIZE, 1> u,
	      Eigen::Matrix<double,SIZE ,INPUT_SIZE> B, 
	      Eigen::Matrix<double, SIZE, SIZE> F,
	      Eigen::Matrix<double, SIZE, SIZE> Q )
	{

		//State transition 
		x=F*x+B*u; 
		
		//covariance update 
		P = F*P*F.transpose() + Q;
		
	}
	
 
	/** correction step, 
	 p - observation 
	 H - observation function
	 R - measurement noise covariance matrix
	*/	
	template<unsigned int MEASUREMENT_SIZE> 
	void correction( Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R)	  
	{
	    // innovation  
	    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y = p - H*x;
	    //innovation covariance
	    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = H*P*H.transpose()+R;
	    //gain 
	    Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> K = P*H.transpose()*S.inverse();
	    //update
	    x = x + K*y;
	    
	    P = (Eigen::Matrix<double, SIZE, SIZE>::Identity() - K*H)*P;
	    
	    
	}


	template <unsigned int MEASUREMENT_SIZE>
	Eigen::Matrix<double, MEASUREMENT_SIZE, 1>  innovation( Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p,
	   Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> H)	  
	{
	    
	    return (p - H*x);
	    
	}
	
 	/** innovation covariance step, 
	 p - observation 
	 h - observation model function
	 H -  observation model
	 R - measurement noise covariance matrix
	*/	
	template <unsigned int MEASUREMENT_SIZE>
	Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE>  innovationCovariance(
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R)	  
	{
	  
	    return (H*P*H.transpose()+R);
  
	}
	
	/** update step, 
	 H - observation model
	 y - innovation
	 K - gain 
	*/	
	template <unsigned int MEASUREMENT_SIZE>
	void update( Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> H,
		     Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> K,
		     Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y)	  
	{

	    x = x + K*y;
	    
	    P = (Eigen::Matrix<double, SIZE, SIZE>::Identity() - K*H)*P;
	    
	}

	/** calculates the gain 
	 J_H - jacobian observation model
	*/
	template <unsigned int MEASUREMENT_SIZE>
	Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> gain( 
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S )	  
	{

	    // correct the estimate and covariance according to measurement
	    return (P*H.transpose()*S.inverse());
	    
	}
	/** Applies the Kalman Filter correction using a chi_square method to reject bad observation samples
	    p - Observation 
	    R - observation noise 
	    H - how state is translated into observation 
	    chi_square_reject_threshold - threashold for the rejection (0 = no rejection) 
	    return bool - if the observation was rejected or not. 
	*/
		template < unsigned int MEASUREMENT_SIZE, unsigned int DEGREE_OF_FREEDOM >
	bool correctionChiSquare(const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &p, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> &R, 
		      const Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> &H, 
		      float chi_square_reject_threshold  ) 
	{
	    //filter->P  = P; 
	    //filter->x  = x.vector(); 
	    //innovation steps
	    // innovation 
	    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y = innovation<MEASUREMENT_SIZE>( p, H );

	    // innovation covariance matrix
	    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = innovationCovariance<MEASUREMENT_SIZE>(H, R); 

	    bool reject_observation; 
	    //test to reject data 
	    if ( chi_square_reject_threshold!=0 ) 
	    {
	      
		reject_observation = chi_square->rejectData<DEGREE_OF_FREEDOM>(y.head(DEGREE_OF_FREEDOM), S.block(0,0,DEGREE_OF_FREEDOM,DEGREE_OF_FREEDOM) ,chi_square_reject_threshold );
		
	    }  
	    else
		reject_observation = false;
	    
	    if(!reject_observation)
	    {
	    
		//Kalman Gain
		Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> K = gain<MEASUREMENT_SIZE>( H, S );

		//update teh state 
		update<MEASUREMENT_SIZE>( H, K, y); 

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

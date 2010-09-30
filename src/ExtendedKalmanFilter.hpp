#ifndef __EXTENDED_KALMAN_FILTER_HPP__
#define __EXTENDED_KALMAN_FILTER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ExtendedKalmanFilter {
 
template <unsigned int SIZE> 
class EKF
    {
      
      public:
	/** what is this line */ 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	/** state estimate */
	Eigen::Matrix<double,SIZE,1>  x;
	
	/** covariance matrix */
	Eigen::Matrix<double, SIZE, SIZE> P;
	
      public:
	/** prediction step 
	    f - state transition
	    Q - process noise covariance matrix, 
	    J_F - Jacobian of the state transition */
	void prediction(Eigen::Matrix<double, SIZE, 1> f,
	      Eigen::Matrix<double, SIZE, SIZE> J_F,
	      Eigen::Matrix<double, SIZE, SIZE> Q )
	{

		//State transition 
		x=f; 
		
		//covariance update 
		P = J_F*P*J_F.transpose() + Q;
		
	}
 
 	/** innovation step, 
	 p - observation 
	 h - observation model function
	*/	
	template <unsigned int MEASUREMENT_SIZE>
	Eigen::Matrix<double, MEASUREMENT_SIZE, 1>  innovation( Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, 1> h)	  
	{
	    
	    return (p - h);
	    
	}
	
 	/** innovation covariance step, 
	 p - observation 
	 h - observation model function
	 J_H - jacobian observation model
	 R - measurement noise covariance matrix
	*/	
	template <unsigned int MEASUREMENT_SIZE>
	Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE>  innovationCovariance(
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> J_H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R)	  
	{
	  
	    return (J_H*P*J_H.transpose()+R);
  
	}
	
	/** update step, 
	 J_H - jacobian observation model
	 y - innovation
	 K - gain 
	*/	
	template <unsigned int MEASUREMENT_SIZE>
	void update( Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> J_H,
		     Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> K,
		     Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y)	  
	{

	    x = x + K*y;
	    
	    P = (Eigen::Matrix<double, SIZE, SIZE>::Identity() - K*J_H)*P;
	    
	}

	/** calculates the gain 
	 J_H - jacobian observation model
	*/
	template <unsigned int MEASUREMENT_SIZE>
	Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> gain( 
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> J_H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S )	  
	{

	    // correct the estimate and covariance according to measurement
	    return (P*J_H.transpose()*S.inverse());
	    
	}

	/** correction - a step that includes the innovation, gain and update steps into a single step 
	 p - observation 
	 h - observation model function
	 J_H - jacobian observation model
	 R - measurement noise covariance matrix
	*/	
	template <unsigned int MEASUREMENT_SIZE>
	void correction(  Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, 1> h,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> J_H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R)	    
	{
	  
	    /** innovation */ 
	    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y;
	
	    /** innovation covariance matrix */ 
	    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S; 
	
	    /** kalman gain */
	    Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> K;
 	    
	    y = p - h;
	    
	    S = J_H*P*J_H.transpose()+R;

	    // correct the estimate and covariance according to measurement
	    K = P*J_H.transpose()*S.inverse();
	    
	    x = x + K*y;
	    
	    P = (Eigen::Matrix<double, SIZE, SIZE>::Identity() - K*J_H)*P;
	    
	}
	
    };
}

#endif

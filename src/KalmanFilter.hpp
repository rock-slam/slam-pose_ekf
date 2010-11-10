#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>


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


      public:
	/** what is this line */ 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
		
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

    };
}

#endif

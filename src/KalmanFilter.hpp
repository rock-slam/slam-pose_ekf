#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace KalmanFilter {
 
template <unsigned int SIZE, unsigned int INPUT_SIZE, unsigned int MEASUREMENT_SIZE> 
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
	
	/** update step 
	    u - Input 
	    B - relation between input and state 
	    F - state transition
	    Q - process noise covariance matrix, 
	    */
	void update(Eigen::Matrix<double, INPUT_SIZE, 1> u,
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
	void correction( Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, SIZE> H,
	  Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> R)	  
	{
	    // correct the estimate and covariance according to measurement
	    
	    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> y = p - H*x;
	    
	    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> S = H*P*H.transpose()+R;

	    Eigen::Matrix<double, SIZE, MEASUREMENT_SIZE> K = P*H.transpose()*S.inverse();
	  
	    x = x + K*y;
	    
	    P = (Eigen::Matrix<double, SIZE, SIZE>::Identity() - K*H)*P;
	    
	}


	
    };
}

#endif

#ifndef __KALMAN_FILTER_TYPES__
#define __KALMAN_FILTER_TYPES__

#include <base/time.h>


namespace pose_estimator
{ 

	struct KFD_PosVelOriAccType
	{
	 
	    base::Time time;    
	    Eigen::Matrix<double,15,1> x; 
	    Eigen::Matrix<double,15,15> P;
	    
	}; 
	

  
}

#endif



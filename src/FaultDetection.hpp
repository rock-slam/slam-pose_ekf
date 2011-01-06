
#ifndef FaultDetection_HPP
#define FaultDetection_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pose_ekf{ 


class ChiSquared 
{
    public: 
	
    /**
    Uses the Chi_Square distribution under gaussion assumptions to reject or not an innovation 
    */ 
    template <unsigned int DEGREE_OF_FREEDOM>
    bool rejectData(Eigen::Matrix< double, DEGREE_OF_FREEDOM, 1 > innovation,
		    Eigen::Matrix< double, DEGREE_OF_FREEDOM, DEGREE_OF_FREEDOM > innovation_covariance, 
		    float threshold)
    {

      
	  if ( (innovation.transpose() * innovation_covariance * innovation)[0] <= threshold )  
	      return false;
	  else
	      return true; 
	  
	  
    }	    

};
  
}
#endif 

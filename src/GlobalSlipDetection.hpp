#ifndef __GLOBAL_SLIP_DETECTION_HPP__
#define __GLOBAL_SLIP_DETECTION_HPP__

#include <Eigen/Core>
#include <math.h>
namespace GlobalSlipDetection {
 
    class GSlip
    {
      
      public:
	void splitGyroIndicator (double deltaYawGyro, double deltaYawOdometry);
      
    	GSlip();
	~GSlip();
	
	
	
    };
}

#endif

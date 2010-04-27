#include "GlobalSlipDetection.hpp"

using namespace GlobalSlipDetection;
using namespace std;

GSlip::GSlip(){
}
GSlip::~GSlip(){
}
void GSlip::splitGyroIndicator (double deltaYawGyro, double deltaYawOdometry) {
  double deltaDif = abs(deltaYawGyro - deltaYawOdometry);
  if (deltaDif > 0.5*3.14/180) 
    std::cout<<" Splip 1 " << deltaDif*180/3.14 <<std::endl; 
 
  
}








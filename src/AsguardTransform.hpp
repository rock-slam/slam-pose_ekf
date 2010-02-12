#ifndef __ASGUARDTRANSFORM_HPP__
#define __ASGUARDTRANSFORM_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>


#ifndef M_PI
#define M_PI 3.141592653589793238462643 
#endif

using namespace std;
namespace asguard 
{
    /** Singleton class which provides coordinate transformations for the 
     * asguard system. Body origin is the centor of the front wheel axle. 
     * The coordinates are defined as follows:
     *
     * x - right direction looking at the robot from the top
     * y - forward direction
     * z - positive upward direction
     */
    class Transform
    {
    public:
	static Transform& Instance() {
	    static Transform singleton;
	    return singleton;
	}
	/** GPS Offset from body origin */
	Eigen::Vector3d z_g;
	/** IMU Offset from body origin */
	Eigen::Vector3d z_i;
	/** LaserScanner Offset from body origin */
	Eigen::Vector3d z_ls;

	/** body to IMU fixed coordinate system */
	Eigen::Quaterniond R_b2i;
	/** body to GPS fixed coordinate system */
	Eigen::Quaterniond R_b2g;
	/** body to LaserScan fixed coordinate system */
	Eigen::Quaterniond R_b2ls;

	/** get imu to world transformation for IMU orientation reading 
	 * @todo: this should be templated for different IMUs, so that e.g.
	 * XSens and DFKIIMU can be used at the same time
	 */
	Eigen::Quaterniond R_i2w( const Eigen::Quaterniond& R_i2xw )
	{
	 
	   return  R_xw2w * R_i2xw ;
	    
	};

    private:
	double sensorHeadAngle;
	Eigen::Quaterniond R_y;
	Eigen::Quaterniond R_xw2w;

	Transform()
	{
	    
	    sensorHeadAngle = -8.2/180.0*M_PI;

	    z_g = Eigen::Vector3d( 0, -0.425, 0.17 );
	    z_i = Eigen::Vector3d( 0, 0.03, 0.10 );
	    z_ls = Eigen::Vector3d( 0, 0.05, 0.15 );
	    
	    
	    R_xw2w=Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()); 
           
	    R_b2i = Eigen::AngleAxisd( sensorHeadAngle, Eigen::Vector3d::UnitY() ) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ() );
	};
	Transform(Transform const&);
	Transform& operator=(Transform const&);
    };
};

#endif

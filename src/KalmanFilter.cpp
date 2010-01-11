#include "KalmanFilter.hpp"
#include <Eigen/LU> 

using namespace KalmanFilter;

void PositionKF::update( Eigen::Vector3d a )
{
    // Transform acceleration which is in body local coordinates
    // to global coordinate frame
    Eigen::Vector3d a_b = a;

    // update the state and covariance matrices
    x_kp.vector() = F*x_k.vector() + B*a_b;
    P_kp = F*P_k*F.transpose() + Q;
    
    // for now just copy the state, we could make 
    // this conditional depending on wether there will
    // be a correction or not.
    x_k.vector() = x_kp.vector();
    P_k = P_kp;
}

void PositionKF::correction( Eigen::Vector3d p )
{
    // correct the estimate and covariance according to measurement
    Eigen::Vector3d y = p - H*x_kp.vector();
    Eigen::Matrix3d S = H*P_kp*H.transpose()+R;
    Eigen::Matrix<double, State::SIZE, MEASUREMENT_SIZE> K = P_kp*H.transpose()*S.inverse();
    x_k.vector() = x_kp.vector() + K*y;
    P_k = (Eigen::Matrix<double, State::SIZE, State::SIZE>::Identity() - K*H)*P_kp;
}


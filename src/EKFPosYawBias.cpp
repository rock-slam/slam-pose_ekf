#include "EKFPosYawBias.hpp"
#include <Eigen/LU> 
#include <math.h>

using namespace pose_estimator;

/** CHECK */ 
EKFPosYawBias::EKFPosYawBias() 
{
    filter =  new ExtendedKalmanFilter::EKF<State::SIZE,INPUT_SIZE,MEASUREMENT_SIZE>;
}

EKFPosYawBias::~EKFPosYawBias()
{
    delete filter; 
}

/** update the filter */ 
void EKFPosYawBias::update( const Eigen::Vector3d &v_w, double d_t )
{	
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 

    //sets the transition matrix 
    Eigen::Matrix<double, State::SIZE, 1> f;
    f.start<3>()=x.xi()+R_w2wb*v_w*d_t;
    f.end<1>()=x.yaw();

    //sets the Jacobian of the state transition matrix 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F
	= jacobianF(v_w, d_t);

    //updates the Kalman Filter 
    filter->update(f,J_F, Q ); 

    //get the updated values 
    x.vector()=filter->x; 
    P=filter->P; 
}

void EKFPosYawBias::correctionPos( const Eigen::Matrix<double, 3, 1> &p )
{
    //jacobian of the observation function 
    Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE> J_H;
    J_H.setIdentity();
    J_H(3,3) = 0;

    // copy the 3 vector of the position input to a 4 vector for the measurement
    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> p_m;
    p_m.start<3>() = p;
    p_m(3) = 0;

    //observation function (the observation is linear so the h matrix can be defined as
    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> h
	= J_H*x.vector(); 

    //correct the state 
    filter->correction( p_m,h, J_H, R); 

    //get the corrected values 
    x.vector()=filter->x; 
    P=filter->P; 
}

void EKFPosYawBias::correction( const Eigen::Matrix<double, MEASUREMENT_SIZE, 1> &p )
{
    //jacobian of the observation function 
    Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE> J_H 
	= jacobianH (); 

    //observation function (the observation is linear so the h matrix can be defined as
    Eigen::Matrix<double, MEASUREMENT_SIZE, 1> h
	= J_H*x.vector(); 

    //correct the state 
    filter->correction( p,h, J_H, R); 

    //get the corrected values 
    x.vector()=filter->x; 
    P=filter->P; 
}

/** Calculates the Jacobian of the transition matrix */ 
Eigen::Matrix<double, State::SIZE, State::SIZE> EKFPosYawBias::jacobianF( const Eigen::Vector3d &v_w, double d_t )
{
    //derivate of the rotation do to yaw bias 
    Eigen::Matrix<double, 3,3> dR_z;
    dR_z<<-sin(x.yaw()(0,0)), -cos(x.yaw()(0,0)), 0,cos(x.yaw()(0,0)),-sin(x.yaw()(0,0)),0,0,0,0; 

    //jacobian 
    Eigen::Matrix<double, State::SIZE, State::SIZE> J_F; 
    J_F.setIdentity(); 
    J_F.block<3,1>(0,3)
	= dR_z*v_w*d_t;

    return J_F;
}

/**jacobian observation model*/ 
Eigen::Matrix<double, EKFPosYawBias::MEASUREMENT_SIZE, State::SIZE> EKFPosYawBias::jacobianH ()
{
    Eigen::Matrix<double, MEASUREMENT_SIZE, State::SIZE>  J_H;
    J_H.setIdentity();

    return J_H; 
}

/** calculates the process noise in world frame*/ 
void EKFPosYawBias::processNoise(const Eigen::Matrix<double, State::SIZE, State::SIZE> &Q)
{ 
    this->Q = Q; 

    // Rotate from world to world with bias 
    //calculates the rotation from world to world frame corrected by the bias 
    Eigen::Quaterniond R_w2wb;
    R_w2wb=Eigen::AngleAxisd(x.yaw()(0,0), Eigen::Vector3d::UnitZ()); 

    Eigen::Matrix3d R_w2wb_ = Eigen::Matrix3d(R_w2wb);    

    this->Q.block<3,3>(0,0) = R_w2wb_*this->Q.block<3,3>(0,0) *R_w2wb_.transpose();
} 

/** calculates the measurement noise */ 
void EKFPosYawBias::measurementNoise(const Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> &R)
{
    this->R=R; 
}

/** calculates the measurement noise */ 
void EKFPosYawBias::measurementNoisePos(const Eigen::Matrix<double, POS_SIZE, POS_SIZE> &R)
{
    this->R.corner<POS_SIZE,POS_SIZE>(Eigen::TopLeft)=R; 
}

/** configurarion hook */ 
void EKFPosYawBias::init(const Eigen::Matrix<double, State::SIZE, State::SIZE> &P, const Eigen::Matrix<double,State::SIZE,1> &x)
{
    Q.setZero(); 
    R.setZero(); 

    this->P=P;
    this->x.vector()=x; 
    filter->x=x; 
    filter->P=P;
}


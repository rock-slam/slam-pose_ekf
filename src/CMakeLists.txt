set(POSE_HDRS
    FaultDetection.hpp
    KalmanFilter.hpp
    ExtendedKalmanFilter.hpp
    EKFPosYawBiasT.hpp
    KFD_PosVelOriAcc.hpp
    KFD_PosVelAcc.hpp
    SlowFastAggregator.hpp
    )

set(POSE_SRCS
    EKFPosYawBiasT.cpp
    KFD_PosVelOriAcc.cpp
    KFD_PosVelAcc.cpp
    )

rock_library(pose_ekf 
    SOURCES ${POSE_SRCS} 
    HEADERS ${POSE_HDRS})

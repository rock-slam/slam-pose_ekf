project(pose_ekf)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

add_definitions(-Wall)

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

pkg_check_modules(DFKI REQUIRED "base-types")
include_directories(${DFKI_INCLUDE_DIRS})
link_directories(${DFKI_LIBRARY_DIRS})

include_directories(./src)

set(POSE_HDRS
    src/FaultDetection.hpp
    src/KalmanFilter.hpp
    src/ExtendedKalmanFilter.hpp
    src/KalmanFilterTypes.hpp
    src/EKFPosYawBiasT.hpp
    src/KFD_PosVelOriAcc.hpp
    )

set(POSE_SRCS
    src/EKFPosYawBiasT.cpp
    src/KFD_PosVelOriAcc.cpp
    )

add_library(pose_ekf SHARED ${POSE_SRCS})

target_link_libraries(pose_ekf ${FAULT_DETECTION_LIBRARIES})


install(TARGETS pose_ekf
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${POSE_HDRS} DESTINATION include/pose_ekf)

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(pose_ekf.pc.in pose_ekf.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/pose_ekf.pc DESTINATION lib/pkgconfig)

include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(test)



project(pose_estimator)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

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
    src/Aggregator.hpp
    src/AsguardTransform.hpp
    src/KalmanFilter.hpp
    )

set(POSE_SRCS
    src/KalmanFilter.cpp
    )

add_library(pose_estimator SHARED ${POSE_SRCS})

install(TARGETS pose_estimator
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${POSE_HDRS} DESTINATION include/pose_estimator)

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(pose_estimator.pc.in pose_estimator.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/pose_estimator.pc DESTINATION lib/pkgconfig)

include_directories(${PROJECT_SOURCE_DIR})
add_subdirectory(test)



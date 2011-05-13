cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
rock_init(pose_ekf 1.0)

rock_find_cmake(Eigen3 REQUIRED)
rock_find_cmake(Boost REQUIRED)

rock_export_includedir(${PROJECT_SOURCE_DIR}/src pose_ekf)

rock_standard_layout()

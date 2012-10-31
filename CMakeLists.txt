cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(pose_ekf 1.0)

rock_find_pkgconfig(BASE_TYPES base-types REQUIRED)
rock_find_cmake(Boost REQUIRED)

rock_export_includedir(${PROJECT_SOURCE_DIR}/src pose_ekf)

rock_standard_layout()

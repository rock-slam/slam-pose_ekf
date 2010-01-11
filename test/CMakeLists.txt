find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

set(SRCS UnitTest.cpp)
add_executable(unit_test ${SRCS})
target_link_libraries(unit_test pose_estimator)

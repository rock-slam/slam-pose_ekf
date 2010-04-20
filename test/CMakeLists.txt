find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

include_directories(${ASGUARD_INCLUDE_DIR})
link_directories(${ASGUARD_LINK_DIR})

set(SRCS UnitTest.cpp)
add_executable(unit_test ${SRCS})
target_link_libraries(unit_test pose_estimator)

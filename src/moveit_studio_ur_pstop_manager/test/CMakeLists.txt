find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ros_testing REQUIRED)

ament_add_gtest_executable(test_reset_ur_pstop
    test_reset_ur_pstop.cpp)
ament_target_dependencies(test_reset_ur_pstop ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_launch_test(launch/test_reset_ur_pstop.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

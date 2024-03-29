find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(moveit_studio_agent REQUIRED)
find_package(moveit_studio_common REQUIRED)
find_package(ros_testing REQUIRED)

# Integration test for ObjectiveServerNode running with other Agent nodes using real Behaviors
ament_add_gtest_executable(test_basic_ur5e_objectives test_basic_ur5e_objectives.cpp)
target_link_libraries(test_basic_ur5e_objectives moveit_studio_agent::do_objective_test_fixture)
add_ros_test(launch/test_basic_ur5e_objectives.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# Integration test for the 'getPlanningGroups' MoveGroup capability.
ament_add_gtest_executable(test_get_planning_groups test_get_planning_groups.cpp)
target_link_libraries(test_get_planning_groups moveit_studio_agent::do_objective_test_fixture)
add_ros_test(launch/test_get_planning_groups.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# Integration test for the UR P-stop manager
ament_add_gtest_executable(test_reset_ur_pstop test_reset_ur_pstop.cpp)
ament_target_dependencies(test_reset_ur_pstop ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_launch_test(launch/test_reset_ur_pstop.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# Install config files needed at runtime by integration tests
install(
  DIRECTORY
    objectives
  DESTINATION share/${PROJECT_NAME}/test
)

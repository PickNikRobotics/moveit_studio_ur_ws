cmake_minimum_required(VERSION 3.22)
project(picknik_ur_gazebo_scan_and_plan_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_studio_common REQUIRED)

extend_moveit_pro_config(picknik_ur_gazebo_config)

install(
  DIRECTORY
    config
    description
    waypoints
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

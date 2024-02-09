cmake_minimum_required(VERSION 3.22)
project(picknik_ur_gazebo_peg_in_hole)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    mesh
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

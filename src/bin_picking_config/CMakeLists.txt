cmake_minimum_required(VERSION 3.22)
project(bin_picking_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    calibrate
    config
    description
    launch
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

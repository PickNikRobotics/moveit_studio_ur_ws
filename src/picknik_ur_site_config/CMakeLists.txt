cmake_minimum_required(VERSION 3.22)
project(picknik_ur_site_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    # Uncomment the following lines if you make use of these directories in you site_config package
    # description
    launch
    objectives
    # waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.22)
project(picknik_ur_site_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

# add custom behaviors
add_subdirectory(behaviors)

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

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
        moveit_studio_behavior_interface behaviors/trigger_pstop_reset_service_plugin_description.xml)

ament_package()

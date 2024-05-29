cmake_minimum_required(VERSION 3.22)
project(setup_mtc_place_pose)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(moveit_studio_vision_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(pluginlib REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    moveit_studio_behavior_interface
    pluginlib
    moveit_task_constructor_core
    moveit_studio_vision_msgs)

add_library(
    setup_mtc_place_pose
    SHARED
    src/setup_mtc_place_pose.cpp
    src/register_behaviors.cpp)
target_include_directories(
    setup_mtc_place_pose
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(setup_mtc_place_pose
    ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
    TARGETS setup_mtc_place_pose
    EXPORT setup_mtc_place_poseTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    add_subdirectory(test)
    ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
    moveit_studio_behavior_interface setup_mtc_place_pose_plugin_description.xml)

ament_export_targets(setup_mtc_place_poseTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

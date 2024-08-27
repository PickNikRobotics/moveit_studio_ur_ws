cmake_minimum_required(VERSION 3.22)
project(translate_pose CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS 
  geometry_msgs
  moveit_studio_behavior_interface 
  pluginlib)
  
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  translate_pose
  SHARED
  src/translate_pose.cpp
  src/register_behaviors.cpp)
target_include_directories(
  translate_pose
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(translate_pose
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS translate_pose
  EXPORT translate_poseTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
  target_compile_options(translate_pose PRIVATE -fprofile-arcs -ftest-coverage)
  target_link_options(translate_pose PRIVATE -fprofile-arcs -ftest-coverage)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface translate_pose_plugin_description.xml)

ament_export_targets(translate_poseTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

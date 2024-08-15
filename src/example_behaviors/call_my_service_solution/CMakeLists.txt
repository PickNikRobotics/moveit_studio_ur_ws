cmake_minimum_required(VERSION 3.22)
project(call_my_service CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(service_interface REQUIRED)
find_package(pluginlib REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface service_interface pluginlib)

add_library(
  call_my_service
  SHARED
  src/call_my_service.cpp
  src/register_behaviors.cpp)
target_include_directories(
  call_my_service
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(call_my_service
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS call_my_service
  EXPORT call_my_serviceTargets
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
  target_compile_options(call_my_service PRIVATE -fprofile-arcs -ftest-coverage)
  target_link_options(call_my_service PRIVATE -fprofile-arcs -ftest-coverage)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface call_my_service_plugin_description.xml)

ament_export_targets(call_my_serviceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

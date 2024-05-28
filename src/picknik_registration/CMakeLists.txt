cmake_minimum_required(VERSION 3.22)
project(picknik_registration CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface pluginlib
moveit_studio_vision
moveit_studio_vision_msgs
PCL
pcl_conversions
pcl_ros
moveit_ros_planning_interface
geometric_shapes
shape_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  picknik_registration
  SHARED
  src/ndt_registration.cpp
  src/spawn_mesh.cpp
  src/ransac_registration.cpp
  src/register_behaviors.cpp)
target_include_directories(
  picknik_registration
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(picknik_registration
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_link_libraries(picknik_registration ${PCL_LIBRARIES})

# Install Libraries
install(
  TARGETS picknik_registration
  EXPORT picknik_registrationTargets
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
  moveit_studio_behavior_interface picknik_registration_plugin_description.xml)

ament_export_targets(picknik_registrationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_studio_behavior_interface
  pluginlib
  std_srvs
  )
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
        trigger_pstop_reset_service
        SHARED
        src/trigger_pstop_reset_service.cpp
        src/register_behaviors.cpp)
target_include_directories(
        trigger_pstop_reset_service
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trigger_pstop_reset_service
        ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  moveit_pro_behavior_test(trigger_pstop_reset_service)
endif()

# Install Libraries
install(
        TARGETS trigger_pstop_reset_service
        EXPORT trigger_pstop_reset_serviceTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

ament_export_targets(trigger_pstop_reset_serviceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

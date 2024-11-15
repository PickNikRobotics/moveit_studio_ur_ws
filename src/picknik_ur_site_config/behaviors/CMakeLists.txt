set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_studio_behavior_interface
  pluginlib
  std_srvs
  geometry_msgs
  moveit_studio_vision_msgs
  sensor_msgs
  ament_index_cpp
  OpenCV
  )

foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
        trigger_pstop_reset_service
        SHARED
        src/trigger_pstop_reset_service.cpp
        src/sam2_segmentation.cpp
        src/register_behaviors.cpp)
target_include_directories(
        trigger_pstop_reset_service
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trigger_pstop_reset_service
        ${THIS_PACKAGE_INCLUDE_DEPENDS})

find_package(moveit_pro_ml REQUIRED)
target_link_libraries(trigger_pstop_reset_service onnx_sam2)


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

# install ML models
install(
        DIRECTORY models
        DESTINATION share/${PROJECT_NAME}
)


ament_export_targets(trigger_pstop_reset_serviceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

cmake_minimum_required(VERSION 3.22)
project(moveit_studio_ur_pstop_manager CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_studio_agent_msgs)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager_msgs
  moveit_studio_agent_msgs
  pluginlib
  rclcpp
  rclcpp_components
  std_srvs
  ur_dashboard_msgs
)

add_executable(protective_stop_manager_node src/protective_stop_manager_node.cpp)
target_include_directories(protective_stop_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(protective_stop_manager_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Mock UR dashboard client
add_executable(mock_ur_dashboard_client_node
  src/mock_ur_dashboard_client.cpp
  src/mock_ur_dashboard_client_node.cpp)
target_include_directories(mock_ur_dashboard_client_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_ur_dashboard_client_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
    mock_ur_dashboard_client_node
    protective_stop_manager_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

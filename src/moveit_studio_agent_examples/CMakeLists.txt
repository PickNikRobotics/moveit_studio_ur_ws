cmake_minimum_required(VERSION 3.16.3)
project(moveit_studio_agent_examples)

find_package(ament_cmake REQUIRED)
find_package(moveit_studio_agent_msgs REQUIRED)
find_package(moveit_studio_behavior_msgs REQUIRED)

# These are packages needed to link against the necessary MoveIt Studio ROS interface definitions.
set(THIS_PACKAGE_INCLUDE_DEPENDS
    moveit_studio_agent_msgs
    moveit_studio_behavior_msgs
)

#############
## Install ##
#############

# Install scripts directory
install(PROGRAMS 
    scripts/call_do_objective.py 
    scripts/call_do_objective_waypoint.py
    scripts/call_do_objective_then_cancel.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

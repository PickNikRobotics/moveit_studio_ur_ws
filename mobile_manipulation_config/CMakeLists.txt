cmake_minimum_required(VERSION 3.22)
project(mobile_manipulation_config)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)



# Install all XML files in directory
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*")

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

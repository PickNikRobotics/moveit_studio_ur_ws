cmake_minimum_required(VERSION 3.22)
project(mobile_manipulation_config)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)


# External XML file location
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}/mujoco_assets/"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*.xml")

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

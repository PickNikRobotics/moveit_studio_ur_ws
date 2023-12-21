cmake_minimum_required(VERSION 3.22)
project(picknik_ur_studio_integration_testing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()

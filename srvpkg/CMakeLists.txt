cmake_minimum_required(VERSION 3.8)
project(srvpkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Waypoint.srv"
)

# Export interface dependency to downstream packages
ament_export_dependencies(rosidl_default_runtime)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


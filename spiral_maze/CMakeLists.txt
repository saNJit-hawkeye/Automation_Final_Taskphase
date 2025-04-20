cmake_minimum_required(VERSION 3.8)
project(spiral_maze)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install Python node
install(PROGRAMS
    codes/movemaze
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource folders (launch, urdf, etc.)
install(
  DIRECTORY
    launch
    models
    world
    urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


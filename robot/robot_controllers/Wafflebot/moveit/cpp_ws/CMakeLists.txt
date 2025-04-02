cmake_minimum_required(VERSION 3.8)
project(collision_publisher)

# Find dependencies first
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Declare the executable target
add_executable(collision_publisher src/collision_publisher.cpp)

ament_target_dependencies(collision_publisher
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS
  collision_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


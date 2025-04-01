cmake_minimum_required(VERSION 3.12)
project(env_collision_checker)

# Ensure Python3 is available
find_package(Python3 COMPONENTS Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pybind11 REQUIRED)

# Debugging info
message(STATUS "MoveIt ROS Planning Interface Found: ${moveit_ros_planning_interface_FOUND}")
message(STATUS "MoveIt Core Found: ${moveit_core_FOUND}")

# Manually set MoveIt include and library paths
set(MOVEIT_INCLUDE_DIRS
  "/home/havard/ws_moveit/install/moveit_ros_planning_interface/include"
  "/home/havard/ws_moveit/install/moveit_core/include"
)
set(MOVEIT_LIBRARIES
  "/home/havard/ws_moveit/install/moveit_ros_planning_interface/lib/libmoveit_move_group_interface.so"
  "/home/havard/ws_moveit/install/moveit_core/lib/libmoveit_core.so"
)

# Include directories
include_directories(${MOVEIT_INCLUDE_DIRS})

# Declare the executable target for the ROS2 node
add_executable(collision_checker_node src/collision_checker_node.cpp)
ament_target_dependencies(collision_checker_node
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

# Declare the Python binding module
pybind11_add_module(moveit_python_binding src/moveit_python_binding.cpp)

# Link required libraries
target_link_libraries(moveit_python_binding PRIVATE
  rclcpp::rclcpp
  ${MOVEIT_LIBRARIES}
)

# Install targets
install(TARGETS
  collision_checker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  moveit_python_binding
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Additional compile options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

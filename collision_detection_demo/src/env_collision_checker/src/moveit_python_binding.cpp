#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <chrono>

// ROS2 and MoveIt includes
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace py = pybind11;

// Function to load file content
std::string loadFileContent(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_path);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Function that encapsulates the main planning steps
void runPlanner(const std::string &urdf_path, const std::string &srdf_path) {
    // Initialize ROS2
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("python_collision_checker_node");

    try {
        // Load URDF and SRDF directly
        std::string robot_description = loadFileContent(urdf_path);
        std::string robot_description_semantic = loadFileContent(srdf_path);

        node->declare_parameter("robot_description", robot_description);
        node->declare_parameter("robot_description_semantic", robot_description_semantic);

        RCLCPP_INFO(node->get_logger(), "URDF and SRDF loaded successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load URDF/SRDF: %s", e.what());
        rclcpp::shutdown();
        throw;
    }

    // Create MoveGroupInterface for the planning group
    moveit::planning_interface::MoveGroupInterface move_group(node, "interbotix_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Allow some time for the planning scene to update
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Define a collision object (a box)
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "environment_box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.2, 0.2, 0.2};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.15;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Add the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);

    RCLCPP_INFO(node->get_logger(), "Added collision object to the planning scene.");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Set a target pose for the end-effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.4;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;
    move_group.setPoseTarget(target_pose);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode planning_result = move_group.plan(my_plan);

    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS)
        RCLCPP_INFO(node->get_logger(), "Motion plan found that avoids the collision object!");
    else
        RCLCPP_WARN(node->get_logger(), "Motion planning failed.");

    rclcpp::shutdown();
}

// Define the Python module
PYBIND11_MODULE(moveit_python_binding, m) {
    m.doc() = "Python bindings for a MoveIt collision checker using ROS2";
    m.def("load_file_content", &loadFileContent, "Load file content from a given file path");
    m.def("run_planner", &runPlanner, "Run the MoveIt planner",
          py::arg("urdf_path"), py::arg("srdf_path"));
}

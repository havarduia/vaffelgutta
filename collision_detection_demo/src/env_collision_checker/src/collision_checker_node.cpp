#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>
#include <sstream>
#include <cstdlib>

std::string loadFileContent(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string processXacroFile(const std::string &xacro_path) {
    std::string command = "xacro " + xacro_path;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe) {
        throw std::runtime_error("Failed to run xacro command.");
    }

    char buffer[128];
    std::stringstream result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result << buffer;
    }
    pclose(pipe);

    return result.str();
}

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("collision_checker_node");

    // Paths to URDF and SRDF
    std::string urdf_path = "/home/havard/git/vaffelgutta/collision_detection_demo/vx300s.urdf";
    std::string srdf_path = "/home/havard/git/vaffelgutta/collision_detection_demo/vx300s.srdf";

    try {
        std::string robot_description = processXacroFile(urdf_path);
        std::string robot_description_semantic = processXacroFile(srdf_path);

        node->declare_parameter("robot_description", robot_description);
        node->declare_parameter("robot_description_semantic", robot_description_semantic);

        RCLCPP_INFO(node->get_logger(), "URDF and SRDF loaded successfully.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load URDF/SRDF: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Create MoveGroupInterface for the robot's planning group
    moveit::planning_interface::MoveGroupInterface move_group(node, "interbotix_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Allow some time for the planning scene to update
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Define a collision object (a box)
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "environment_box";

    // Define the box shape and dimensions
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.2, 0.2, 0.2};  // x, y, z dimensions

    // Define the pose of the box (positioned in front of the robot)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // No rotation
    box_pose.position.x = 0.5;     // 0.8 meters in front of the robot
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.15;    // So that the box sits on the ground

    // Add the box shape and pose to the collision object
    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Add the collision object into the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);

    RCLCPP_INFO(node->get_logger(), "Added collision object to the planning scene.");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Set a target pose for the end-effector (expressed in the planning frame)
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.4;  // Chosen so the robot must avoid the box
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
    return 0;
}

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include "json.hpp"

using json = nlohmann::json;
// Function to load file conten
json open_boxes(int filechoice)
{
  std::string readbuffer;
  std::string text;
  std::filesystem::path source_dir = std::filesystem::path(__FILE__).parent_path().lexically_normal();
  std::filesystem::path filepath_add = "../../../../../assets/boundingboxes/publish/add.json";
  std::filesystem::path filepath_rm = "../../../../../assets/boundingboxes/publish/remove.json";
  std::filesystem::path filepath;

  if (filechoice == 0)
  {
    filepath = (source_dir / filepath_add);
  }
  else
  {
    filepath = (source_dir / filepath_rm);
  }

  bool file_loaded = false;
  int retries_timer = 0;
  while (!file_loaded && retries_timer <=50)
  {
    retries_timer++;
    std::ifstream file(filepath);
    if (file)
    {
      std::stringstream buffer;
      buffer << file.rdbuf();
      text = buffer.str();
      if (!text.empty())
      {
        file_loaded = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  json j = json::parse(text);
  return j;
}



class CollisionChecker : public rclcpp::Node
{
  public:
    CollisionChecker() : Node("collision_checker_node")
  {
    service_ = this->create_service<std_srvs::srv::Trigger>(
        "publish_boxes", std::bind(&CollisionChecker::handle_service, this, std::placeholders::_1, std::placeholders::_2));    
  }

private:
    void handle_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = update_collision();  // Update response based on the collision check
    }
    

    bool update_collision() {

    for(int i = 0; i<=1; i++){
        json boxes_dict = open_boxes(i);
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        for (auto mybox : boxes_dict.items()){
            
            // Define a collision object (a box)
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = mybox.key();
            
            std::array<std::array<float,3>,2> mybox_corners = mybox.value();
            std::array<float, 3> min_corner = mybox_corners[0];
            std::array<float, 3> max_corner = mybox_corners[1];
            float size_x = max_corner[0] - min_corner[0];
            float size_y = max_corner[1] - min_corner[1];
            float size_z = max_corner[2] - min_corner[2];
            
            float origin_x = min_corner[0] + size_x/2;
            float origin_y = min_corner[1] + size_y/2;
            float origin_z = min_corner[2] + size_z/2;
            
            // Define the box shape and dimensions
            shape_msgs::msg::SolidPrimitive box;
            box.type = shape_msgs::msg::SolidPrimitive::BOX;
            box.dimensions = {size_x, size_y, size_z};  // x, y, z dimensions

            // Define the pose of the box (positioned in front of the robot)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;  // No rotation
            box_pose.position.x = origin_x;     
            box_pose.position.y = origin_y;
            box_pose.position.z = origin_z;    

            // Add the box shape and pose to the collision object
            collision_object.primitives.push_back(box);
            collision_object.primitive_poses.push_back(box_pose);

            if (i == 0){
            collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
            }
            else{
                collision_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            }
            collision_objects.push_back(collision_object);    
        }
        // Add the collision object into the planning scene
        planning_scene_interface.applyCollisionObjects(collision_objects);
        }
    return true;
    }
    // Add the collision object into the planning scene
    planning_scene_interface.applyCollisionObjects(collision_objects);
  }
  return true;
}

private:
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};



int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CollisionChecker>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

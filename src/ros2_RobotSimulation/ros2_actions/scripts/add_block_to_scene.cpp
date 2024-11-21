#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

void addBlockToPlanningScene(const std::string &block_name, double x, double y, double z) {
    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Define a collision object representing the block
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = block_name;  // Use the name passed as argument

    // Define the block as a box (5cm x 5cm x 5cm)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.05;  // Ensure these match Gazebo dimensions
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.05;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.05;

    // Define the block's pose (matching the pose in Gazebo)
    geometry_msgs::msg::Pose block_pose;
    block_pose.position.x = x;  // Use the x value passed as argument
    block_pose.position.y = y;  // Use the y value passed as argument
    block_pose.position.z = z;  // Use the z value passed as argument
    block_pose.orientation.w = 1.0;

    // Add the primitive and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(block_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Block '%s' added to MoveIt Planning Scene at (%f, %f, %f).", block_name.c_str(), x, y, z);
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::cerr << "Usage: add_block_to_scene <block_name> <x> <y> <z>" << std::endl;
        return 1;
    }

    // Get parameters from command-line arguments
    std::string block_name = argv[1];
    double x = std::atof(argv[2]);
    double y = std::atof(argv[3]);
    double z = std::atof(argv[4]);

    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_block_to_planning_scene_node");

    // Add the block to MoveIt planning scene
    addBlockToPlanningScene(block_name, x, y, z);

    rclcpp::shutdown();
    return 0;
}

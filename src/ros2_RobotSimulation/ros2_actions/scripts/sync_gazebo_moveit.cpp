#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

class GazeboMoveItSync : public rclcpp::Node
{
public:
    GazeboMoveItSync()
        : Node("gazebo_moveit_sync")
    {
        // Subscribe to /gazebo/model_states topic to get the positions of models in Gazebo
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&GazeboMoveItSync::modelStatesCallback, this, std::placeholders::_1));

        // Create the MoveIt planning scene interface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Add tracked block names
        tracked_blocks_ = {"block1"};
    }

private:
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::vector<std::string> tracked_blocks_;

    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // Iterate through all models in Gazebo
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string &model_name = msg->name[i];

            // Check if this model is in the tracked blocks list
            if (std::find(tracked_blocks_.begin(), tracked_blocks_.end(), model_name) != tracked_blocks_.end())
            {
                // Get the pose of the block in Gazebo
                const geometry_msgs::msg::Pose &block_pose = msg->pose[i];

                // Update the block's pose in MoveIt or remove it if it no longer exists
                if (modelExistsInGazebo(model_name, msg))
                {
                    updateBlockPoseInMoveIt(model_name, block_pose);
                }
                else
                {
                    removeBlockFromMoveIt(model_name);
                }
            }
        }
    }

    bool modelExistsInGazebo(const std::string &model_name, const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // Check if the model exists in the Gazebo model states
        return std::find(msg->name.begin(), msg->name.end(), model_name) != msg->name.end();
    }

    void updateBlockPoseInMoveIt(const std::string &block_name, const geometry_msgs::msg::Pose &block_pose)
    {
        // Define the block as a box (5cm x 5cm x 5cm), matching the size in Gazebo
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = block_name;
        collision_object.header.frame_id = "world";

        // Define the block as a box (5cm x 5cm x 5cm)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.05;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.05;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.05;

        // Add the primitive and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(block_pose);
        collision_object.operation = collision_object.ADD;

        // Remove the old object from the scene if it exists, then add the updated one
        planning_scene_interface_->removeCollisionObjects({block_name});
        planning_scene_interface_->applyCollisionObject(collision_object);

        RCLCPP_INFO(this->get_logger(), "Updated %s pose in MoveIt", block_name.c_str());
    }

    void removeBlockFromMoveIt(const std::string &block_name)
    {
        // Remove the block from the MoveIt planning scene
        planning_scene_interface_->removeCollisionObjects({block_name});
        RCLCPP_INFO(this->get_logger(), "Removed %s from MoveIt", block_name.c_str());
    }
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboMoveItSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

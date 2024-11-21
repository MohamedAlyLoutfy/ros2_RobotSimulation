#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <functional>
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros2_data/action/move_xyz.hpp"
#include "pose_messages/msg/cylinder_pose.hpp"

class PoseSubscriberAndActionClient : public rclcpp::Node
{
public:
    using MoveXYZ = ros2_data::action::MoveXYZ;
    using GoalHandleMoveXYZ = rclcpp_action::ClientGoalHandle<MoveXYZ>;

    explicit PoseSubscriberAndActionClient()
    : Node("pose_subscriber_action_client"), is_executing_(false)  // Initialize execution flag to false
    {
        // Create an action client for MoveXYZ action
        this->action_client_ = rclcpp_action::create_client<MoveXYZ>(this, "/MoveXYZ");

        // Ensure the action server is available
        while (!this->action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
        }

        // Subscribe to the topic that provides the pose (x, y, z)
        this->pose_subscription_ = this->create_subscription<pose_messages::msg::CylinderPose>(
            "/cylinder_pose", 10, std::bind(&PoseSubscriberAndActionClient::pose_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<pose_messages::msg::CylinderPose>::SharedPtr pose_subscription_;
    rclcpp_action::Client<MoveXYZ>::SharedPtr action_client_;
    bool is_executing_;  // Flag to track whether a movement is being executed

    // Callback function that is triggered when a new pose is received
    void pose_callback(const pose_messages::msg::CylinderPose::SharedPtr msg)
    {
        // Ignore the pose if the robot is currently executing a movement
        if (is_executing_) {
            RCLCPP_WARN(this->get_logger(), "Robot is currently executing a movement, ignoring pose.");
            return;
        }

        if (msg->positions.size() >= 3) {
            RCLCPP_INFO(this->get_logger(), "Received pose: x=%.2f, y=%.2f, z=%.2f", msg->positions[0], msg->positions[1], msg->positions[2]);

            // Prepare the goal message
            auto goal_msg = MoveXYZ::Goal();
            goal_msg.positionx = msg->positions[0];
            goal_msg.positiony = msg->positions[1];
            goal_msg.positionz = msg->positions[2];
            goal_msg.speed = 1.0;

            // Set the execution flag
            is_executing_ = true;

            // Send the goal to the action server
            send_goal(goal_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Insufficient position data in CylinderPose message.");
        }
    }

    void send_goal(const MoveXYZ::Goal & goal_msg)
    {
        auto goal_options = rclcpp_action::Client<MoveXYZ>::SendGoalOptions();
        goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMoveXYZ> goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server.");
                is_executing_ = false;  // Clear the flag if the goal was rejected
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
            }
        };

        goal_options.result_callback = std::bind(&PoseSubscriberAndActionClient::result_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, goal_options);
    }

    void result_callback(const GoalHandleMoveXYZ::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Action succeeded: %s", result.result->result.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Action was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Action was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }

        // Clear the execution flag once the action is complete
        is_executing_ = false;
    }
};

int main(int argc, char ** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node and spin it
    auto node = std::make_shared<PoseSubscriberAndActionClient>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

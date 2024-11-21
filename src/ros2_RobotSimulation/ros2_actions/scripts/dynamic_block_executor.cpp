#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <functional>
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros2_data/action/move_xyz.hpp"
#include "pose_messages/msg/cylinder_pose.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

class DynamicProgramExecutor : public rclcpp::Node
{
public:
    DynamicProgramExecutor()
        : Node("dynamic_program_executor"), is_executing_(false)
    {
        // Subscribe to the cylinder pose topic
        pose_subscription_ = this->create_subscription<pose_messages::msg::CylinderPose>(
            "/cylinder_pose", 10, std::bind(&DynamicProgramExecutor::pose_callback, this, std::placeholders::_1));

        // Initialize the program template
        initialize_program_template();
    }

private:
    rclcpp::Subscription<pose_messages::msg::CylinderPose>::SharedPtr pose_subscription_;
    std::string program_file_ = "/home/adm-anm-my/projects/ros2_sim/src/ros2_RobotSimulation/ros2_execution/programs/dynamic_program.txt";  // Update this with the actual path
    std::vector<std::string> program_template_;
    bool is_executing_;  // Flag to track whether a program is being executed

    // Initialize the program template
    // Initialize the program template
    void initialize_program_template()
    {
        program_template_ = {
            "{\"action\": \"MoveJs\", \"value\": {\"joint1\": 0.0, \"joint2\": 0.0, \"joint3\": 0.0, \"joint4\": 0.0, \"joint5\": 0.0, \"joint6\": 0.0, \"joint7\": 0.0}, \"speed\": 1.0}",
            "{\"action\": \"GripperClose\"}",
            "{\"action\": \"MoveJs\", \"value\": {\"joint1\": 0.0, \"joint2\": -45.0, \"joint3\": 0.0, \"joint4\": -135.0, \"joint5\": 0.0, \"joint6\": 0.0, \"joint7\": 0.0}, \"speed\": 1.0}",
            "{\"action\": \"GripperOpen\"}",
            "{\"action\": \"MoveXYZ\", \"value\": {\"positionx\": 0.0, \"positiony\": 0.0, \"positionz\": 0.0}, \"speed\": 1.0}",  // Placeholder for object pose

            // New actions for attaching the object and controlling the gripper
            //"{\"action\": \"Attach\", \"value\": {\"object\": \"box\", \"endeffector\": \"end_effector_frame\"}}",
            "{\"action\": \"GripperClose\"}",
            //"{\"action\": \"MoveJs\", \"value\": {\"joint1\": 0.0, \"joint2\": 0.0, \"joint3\": 0.0, \"joint4\": 0.0, \"joint5\": 0.0, \"joint6\": 0.0, \"joint7\": 0.0}, \"speed\": 0.5}"
            //"{\"action\": \"GripperOpen\"}"
            //"{\"action\": \"Detach\", \"value\": {\"object\": \"box\"}}"
        };
    }


    // Callback function for the pose subscription
    void pose_callback(const pose_messages::msg::CylinderPose::SharedPtr msg)
    {
        // Ignore new poses if a program is currently executing
        if (is_executing_) {
            RCLCPP_WARN(this->get_logger(), "Program is currently executing, ignoring new pose.");
            return;
        }

        if (msg->positions.size() >= 3) {
            RCLCPP_INFO(this->get_logger(), "Received pose: x=%.2f, y=%.2f, z=%.2f", msg->positions[0], msg->positions[1], msg->positions[2]);

            // Update the program with the received pose
            update_program_with_pose(msg->positions[0], msg->positions[1], msg->positions[2]+0.12);

            // Set the execution flag to prevent further pose processing
            is_executing_ = true;

            // Execute the updated program
            execute_program();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Insufficient position data in CylinderPose message.");
        }
    }

    // Update the program template with the received pose
    void update_program_with_pose(double x, double y, double z)
    {
        std::ostringstream updated_pose;
        updated_pose << "{\"action\": \"MoveXYZ\", \"value\": {\"positionx\": " << x 
                     << ", \"positiony\": " << y << ", \"positionz\": " << z << "}, \"speed\": 1.0}";

        // Update the second action (MoveXYZ) with the new pose
        program_template_[4] = updated_pose.str();

        // Write the updated program to a file
        std::ofstream file(program_file_);
        if (file.is_open()) {
            for (const auto &line : program_template_) {
                file << line << std::endl;
            }
            file.close();
            RCLCPP_INFO(this->get_logger(), "Updated program saved to %s", program_file_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the program file: %s", program_file_.c_str());
        }
    }

    // Execute the updated program by running the ros2_execution.py script
    void execute_program()
    {
        pid_t pid = fork();
        if (pid == 0) {  // Child process
            // Execute ros2 run command for ros2_execution
            execlp("ros2", "ros2", "run", "ros2_execution", "ros2_execution.py", "--ros-args",
                "-p", "PROGRAM_FILENAME:=dynamic_program",  // Correct format
                "-p", "ROBOT_MODEL:=panda",
                "-p", "EE_MODEL:=panda_hand",
                (char*)NULL);
            _exit(EXIT_FAILURE);  // If execlp fails
        } else if (pid > 0) {  // Parent process
            // Wait for the child process to complete
            int status;
            waitpid(pid, &status, 0);
            if (WIFEXITED(status)) {
                RCLCPP_INFO(this->get_logger(), "Program executed with exit status: %d", WEXITSTATUS(status));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Program execution failed.");
            }

            // Once the execution is done, allow receiving new poses again
            //is_executing_ = false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fork failed.");
        }
    }

};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicProgramExecutor>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

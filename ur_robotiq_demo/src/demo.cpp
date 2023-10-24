#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

std::vector<double> initial_pose = {0.692640, -1.459506, 1.595010, -1.735481, -1.571823, -3.187180};

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("pnp_node");
    // Create a ROS logger
    auto const logger = rclcpp::get_logger("pnp_node");

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });


    // Create the MoveIt Move Group Interface for xarm and gripper
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(10.0); // Making sure that there will be enough time for planning


    // Move to initial position
    move_group.setJointValueTarget(initial_pose);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to the robot using pose goal
    geometry_msgs::msg::PoseStamped target_pose = move_group.getCurrentPose();

    target_pose.pose.position.x = target_pose.pose.position.x-0.1;
    target_pose.pose.position.y = target_pose.pose.position.y-0.1;
    target_pose.pose.position.z = 0.5;

    // Set the target pose
    move_group.setPoseTarget(target_pose, "tool0");
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to initial position
    move_group.setJointValueTarget(initial_pose);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
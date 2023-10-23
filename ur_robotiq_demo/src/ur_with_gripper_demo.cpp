#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "gripper_srv/srv/gripper_service.hpp"


std::vector<double> initial_pose = {0.692640, -1.459506, 1.595010, -1.735481, -1.571823, -3.187180};
geometry_msgs::msg::PoseStamped block_pose;

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("pnp_node");
    auto const gripper_node = std::make_shared<rclcpp::Node>("gripper_node");

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("pnp_node");

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Connecting to Robotiq gripper serveice
    auto gripper_service_client = gripper_node->create_client<gripper_srv::srv::GripperService>("gripper_service");
    while (!gripper_service_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();

    // Create the MoveIt Move Group Interface for xarm and gripper
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setPlanningTime(10.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction;

    geometry_msgs::msg::PoseStamped ur_pose;

    block_pose.pose.position.x = 0.129344;
    block_pose.pose.position.y = 0.541776;
    block_pose.pose.position.z = 0.037411;
    block_pose.pose.orientation.x = 0.115665;
    block_pose.pose.orientation.y = 0.993268;
    block_pose.pose.orientation.z = 0.005760;
    block_pose.pose.orientation.w = 0.002461;

    geometry_msgs::msg::PoseStamped box_pose;
    box_pose.pose.position.x = 0.412808;
    box_pose.pose.position.y = 0.332942;
    box_pose.pose.position.z = 0.200551;
    box_pose.pose.orientation.x = 0.583260;
    box_pose.pose.orientation.y = 0.812264;
    box_pose.pose.orientation.z = 0.004067;
    box_pose.pose.orientation.w = 0.004424;

    // Move to initial position
    move_group.setJointValueTarget(initial_pose);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Gripper open
    request->position = 0;
    request->speed = 255;
    request->force = 255;
    auto response = gripper_service_client->async_send_request(request);
    // executor.spin_until_future_complete(response);
    rclcpp::spin_until_future_complete(gripper_node, response);

    // Move to block
    ur_pose = move_group.getCurrentPose();
    ur_pose.pose = block_pose.pose;
    ur_pose.pose.position.z = block_pose.pose.position.z+0.2;

    waypoints = {};
    waypoints.push_back(ur_pose.pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group.execute(trajectory);
    }

    // Move down
    waypoints = {};
    ur_pose.pose.position.z = ur_pose.pose.position.z-0.2;
    waypoints.push_back(ur_pose.pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group.execute(trajectory);
    }

    // Close gripper
    request->position = 110;
    request->speed = 255;
    request->force = 255;
    response = gripper_service_client->async_send_request(request);
    // executor.spin_until_future_complete(response);
    rclcpp::spin_until_future_complete(gripper_node, response);

    // Move up
    waypoints = {};
    ur_pose.pose.position.z = ur_pose.pose.position.z+0.2;
    waypoints.push_back(ur_pose.pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group.execute(trajectory);
    }

    // Move to initial position
    move_group.setJointValueTarget(initial_pose);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to box
    ur_pose = move_group.getCurrentPose();
    ur_pose.pose= box_pose.pose;
    ur_pose.pose.position.z = box_pose.pose.position.z+0.2;

    waypoints = {};
    waypoints.push_back(ur_pose.pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group.execute(trajectory);
    }

    // Move down
    waypoints = {};
    ur_pose.pose.position.z = ur_pose.pose.position.z-0.2;
    waypoints.push_back(ur_pose.pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group.execute(trajectory);
    }

    // Gripper open
    request->position = 0;
    request->speed = 255;
    request->force = 255;
    response = gripper_service_client->async_send_request(request);
    // executor.spin_until_future_complete(response);
    rclcpp::spin_until_future_complete(gripper_node, response);

    // Move up
    waypoints = {};
    ur_pose.pose.position.z = ur_pose.pose.position.z+0.2;
    waypoints.push_back(ur_pose.pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group.execute(trajectory);
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
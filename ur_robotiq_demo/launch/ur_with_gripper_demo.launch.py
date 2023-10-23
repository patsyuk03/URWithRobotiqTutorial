import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="192.168.0.120"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="false"))

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
 
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': "ur5e",
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': "false",
            'initial_joint_controller': "joint_trajectory_controller",
        }.items(),
    ))

    ld.add_action(TimerAction(period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'launch/demo.launch.py')
            )
        )]
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robotiq_hande_ros2_driver'), 'launch/gripper_bringup.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
        }.items(),
        condition=UnlessCondition(use_fake_hardware)
    ))

    return ld
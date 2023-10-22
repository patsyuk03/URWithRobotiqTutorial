import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
 
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': "ur5e",
            'robot_ip': "192.168.0.120",
            'use_fake_hardware': "true",
            'launch_rviz': "false",
            'initial_joint_controller': "joint_trajectory_controller",
        }.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_moveit_config'), 'launch/ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': "ur5e",
            'use_fake_hardware': "true",
            'launch_rviz': "true",
        }.items(),
    ))

    ld.add_action(Node(
        package="ur_robotiq_demo",
        executable="demo",
        output="screen",
    ))

    return ld
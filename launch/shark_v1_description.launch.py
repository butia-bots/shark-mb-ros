import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Get path to xacro file
    path_to_urdf = get_package_share_path('shark-mb-ros') / 'urdf' / 'shark_v1.xacro'

    # Create a list of nodes
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
            'robot_description': ParameterValue(
            Command(['xacro ', str(path_to_urdf)]), value_type=str
        )
    }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher'
        )
    ]

    # Create and return the launch description
    return LaunchDescription(nodes)


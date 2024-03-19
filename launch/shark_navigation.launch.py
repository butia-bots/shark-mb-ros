import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('shark-mb-ros'), 'config')
    map_file = os.path.join(config_dir, 'map.yaml')
    param_file = os.path.join(config_dir, 'tb3_nav2_params.yaml')
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py')),
            launch_arguments={
                'map':map_file,
                'param_file': param_file}.items(),
        ),

         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])

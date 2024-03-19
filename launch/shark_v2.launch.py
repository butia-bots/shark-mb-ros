import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='shark-mb-ros').find('shark-mb-ros')
    config_dir = os.path.join(get_package_share_directory('shark-mb-ros'), 'rviz')
    urdf_file_path = 'src/description/shark_v1.xacro'
    robot_name_in_model = 'shark-v1'
    rviz_config_dir = os.path.join(config_dir, 'base.rviz')
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    ),

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition= 'true'
        #parameters=[{'use_sim_time': true}],
    ),

    ##adicionar remapping cmd_vel

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    ),

    return launch.LaunchDescription([
    robot_state_publisher_node,
    joint_state_publisher_node,
    urg_front_node,
    urg_back_node,
    rviz_node])
    

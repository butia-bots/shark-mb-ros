import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Get package directories
    path_to_urdf = get_package_share_path('shark-mb-ros') / 'urdf' / 'shark_v1.xacro'

    shark_mb_ros_pkg = get_package_share_directory('shark-mb-ros')
    hoverboard_driver_pkg = get_package_share_directory('hoverboard_driver')
    teleop_twist_joy_pkg = get_package_share_directory('teleop_twist_joy')

    hoverboard_launch_dir = os.path.join(hoverboard_driver_pkg, "launch")

    # Create nodes
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
            Command(['xacro ', str(path_to_urdf)]), value_type=str)
            }]
    )

    joint_state_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher'
        )

    hoverboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([hoverboard_launch_dir, "/diffbot.launch_copy.py"])
    )


    urg_node_front = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node_front',
        #output=LaunchConfiguration('output'),
        parameters=[
            {'frame_id': 'hokuyo_front_link'},
            {'serial_port': '/dev/ttyACM0'},
            {'angle_min': -0.7},
            {'angle_max': 0.7}
        ]
    )

    urg_node_back = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node_back',
        #output=LaunchConfiguration('output'),
        parameters=[
            {'frame_id': 'hokuyo_back_link'},
            {'serial_port': '/dev/ttyACM1'},
            {'angle_min': -0.7},
            {'angle_max': 0.7}
        ],
        remappings=[
            ('/scan', '/scan2')
        ]
    )

    # teleop_twist_joy = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_twist_joy',
    #     output=LaunchConfiguration('output'),
    #     arguments=['joy_config:=atk3']
    # )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        #output=LaunchConfiguration('output'),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', os.path.join(shark_mb_ros_pkg, 'rviz', 'base.rviz')]
    )

    # Define the launch description
    return LaunchDescription([
        joint_state_node,
        use_rviz,
        robot_description,
        hoverboard_launch,
        urg_node_front,
        urg_node_back,
        #teleop_twist_joy,
        rviz
    ])
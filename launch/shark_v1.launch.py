import launch
import launch_ros.actions

def generate_launch_description():
    output = launch.actions.DeclareLaunchArgument('output', default_value='screen')
    use_rviz = launch.actions.DeclareLaunchArgument('use_rviz', default_value='true')

    shark_description = launch_ros.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [find('shark-mb-ros'), 'launch', 'shark_v1_description.launch.py']
        )
    )

    hoverboard_driver = launch_ros.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [find('hoverboard_driver'), 'launch', 'hoverboard.launch.py']
        )
    )

    urg_node_front = launch_ros.actions.Node(
        package='urg_node',
        executable='urg_node',
        name='urg_node_front',
        output='$(arg output)',
        parameters=[
            {'frame_id': 'hokuyo_front_link'},
            {'serial_port': '/dev/ttyURG_H1208745'},
            {'angle_min': -0.7},
            {'angle_max': 0.7}
        ]
    )

    urg_node_back = launch_ros.actions.Node(
        package='urg_node',
        executable='urg_node',
        name='urg_node_back',
        output='$(arg output)',
        parameters=[
            {'frame_id': 'hokuyo_back_link'},
            {'serial_port': '/dev/ttyURG_H1309421'},
            {'angle_min': -0.7},
            {'angle_max': 0.7}
        ],
        remappings=[('/scan', '/scan2')]
    )

    remap_cmd_vel = launch_ros.actions.RemapArguments(
        [
            ('/cmd_vel', '/hoverboard_velocity_controller/cmd_vel')
        ]
    )

    teleop_twist_joy = launch_ros.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [find('teleop_twist_joy'), 'launch', 'teleop.launch.py']
        ),
        launch_arguments={'joy_config': 'atk3'}.items()
    )

    rviz = launch.actions.Group(
        [
            launch.actions.OpaqueFunction(function=launch_setup_rviz)
        ],
        if_condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([
        output,
        use_rviz,
        shark_description,
        hoverboard_driver,
        urg_node_front,
        urg_node_back,
        remap_cmd_vel,
        teleop_twist_joy,
        rviz
    ])

def launch_setup_rviz(context):
    import launch_ros.actions
    return launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', find('shark-mb-ros'), 'rviz', 'base.rviz'],
        output=context.locals['output']
    )

def find(package_name):
    import os
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package_name))


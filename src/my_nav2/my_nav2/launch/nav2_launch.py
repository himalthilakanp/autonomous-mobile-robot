import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('my_nav2')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'config', 'my_map.yaml'),
        description='Full path to map file'
    )

    declare_params = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Nav2 params file'
    )

    # ----------------- Nav2 Nodes -----------------

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_file}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_tree = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        map_server,
        amcl,
        controller,
        planner,
        behavior_tree,
        bt_navigator,
        lifecycle_manager
    ])

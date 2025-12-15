import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to LUA config
    carto_share = get_package_share_directory('cartographer_configs')
    lua_file = os.path.join(carto_share, 'config', 'ms200_imu.lua')

    # -------------------------------
    # 1. CARTOGRAPHER NODE
    # -------------------------------
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.dirname(lua_file),
            '-configuration_basename', os.path.basename(lua_file)
        ],
        remappings=[
            ('scan', '/MS200/scan'),   # Lidar scan topic
            ('imu',  '/imu/data')      # IMU topic
        ]
    )

    # -------------------------------
    # 2. OCCUPANCY GRID NODE (MAP)
    # -------------------------------
    occupancy_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'resolution': 0.05,
                'publish_period_sec': 1.0,
                'map_topic': '/map'
            }
        ]
    )

    # -------------------------------
    # 3. STATIC TRANSFORMS
    # -------------------------------
    # imu_link → base_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # laser_frame → base_link
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # -------------------------------
    # LAUNCH EVERYTHING
    # -------------------------------
    return LaunchDescription([
        carto_node,
        occupancy_node,
        static_tf_imu,
        static_tf_laser
    ])

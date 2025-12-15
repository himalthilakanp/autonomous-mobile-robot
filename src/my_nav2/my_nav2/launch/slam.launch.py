from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # --- SLAM Toolbox Node ---
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',   # works better for realtime robots
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'slam_params_file': ''}  # uses default params
            ],
            remappings=[
                ('scan', '/MS200/scan')   # <-- YOUR LiDAR topic
            ]
        ),

        # --- RViz2 for visualization ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '']
        )
    ])

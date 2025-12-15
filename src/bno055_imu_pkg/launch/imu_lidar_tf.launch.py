from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # base_link → imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=[
                '0', '0', '0',        # x, y, z
                '0', '0', '0',        # roll, pitch, yaw
                'base_link',          # parent
                'imu_link'            # child
            ]
        ),

        # imu_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_laser_tf',
            arguments=[
                '0', '0', '0.095',     # x, y, z (LiDAR above IMU)
                '0', '0', '0',        # roll, pitch, yaw
                'imu_link',           # parent
                'laser_frame'         # child
            ]
        )
    ])

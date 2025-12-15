from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---- Package paths ----
    bno_pkg = get_package_share_directory('bno055_imu_pkg')
    lidar_pkg = get_package_share_directory('oradar_lidar')
    carto_pkg = get_package_share_directory('cartographer_configs')
    nav2_pkg = get_package_share_directory('my_nav2')

    return LaunchDescription([

        # -------- IMU node --------
        Node(
            package='bno055_imu_pkg',
            executable='bno_test',
            name='bno_test',
            output='screen'
        ),

        # -------- IMU + LiDAR TF --------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bno_pkg, 'launch', 'imu_lidar_tf.launch.py')
            )
        ),

        # -------- LiDAR --------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'ms200_scan.launch.py')
            )
        ),

        # -------- Cartographer --------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(carto_pkg, 'launch', 'carto_launch.py')
            )
        ),

        # -------- Nav2 / SLAM --------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'slam.launch.py')
            )
        ),

        # -------- Motor Serial Bridge --------
        Node(
            package='amr_keyboard_control',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        ),

        # -------- Keyboard Teleop --------
        Node(
            package='amr_keyboard_control',
            executable='keyboard_motor_control',
            name='keyboard_motor_control',
            output='screen'
        ),
    ])

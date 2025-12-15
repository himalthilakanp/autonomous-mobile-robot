from setuptools import setup
import os
from glob import glob

package_name = 'bno055_imu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='tony@example.com',
    description='BNO055 IMU ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 🚀 Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'bno_test = bno055_imu_pkg.bno_test:main',
        ],
    },
)

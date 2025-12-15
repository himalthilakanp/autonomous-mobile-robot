from setuptools import setup
import os
from glob import glob

package_name = 'cartographer_configs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(package_name + '/launch/*.py')),
        # Config files (lua)
        (os.path.join('share', package_name, 'config'), glob(package_name + '/config/*.lua')),
        # RViz config
        (os.path.join('share', package_name, 'rviz'), glob(package_name + '/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='tony@example.com',
    description='Cartographer config package for MS200 + BNO055',
    license='Apache License 2.0',
    entry_points={},
)

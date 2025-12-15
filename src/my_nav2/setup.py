from setuptools import setup
from glob import glob
import os

package_name = 'my_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # FIXED PATHS → use my_nav2/config/
        ('share/{}/config'.format(package_name), glob('my_nav2/config/*')),

        # FIXED PATHS → use my_nav2/launch/
        ('share/{}/launch'.format(package_name), glob('my_nav2/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='you@example.com',
    description='Nav2 custom package',
    license='Apache-2.0',
)

from setuptools import find_packages, setup

package_name = 'autonomous_mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/amr_bringup.launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='himalthilakanp96@gmail.com',
    description='AMR bringup package to launch sensors, navigation and motor control',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)

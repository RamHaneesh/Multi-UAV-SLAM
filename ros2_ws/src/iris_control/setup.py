from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'iris_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),

    data_files=[
        # Required for ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share',package_name,'launch'), glob('launch/*.py')),

        # Config (contains mission.json)
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Rudraksha Bandodkar',
    maintainer_email='vigyannveshi@gmail.com',
    description='Control for single Iris UAV in (Gazebo Harmonic)+ROS2+Ardupilot',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'mission_planner = iris_control.mission_planner:main',
            'keyboard_control = iris_control.keyboard_control:main',
        ],
    },
)

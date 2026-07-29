from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'iris_transforms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),

    data_files=[
        # Required for ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # rviz config files
        (os.path.join('share', package_name, 'rviz2_config'), glob('rviz2_config/*.rviz')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Rudraksha Bandodkar',
    maintainer_email='vigyannveshi@gmail.com',
    description='To publish transforms for iris uav',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'odom_base_tf_broadcaster = iris_transforms.odom_base_tf_broadcaster:main',
            'camera_base_tf_broadcaster = iris_transforms.camera_base_tf_broadcaster:main',
            'map_odom_tf_broadcaster = iris_transforms.map_odom_tf_broadcaster:main',
            'ekf3_path_publisher = iris_transforms.ekf3_path_publisher:main',
            'ground_truth_publisher = iris_transforms.ground_truth_publisher:main',
        ],
    },
)

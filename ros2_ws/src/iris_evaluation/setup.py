from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'iris_evaluation'

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
        (os.path.join('share', package_name, 'utils'), glob('utils/*.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Rudraksha Bandodkar',
    maintainer_email='vigyannveshi@gmail.com',
    description='To perform metric evaluations for Iris UAV',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'ground_truth_publisher = iris_evaluation.ground_truth_publisher:main',
        ],
    },
)

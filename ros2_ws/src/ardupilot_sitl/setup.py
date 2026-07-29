from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ardupilot_sitl'

# function to ensure all the internal directories are included in install
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        install_path = os.path.join('share', package_name, path)
        file_list = [os.path.join(path, f) for f in filenames]
        if file_list:
            paths.append((install_path, file_list))
    return paths

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

        # config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ] + package_files('ardusim'), # ardusim

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Rudraksha Bandodkar',
    maintainer_email='vigyannveshi@gmail.com',
    description='Ardupilot SITL, Mavlink, MAVROS setup',
    license='Apache License 2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

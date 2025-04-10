#!/usr/bin/env python3

from setuptools import setup
import os
from glob import glob

package_name = 'ps5_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bninaos',
    maintainer_email='user@example.com',
    description='PS5 controller for quadruped robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps5_teleop = ps5_controller.ps5_teleop:main',
        ],
    },
)

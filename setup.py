import os
import glob
from setuptools import find_packages, setup

package_name = 'marinero_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert',
    maintainer_email='albert.androsic2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_marker = marinero_control.gazebo_marker:main',
            'docking_station = marinero_control.docking_station:main',
            'marinero_teleop = marinero_control.marinero_teleop:main',
            'marinero_control_navigator = marinero_control.marinero_control_navigator:main',
            'marinero_yolo = marinero_control.marinero_yolo:main',
            'marinero_odometry_old_model = marinero_control.marinero_odometry_old_model:main',
            'marinero_odometry_new_model = marinero_control.marinero_odometry_new_model:main',
            'marinero_tracker = marinero_control.marinero_tracker:main',
            'marinero_camera = marinero_control.marinero_camera:main',
            'marinero_camera_w_joy = marinero_control.marinero_camera_w_joy:main',
            'marinero_camera_w_keyboard = marinero_control.marinero_camera_w_keyboard:main'
        ],
    },
)

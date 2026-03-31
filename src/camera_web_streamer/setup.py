from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_web_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinwooseo',
    maintainer_email='jinwooseo@todo.todo',
    description='ROS2 camera web streaming node with Flask MJPEG server',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'web_stream_node = camera_web_streamer.web_stream_node:main',
            'camera_publisher_node = camera_web_streamer.camera_publisher_node:main',
        ],
    },
)

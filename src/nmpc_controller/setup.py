import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nmpc_controller'

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
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinwooseo',
    maintainer_email='jinuseo21@gmail.com',
    description='NMPC trajectory following controller for RC car',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'nmpc_node = nmpc_controller.nmpc_node:main',
        ],
    },
)

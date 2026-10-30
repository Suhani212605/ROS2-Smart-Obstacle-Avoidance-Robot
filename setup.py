import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_obstacle_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suhani S Rao',
    maintainer_email='suhanisrao1ga23is166@gmail.com',
    description='Simulated LiDAR obstacle avoidance on Turtlesim',
    license='MIT',
    entry_points={'console_scripts': [
        'lidar_sim_node = smart_obstacle_avoidance.lidar_sim_node:main',
        'avoidance_node = smart_obstacle_avoidance.avoidance_node:main',
    ]},
)

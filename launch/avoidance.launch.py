"""Starts turtlesim, the simulated LiDAR, the controller, and draws obstacles as parked turtles."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from smart_obstacle_avoidance.core import OBSTACLES


def generate_launch_description():
    params = os.path.join(get_package_share_directory('smart_obstacle_avoidance'),
                          'config', 'params.yaml')
    spawn = [ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                                 f"{{x: {x}, y: {y}, theta: 0.0, name: 'obstacle{i + 1}'}}"])
             for i, (x, y, _r) in enumerate(OBSTACLES)]
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(package='smart_obstacle_avoidance', executable='lidar_sim_node',
             name='lidar_sim_node', parameters=[params], output='screen'),
        Node(package='smart_obstacle_avoidance', executable='avoidance_node',
             name='avoidance_node', parameters=[params], output='screen'),
        TimerAction(period=2.0, actions=spawn),            # wait for turtlesim's /spawn service
    ])

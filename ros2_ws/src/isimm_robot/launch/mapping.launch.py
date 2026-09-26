import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('isimm_robot')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([
        # Robot base: motors, sensors, RPLIDAR, RF2O, TF...
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot.launch.py')
            )
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),

        # Automatically configure + activate SLAM Toolbox
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[
                {
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                }
            ],
        ),
    ])

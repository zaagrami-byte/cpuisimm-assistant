import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('isimm_robot')
    loc_params = os.path.join(pkg, 'config', 'slam_localization_params.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'robot.rviz')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file', default_value=os.path.expanduser('~/maps/isimm_map.posegraph'),
            description='Fichier .posegraph sauvegardé par slam_toolbox'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot.launch.py'))),

        # localisation : slam_toolbox charge la posegraph et publie map->odom + /map
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[loc_params, {'map_file_name': map_file}],
             output='screen'),

        # Nav2 complet (lifecycle managers inclus dans navigation_launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params,
                              'use_sim_time': 'false'}.items()),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_cfg], output='screen'),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'map_file',
            default_value='/home/isimmassistant/isimm/maps/isimm_main.posegraph',
            description='Posegraph SLAM Toolbox utilisée pour la localisation'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Lancer RViz sur la Raspberry Pi'
        ),

        # Robot base:
        # moteurs, ESP32, RPLIDAR, RF2O, TF...
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot.launch.py')
            )
        ),

        # SLAM Toolbox en mode localisation
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                loc_params,
                {
                    'map_file_name': map_file
                }
            ],
            output='screen'
        ),

        # Activation automatique de SLAM Toolbox
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam_localization',
            output='screen',
            parameters=[
                {
                    'autostart': True,
                    'node_names': ['slam_toolbox']
                }
            ]
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup,
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false'
            }.items()
        ),

        # RViz optionnel
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_cfg],
            output='screen'
        ),
    ])

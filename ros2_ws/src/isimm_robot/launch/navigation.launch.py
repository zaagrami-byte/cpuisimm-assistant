import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

NAV2_LIFECYCLE_NODES = [
    'controller_server', 'planner_server', 'behavior_server',
    'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower',
]


def _resolve_map(map_file):
    """slam_toolbox veut le chemin SANS extension (.posegraph + .data)."""
    base = map_file
    for ext in ('.posegraph', '.data'):
        if base.endswith(ext):
            base = base[:-len(ext)]
    for ext in ('.posegraph', '.data'):
        if not os.path.isfile(base + ext):
            raise RuntimeError(
                f'Carte introuvable : {base + ext}. Vérifiez map_file:=... '
                '(sérialisez la carte depuis le mapping avec serialize_map).')
    return base


def _localization(context, *args, **kwargs):
    pkg = get_package_share_directory('isimm_robot')
    loc_params = os.path.join(pkg, 'config', 'slam_localization_params.yaml')
    base = _resolve_map(LaunchConfiguration('map_file').perform(context))

    slam = LifecycleNode(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', namespace='',
        parameters=[loc_params, {'map_file_name': base, 'use_sim_time': False}],
        output='screen')

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[
            LogInfo(msg=f'slam_toolbox (localisation, carte {base}) -> activation'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE)),
        ]))
    return [slam, configure, activate]


def generate_launch_description():
    pkg = get_package_share_directory('isimm_robot')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'robot.rviz')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot.launch.py')),
        launch_arguments={'mode': 'navigation'}.items())

    # Chaîne cmd_vel (aucun chemin direct vers les moteurs) :
    #   controller/behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
    #   -> collision_monitor -> cmd_vel -> safety_node -> safe_cmd_vel -> moteurs
    to_nav = [('cmd_vel', 'cmd_vel_nav')]

    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params], remappings=to_nav),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params], remappings=to_nav),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[nav2_params], remappings=to_nav),   # sortie : cmd_vel_smoothed
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': NAV2_LIFECYCLE_NODES,
                          'bond_timeout': 10.0,
                          'attempt_respawn_reconnection': True}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file', default_value='/home/isimmassistant/isimm/maps/isimm_main',
            description='Posegraph slam_toolbox (avec ou sans .posegraph)'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='RViz sur la Pi (sinon lancez-le sur le PC)'),
        base,
        OpaqueFunction(function=_localization),
        *nav2_nodes,
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', rviz_cfg], output='screen'),
    ])

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

UNIQUE_NODES = ('/safety_node', '/motor_controller_node', '/rplidar_node')


def _guard_duplicates(context, *args, **kwargs):
    """Refuse de démarrer si la base robot tourne déjà (ex: mapping + navigation)."""
    if LaunchConfiguration('check_duplicates').perform(context).lower() != 'true':
        return []
    try:
        out = subprocess.run(
            ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '2'],
            capture_output=True, text=True, timeout=15).stdout.split()
    except Exception:
        return []
    dup = [n for n in UNIQUE_NODES if n in out]
    if dup:
        raise RuntimeError(
            f'Base robot déjà lancée ({dup}). Ne lancez JAMAIS mapping et navigation '
            'ensemble : arrêtez l\'autre launch d\'abord.')
    return []


def generate_launch_description():
    pkg = get_package_share_directory('isimm_robot')
    params = os.path.join(pkg, 'config', 'robot_params.yaml')
    urdf = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    mode = LaunchConfiguration('mode')

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='base',
                              description='base | mapping | navigation (diagnostic)'),
        DeclareLaunchArgument('check_duplicates', default_value='true'),
        OpaqueFunction(function=_guard_duplicates),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description, params], output='screen'),

        Node(package='rplidar_ros', executable='rplidar_composition',
             name='rplidar_node', parameters=[params], output='screen'),

        # RF2O : PAS de name= (le remap __node s'appliquerait aux 2 nœuds du process).
        # Nœud réel : CLaserOdometry2DNode ; paramètres lus dans robot_params.yaml.
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             parameters=[params], output='screen',
             arguments=['--ros-args', '--log-level', 'CLaserOdometry2D:=warn']),

        Node(package='isimm_robot', executable='esp32_sensor_node',
             name='esp32_sensor_node', parameters=[params], output='screen'),

        # Entrée /cmd_vel (sortie de collision_monitor), sortie /safe_cmd_vel. AUCUN remap.
        Node(package='isimm_robot', executable='safety_node',
             name='safety_node', parameters=[params], output='screen'),

        Node(package='isimm_robot', executable='motor_controller_node',
             name='motor_controller_node', parameters=[params], output='screen'),

        Node(package='isimm_robot', executable='robot_status_node',
             name='robot_status_node', parameters=[params, {'mode': mode}],
             output='screen'),
    ])

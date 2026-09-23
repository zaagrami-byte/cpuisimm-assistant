import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('isimm_robot')
    params = os.path.join(pkg, 'config', 'robot_params.yaml')
    urdf = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    use_gui = LaunchConfiguration('use_joint_state_gui')

    robot_description = {
        'robot_description': Command(['xacro ', urdf,
                                      ' use_joint_state_gui:=', use_gui])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_joint_state_gui', default_value='false'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description, params], output='screen'),

        Node(package='rplidar_ros', executable='rplidar_node',
             name='rplidar_node', parameters=[params], output='screen'),

        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry_node', parameters=[params], output='screen'),

        Node(package='isimm_robot', executable='esp32_sensor_node',
             name='esp32_sensor_node', parameters=[params], output='screen'),

        Node(package='isimm_robot', executable='safety_node',
             name='safety_node', parameters=[params], output='screen'),

        Node(package='isimm_robot', executable='motor_controller_node',
             name='motor_controller_node', parameters=[params], output='screen'),

        Node(package='isimm_robot', executable='robot_status_node',
             name='robot_status_node', parameters=[params], output='screen'),
    ])

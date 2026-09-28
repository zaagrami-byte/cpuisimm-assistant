from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Usage autonome (test). Conventions projet : /scan -> /odom, odom -> base_link.
    # PAS de name= : le nœud interne CLaserOdometry2D serait renommé en doublon.
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'CLaserOdometry2D:=warn'],
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0}],
        ),
    ])

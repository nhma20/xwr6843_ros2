from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

username = os.path.expanduser('~')

def generate_launch_description():

    mmwave = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/left_pcl'),
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr6843AOP_profile_10Hz_v2.cfg'},
            {'cli_port': '/dev/radar_left_CLI'},
            {'data_port': '/dev/radar_left_DATA'},
            {'frame_id': 'left_frame'},
            {'radar_azimuth_fov': 140},
            {'radar_elevation_fov': 140},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )


    return LaunchDescription([

        mmwave

    ])

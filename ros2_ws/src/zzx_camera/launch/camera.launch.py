import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='synthetic'),
        DeclareLaunchArgument('allow_device', default_value='false'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='30'),
        Node(package='zzx_camera', executable='camera_manager', namespace='camera',
             output='screen', additional_env={
                 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                 'ROS_LOCALHOST_ONLY': os.environ.get('ROS_LOCALHOST_ONLY', '1'),
                 'CYCLONEDDS_URI': os.environ.get('CYCLONEDDS_URI', 'file://' + str(
                     Path(get_package_share_directory('zzx_camera')) / 'config/cyclonedds_wsl.xml')),
             }, parameters=[{
                 'backend': LaunchConfiguration('backend'),
                 'allow_device': ParameterValue(LaunchConfiguration('allow_device'), value_type=bool),
                 **{key: ParameterValue(LaunchConfiguration(key), value_type=int)
                    for key in ('width', 'height', 'fps')},
             }]),
    ])

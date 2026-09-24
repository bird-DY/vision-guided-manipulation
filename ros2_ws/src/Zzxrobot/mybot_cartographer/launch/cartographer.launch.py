
"""Launch Cartographer and its occupancy-grid publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create Cartographer nodes with explicit, overridable parameters."""
    package_share = get_package_share_directory('mybot_cartographer')
    config_directory = os.path.join(package_share, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period = LaunchConfiguration('publish_period_sec')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_rviz = LaunchConfiguration('use_rviz')

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            configuration_directory,
            '-configuration_basename',
            configuration_basename,
        ],
    )
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution',
            resolution,
            '-publish_period_sec',
            publish_period,
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=config_directory,
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='mybot_2d.lua',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with its default configuration.',
        ),
        cartographer,
        occupancy_grid,
        rviz,
    ])

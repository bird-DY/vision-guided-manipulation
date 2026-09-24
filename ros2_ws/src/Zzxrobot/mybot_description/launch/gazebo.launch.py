"""Launch the Zzxrobot simulation with one authoritative controller map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Create Gazebo, robot-state publisher and controller spawners."""
    description_share = get_package_share_directory('mybot_description')
    moveit_share = get_package_share_directory('mybot')

    model_path = os.path.join(description_share, 'urdf', 'six_arm.gazebo.xacro')
    default_world = os.path.join(description_share, 'world', 'my_world2.world')
    controllers_file = os.path.join(moveit_share, 'config', 'ros2_controllers.yaml')
    world = LaunchConfiguration('world')

    robot_description = xacro.process_file(
        model_path,
        mappings={'controllers_file': controllers_file},
    ).toxml()

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            world,
        ],
        output='screen',
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 15.0},
            {'use_sim_time': True},
        ],
        output='screen',
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mybot', '-topic', 'robot_description'],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'my_group_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world file.',
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[arm_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_spawner,
                on_exit=[gripper_spawner],
            )
        ),
    ])

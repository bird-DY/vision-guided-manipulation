"""Launch exactly one profile. HTTP requires explicit lifecycle activation."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from zzx_http_backend.ownership import CommandLease


def build(context):
    profile = LaunchConfiguration('profile').perform(context)
    resource = LaunchConfiguration('ownership_id').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    with open(Path(get_package_share_directory('zzx_bringup')) / 'config/profiles.json') as stream:
        profiles = json.load(stream)
    if profile not in profiles:
        raise ValueError('profile must be fake, http or sim')
    selected = profiles[profile]
    camera_backend = LaunchConfiguration('camera_backend').perform(context)
    camera_actions = []
    if camera_backend not in ('none', 'synthetic', 'orbbec', 'sdk'):
        raise ValueError('camera_backend must be none, synthetic, orbbec or sdk')
    if camera_backend != 'none':
        camera_path = Path(get_package_share_directory('zzx_camera')) / 'launch/camera.launch.py'
        camera_actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(camera_path)),
            launch_arguments={'backend': camera_backend,
                              'allow_device': LaunchConfiguration('camera_allow_device')}.items())]
    if profile == 'sim':
        if resource != 'zzxrobot':
            raise ValueError('sim uses fixed upstream controller names and requires ownership_id=zzxrobot')
        lease = CommandLease(resource)
        path = Path(get_package_share_directory(selected['package'])) / 'launch' / selected['launch']
        def release(context):
            lease.close()
            return []
        return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=release)])),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))] + camera_actions
    params = {'ownership_id': resource}
    nodes = []
    if profile == 'http':
        arm_port = int(LaunchConfiguration('arm_port').perform(context))
        hand_port = int(LaunchConfiguration('hand_port').perform(context))
        if not 1024 <= arm_port <= 65535 or not 1024 <= hand_port <= 65535 or arm_port == hand_port:
            raise ValueError('choose two distinct ports in [1024,65535]')
        params.update(arm_url=f'http://127.0.0.1:{arm_port}',
                      hand_url=f'http://127.0.0.1:{hand_port}')
        for kind, port in [('arm', arm_port), ('hand', hand_port)]:
            executable = Path(get_package_prefix('zzx_http_contracts')) / 'lib/zzx_http_contracts/competition_http_fake'
            nodes.append(ExecuteProcess(cmd=[str(executable), '--kind', kind, '--port', str(port)],
                                        output='screen'))
    nodes.append(Node(package=selected['package'], executable=selected['executable'],
                      namespace=namespace, parameters=[params], output='screen'))
    actions = []
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
                       on_exit=[EmitEvent(event=Shutdown(reason='managed backend process exited'))])))
    return actions + nodes + camera_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='fake'),
        DeclareLaunchArgument('ownership_id', default_value='zzxrobot'),
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument('arm_port', default_value='18087'),
        DeclareLaunchArgument('hand_port', default_value='18088'),
        DeclareLaunchArgument('camera_backend', default_value='none'),
        DeclareLaunchArgument('camera_allow_device', default_value='false'),
        OpaqueFunction(function=build),
    ])

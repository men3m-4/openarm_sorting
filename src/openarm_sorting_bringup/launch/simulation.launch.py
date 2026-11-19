"""Gazebo simulation of the cup-sorting cell: world, robot, ROS bridges and controllers.

  ros2 launch openarm_sorting_bringup simulation.launch.py              # with Gazebo GUI
  ros2 launch openarm_sorting_bringup simulation.launch.py gz_gui:=false

Sequence (event driven, no fixed sleeps):
  0. pre-flight: refuse to start if another Gazebo server is running (it would own
     /controller_manager); set GZ_SIM_RESOURCE_PATH (+ optional NVIDIA offload)
  1. Gazebo (world running, -r) + robot_state_publisher (robot.launch.py) + ros_gz_bridge
     (/clock and the four RGB-D camera topics, Gazebo -> ROS)
  2. spawn the robot from /robot_description (ros_gz_sim create waits for the topic)
  3. when the spawn process exits successfully -> controllers.launch.py
     (a failed spawn shuts the launch down)

Gazebo exiting (e.g. its window closed) shuts the whole launch down. When the Gazebo
front-end process has exited, any Gazebo server/GUI process of THIS run that is still alive
is stopped (see launch_support.stop_marked_gazebo_processes), so no orphaned simulation keeps
/controller_manager after Ctrl+C.
Included by cup_sorting.launch.py.
"""
import os
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import (RUN_MARKER_VAR, environment_actions, running_gazebo_servers,
                                                    share_path, stop_marked_gazebo_processes)

ROBOT_NAME = 'openarm_v20'
CAMERA_BRIDGES = [
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    # simulated RGB-D sensor (Gazebo -> ROS only)
    '/openarm_zed/image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/openarm_zed/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/openarm_zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    '/openarm_zed/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
]


def _is_gazebo_frontend(action) -> bool:
    """Matches the `gz sim` process started by ros_gz_sim's gz_sim.launch.py (name 'gazebo')."""
    details = getattr(action, 'process_details', None) if isinstance(action, ExecuteProcess) else None
    return bool(details) and details.get('name', '').startswith('gazebo')


def _include(name, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share_path('launch', name)),
        launch_arguments=launch_arguments.items())


def launch_setup(context):
    others = running_gazebo_servers()
    if others:
        return [
            LogInfo(msg='[simulation] ERROR: a Gazebo simulation is already running (it owns '
                        '/controller_manager). Stop it first, then launch again:\n  '
                        + '\n  '.join(others)),
            Shutdown(reason='another Gazebo simulation is already running'),
        ]

    gz_gui = LaunchConfiguration('gz_gui').perform(context).lower() in ('true', '1')
    world = LaunchConfiguration('world').perform(context)
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    nvidia = LaunchConfiguration('nvidia_offload').perform(context)
    if not os.path.isfile(world):
        return [LogInfo(msg=f'[simulation] ERROR: world file not found: {world}'),
                Shutdown(reason='world file not found')]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -r: run immediately (paused -> no camera frames, controller activation times out)
            'gz_args': f'-r {"" if gz_gui else "-s --headless-rendering "}{world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='cup_sorting_bridge',
        output='log', arguments=CAMERA_BRIDGES, parameters=[{'use_sim_time': True}],
        on_exit=Shutdown(reason='ros_gz_bridge exited'),
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', name='spawn_openarm', output='screen',
        arguments=['-topic', '/robot_description', '-name', ROBOT_NAME, '-allow_renaming', 'false'],
    )

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg=f'[simulation] ERROR: spawning the robot failed (exit code {event.returncode})'),
                    Shutdown(reason='robot spawn failed')]
        return [LogInfo(msg='[simulation] robot spawned -> loading the controllers'),
                _include('controllers.launch.py', controllers_file=controllers_file)]

    run_id = uuid.uuid4().hex

    def after_gazebo(_event, _context):
        report = stop_marked_gazebo_processes(run_id)
        return [LogInfo(msg=f'[simulation] {line}') for line in report]

    return environment_actions(nvidia) + [
        SetEnvironmentVariable(RUN_MARKER_VAR, run_id),
        LogInfo(msg=f'[simulation] world: {world} ({"GUI" if gz_gui else "headless"})'),
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=_is_gazebo_frontend, on_exit=after_gazebo)),
        _include('robot.launch.py', controllers_file=controllers_file, use_sim_time='true'),
        bridge,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('world', default_value=share_path('worlds', 'openarm_cup_sorting_world.sdf'),
                              description='Gazebo world (SDF)'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='controller YAML (empty = openarm_sorting_description default)'),
        DeclareLaunchArgument('nvidia_offload', default_value='auto',
                              description='auto|true|false: NVIDIA PRIME render offload for Gazebo/RViz '
                                          '(auto = if the NVIDIA EGL vendor file exists)'),
        OpaqueFunction(function=launch_setup),
    ])

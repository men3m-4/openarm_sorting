"""OpenArm cup sorting - complete application (main entry point).

  ros2 launch openarm_sorting_bringup cup_sorting.launch.py
  ros2 launch openarm_sorting_bringup cup_sorting.launch.py gz_gui:=false rviz:=false   # headless
  ros2 launch openarm_sorting_bringup cup_sorting.launch.py auto_start:=true            # no prompt

Startup (event driven, no fixed sleeps):
  0. pre-flight: refuse to start if another Gazebo simulation is running
  1. simulation.launch.py   Gazebo world + robot_state_publisher + bridges, robot spawn,
                            then the five controllers (controllers.launch.py)
  2. wait_for_controllers   readiness gate: exits when all controllers are ACTIVE
  3. on gate success   ->   moveit.launch.py (move_group), rviz.launch.py, perception.launch.py
                            (cube_detector), coordinator.launch.py (coordinator + task_prompt)
     on gate failure   ->   shutdown with a reason
  4. task_prompt waits until MoveIt, controllers, gripper, coordinator and both cube
     detections are ready, then asks:  Do you want to execute the sorting task? [y/N]
     y/Y -> /cup_sorting/run (red cube, then blue cube, LEFT arm, one cup)
     n/N/Enter -> nothing is executed; the simulation keeps running

Shutdown: Ctrl+C. Gazebo, robot_state_publisher, the bridge, move_group, the detector or the
coordinator exiting also shuts everything down (each with a logged reason). Closing RViz
or the prompt finishing does not.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import controller_names, running_gazebo_servers, share_path


def _include(name, condition=None, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share_path('launch', name)),
        launch_arguments=launch_arguments.items(), condition=condition)


def launch_setup(context):
    # pre-flight before anything is started (simulation.launch.py checks again when used alone)
    others = running_gazebo_servers()
    if others:
        return [LogInfo(msg='[cup_sorting] ERROR: a Gazebo simulation is already running (it owns '
                            '/controller_manager). Stop it first, then launch again:\n  ' + '\n  '.join(others)),
                Shutdown(reason='another Gazebo simulation is already running')]

    cfg = {name: LaunchConfiguration(name).perform(context) for name in (
        'gz_gui', 'controllers_file', 'nvidia_offload', 'controllers_timeout',
        'auto_start', 'prompt', 'detector_output')}
    controllers = controller_names(cfg['controllers_file'] or None)

    gate = Node(
        package='openarm_sorting_bringup', executable='wait_for_controllers',
        name='wait_for_controllers', output='screen',
        arguments=controllers + ['--timeout', cfg['controllers_timeout']],
    )

    application = [
        LogInfo(msg='[cup_sorting] controllers active -> starting MoveIt, RViz, perception, coordinator'),
        _include('moveit.launch.py', controllers_file=cfg['controllers_file']),
        _include('rviz.launch.py', condition=IfCondition(LaunchConfiguration('rviz')),
                 controllers_file=cfg['controllers_file'], nvidia_offload=cfg['nvidia_offload']),
        _include('perception.launch.py', condition=IfCondition(LaunchConfiguration('perception')),
                 output=cfg['detector_output']),
        _include('coordinator.launch.py', condition=IfCondition(LaunchConfiguration('coordinator')),
                 auto_start=cfg['auto_start'], prompt=cfg['prompt'],
                 controllers_file=cfg['controllers_file']),
    ]

    def after_gate(event, _context):
        if event.returncode == 0:
            return application
        if event.returncode < 0:          # stopped by a signal: the launch is already shutting down
            return []
        return [LogInfo(msg='[cup_sorting] ERROR: the controllers did not become active '
                            f'(wait_for_controllers exit code {event.returncode}); shutting down'),
                Shutdown(reason='controllers not active')]

    return [
        _include('simulation.launch.py', gz_gui=cfg['gz_gui'], controllers_file=cfg['controllers_file'],
                 nvidia_offload=cfg['nvidia_offload']),
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_gate)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz (MoveIt + camera)'),
        DeclareLaunchArgument('perception', default_value='true', description='Start the cube detector'),
        DeclareLaunchArgument('coordinator', default_value='true',
                              description='Start the sorting coordinator (and the prompt)'),
        DeclareLaunchArgument('prompt', default_value='true',
                              description='Ask for confirmation before executing the task'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='Execute the task automatically once ready (no prompt)'),
        DeclareLaunchArgument('detector_output', default_value='own_log',
                              description="cube_detector output: own_log (default, keeps the prompt "
                                          "readable) | screen | log"),
        DeclareLaunchArgument('nvidia_offload', default_value='auto', description='auto|true|false'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='controller YAML (empty = openarm_sorting_description default)'),
        DeclareLaunchArgument('controllers_timeout', default_value='240',
                              description='s to wait for all controllers to become active'),
        OpaqueFunction(function=launch_setup),
    ])

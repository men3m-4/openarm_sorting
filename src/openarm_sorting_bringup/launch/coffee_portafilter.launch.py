"""OpenArm coffee workstation - portafilter installation / removal (main entry point).

  ros2 launch openarm_sorting_bringup coffee_portafilter.launch.py
  ros2 launch openarm_sorting_bringup coffee_portafilter.launch.py gz_gui:=false rviz:=false   # headless
  ros2 launch openarm_sorting_bringup coffee_portafilter.launch.py auto_start:=true            # no prompt

Startup (event driven, same structure as cup_sorting.launch.py):
  0. pre-flight: refuse to start if another Gazebo simulation is running
  1. simulation.launch.py  world:=openarm_coffee_world.sdf (robot, bridges, controllers)
  2. wait_for_controllers  readiness gate
  3. on success -> moveit.launch.py, rviz.launch.py (coffee.rviz), coffee_perception.launch.py
                   (ArUco detector + object tracker), coffee_task.launch.py (task + prompt)
  4. the prompt waits for /portafilter_task/ready and asks before anything moves.

Only the Gazebo-specific parts (world, models, bridges, gz_ros2_control) live in the
simulation layer; perception and task nodes are the generic visual_manipulation_* packages.
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
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(share_path('launch', name)),
                                    launch_arguments=launch_arguments.items(), condition=condition)


def launch_setup(context):
    others = running_gazebo_servers()
    if others:
        return [LogInfo(msg='[coffee] ERROR: a Gazebo simulation is already running (it owns '
                            '/controller_manager). Stop it first:\n  ' + '\n  '.join(others)),
                Shutdown(reason='another Gazebo simulation is already running')]
    cfg = {n: LaunchConfiguration(n).perform(context) for n in (
        'gz_gui', 'controllers_file', 'nvidia_offload', 'controllers_timeout', 'auto_start', 'prompt',
        'perception_output', 'world', 'perception_params', 'task_params', 'geometry_file', 'grasp_assist')}
    assist = cfg['grasp_assist'].lower() in ('true', '1')
    gate = Node(package='openarm_sorting_bringup', executable='wait_for_controllers', name='wait_for_controllers',
                output='screen',
                arguments=controller_names(cfg['controllers_file'] or None) + ['--timeout', cfg['controllers_timeout']])
    application = [
        LogInfo(msg='[coffee] controllers active -> starting MoveIt, RViz, perception, portafilter task'),
        _include('moveit.launch.py', controllers_file=cfg['controllers_file']),
        _include('rviz.launch.py', condition=IfCondition(LaunchConfiguration('rviz')), rviz_config='coffee.rviz',
                 controllers_file=cfg['controllers_file'], nvidia_offload=cfg['nvidia_offload']),
        _include('coffee_perception.launch.py', condition=IfCondition(LaunchConfiguration('perception')),
                 output=cfg['perception_output'], perception_params=cfg['perception_params'],
                 geometry_file=cfg['geometry_file']),
        _include('coffee_task.launch.py', condition=IfCondition(LaunchConfiguration('task')),
                 auto_start=cfg['auto_start'], prompt=cfg['prompt'], task_params=cfg['task_params'],
                 geometry_file=cfg['geometry_file']),
    ]

    def after_gate(event, _context):
        if event.returncode == 0:
            return application
        if event.returncode < 0:
            return []
        return [LogInfo(msg=f'[coffee] ERROR: controllers not active (exit code {event.returncode})'),
                Shutdown(reason='controllers not active')]

    sim_layer = []
    if assist:   # Gazebo-only grasp assist (see visual_manipulation_gazebo/grasp_assist.py)
        sim_layer.append(Node(package='visual_manipulation_gazebo', executable='grasp_assist', name='grasp_assist',
                              output='log', parameters=[{'use_sim_time': True}]))
    return sim_layer + [
        _include('simulation.launch.py', gz_gui=cfg['gz_gui'], controllers_file=cfg['controllers_file'],
                 nvidia_offload=cfg['nvidia_offload'], world=cfg['world'],
                 grasp_assist_child_model='portafilter' if assist else ''),
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_gate)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('perception', default_value='true', description='Start ArUco perception'),
        DeclareLaunchArgument('task', default_value='true', description='Start the portafilter task (and prompt)'),
        DeclareLaunchArgument('prompt', default_value='true', description='Ask before executing the task'),
        DeclareLaunchArgument('auto_start', default_value='false', description='Run the task once ready (no prompt)'),
        DeclareLaunchArgument('perception_output', default_value='own_log', description='screen | log | own_log'),
        DeclareLaunchArgument('world', default_value=share_path('worlds', 'openarm_coffee_world.sdf')),
        DeclareLaunchArgument('geometry_file', default_value=share_path('config', 'coffee', 'coffee_geometry.yaml')),
        DeclareLaunchArgument('perception_params', default_value=share_path('config', 'coffee', 'perception.yaml')),
        DeclareLaunchArgument('task_params', default_value=share_path('config', 'coffee', 'task.yaml')),
        DeclareLaunchArgument('grasp_assist', default_value='true',
                              description='Gazebo-only grasp assist (DetachableJoint) for the portafilter; '
                                          'false = pure friction grasp (see docs/coffee/COFFEE_TASK.md)'),
        DeclareLaunchArgument('nvidia_offload', default_value='auto', description='auto|true|false'),
        DeclareLaunchArgument('controllers_file', default_value=''),
        DeclareLaunchArgument('controllers_timeout', default_value='240'),
        OpaqueFunction(function=launch_setup),
    ])

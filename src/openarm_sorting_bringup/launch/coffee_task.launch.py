"""Task layer of the coffee scenario: the generic portafilter task coordinator configured for
the OpenArm (LEFT arm) and, optionally, the operator confirmation prompt.

  ros2 launch openarm_sorting_bringup coffee_task.launch.py              # + y/N prompt
  ros2 launch openarm_sorting_bringup coffee_task.launch.py prompt:=false
  ros2 launch openarm_sorting_bringup coffee_task.launch.py auto_start:=true

Services: /portafilter_task/{run, install, remove, detect, plan_check, go_ready, ready, cancel}
Status:   /portafilter_task/status
Parameters: config/coffee/coffee_geometry.yaml + config/coffee/task.yaml (task_params:=...).
Included by coffee_portafilter.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import share_path


def _true(context, name):
    v = LaunchConfiguration(name).perform(context).strip().lower()
    if v not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name} must be true or false, got {v!r}')
    return v in ('true', '1')


def launch_setup(context):
    auto_start, prompt = _true(context, 'auto_start'), _true(context, 'prompt')
    actions = [Node(
        package='visual_manipulation_tasks', executable='portafilter_task', name='portafilter_task', output='screen',
        parameters=[LaunchConfiguration('geometry_file').perform(context),
                    LaunchConfiguration('task_params').perform(context),
                    {'use_sim_time': _true(context, 'use_sim_time'), 'auto_start': auto_start}],
        on_exit=Shutdown(reason='portafilter_task exited'),
    )]
    if auto_start:
        actions.append(LogInfo(msg='[coffee] auto_start:=true -> the portafilter task starts once everything is ready'))
    elif prompt:
        actions.append(Node(package='visual_manipulation_tasks', executable='task_prompt', name='task_prompt',
                            output='screen', ros_arguments=['--disable-stdout-logs'],
                            parameters=[{'title': 'OpenArm Coffee Workstation - portafilter task'}]))
    else:
        actions.append(LogInfo(msg='[coffee] prompt:=false -> start with:  '
                                   'ros2 service call /portafilter_task/run std_srvs/srv/Trigger'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('prompt', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('geometry_file', default_value=share_path('config', 'coffee', 'coffee_geometry.yaml')),
        DeclareLaunchArgument('task_params', default_value=share_path('config', 'coffee', 'task.yaml')),
        OpaqueFunction(function=launch_setup),
    ])

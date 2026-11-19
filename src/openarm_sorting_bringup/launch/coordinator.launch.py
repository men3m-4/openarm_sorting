"""Task layer: the cup-sorting coordinator (LEFT arm) and the operator confirmation prompt.

  ros2 launch openarm_sorting_bringup coordinator.launch.py            # coordinator + y/N prompt
  ros2 launch openarm_sorting_bringup coordinator.launch.py prompt:=false
  ros2 launch openarm_sorting_bringup coordinator.launch.py auto_start:=true

* cup_sorting_coordinator: offers /cup_sorting/run (red cube, then blue cube) and the step /
  test services; it waits for MoveIt, the controllers and the gripper by itself and never
  moves the robot unless a service is called (or auto_start:=true).
* task_prompt (prompt:=true and auto_start:=false): waits until the coordinator reports ready
  (/cup_sorting/ready) and both cubes are detected, asks
  "Do you want to execute the sorting task? [y/N]" on the terminal and calls /cup_sorting/run
  only after an explicit y/Y. Its exit does not stop anything.

Parameters: openarm_sorting_manipulation/config/{coordinator.yaml, cup_sorting_scene.yaml}.
Included by cup_sorting.launch.py (coordinator:=true).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import controller_names


def _true(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name} must be true or false, got {value!r}')
    return value in ('true', '1')


def launch_setup(context):
    auto_start = _true(context, 'auto_start')
    prompt = _true(context, 'prompt')
    required = controller_names(LaunchConfiguration('controllers_file').perform(context) or None)

    actions = [Node(
        package='openarm_sorting_manipulation', executable='cup_sorting_coordinator',
        name='cup_sorting_coordinator', output='screen',
        parameters=[LaunchConfiguration('coordinator_params').perform(context),
                    LaunchConfiguration('scene_file').perform(context),
                    {'use_sim_time': True, 'auto_start': auto_start, 'required_controllers': required}],
        on_exit=Shutdown(reason='cup_sorting_coordinator exited'),
    )]
    if auto_start:
        actions.append(LogInfo(msg='[coordinator] auto_start:=true -> the sort (red, then blue) starts '
                                   'by itself once the system is ready (no prompt)'))
    elif prompt:
        actions.append(Node(
            package='openarm_sorting_manipulation', executable='task_prompt',
            name='cup_sorting_task_prompt', output='screen',
            # operator output goes straight to the terminal (in order); ROS logs to file + /rosout
            ros_arguments=['--disable-stdout-logs'],
        ))
    else:
        actions.append(LogInfo(msg='[coordinator] prompt:=false -> start the task with:  '
                                   'ros2 service call /cup_sorting/run std_srvs/srv/Trigger'))
    return actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory('openarm_sorting_manipulation'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='true: run the sort automatically once ready (no prompt)'),
        DeclareLaunchArgument('prompt', default_value='true',
                              description='ask "execute the sorting task? [y/N]" once ready'),
        DeclareLaunchArgument('coordinator_params', default_value=os.path.join(config, 'coordinator.yaml')),
        DeclareLaunchArgument('scene_file', default_value=os.path.join(config, 'cup_sorting_scene.yaml'),
                              description='table / cup geometry (must match the Gazebo world)'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='controller YAML whose controllers must be active'),
        OpaqueFunction(function=launch_setup),
    ])

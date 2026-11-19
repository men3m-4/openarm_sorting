"""MoveIt 2 move_group for the simulated OpenArm.

Configuration = the existing upstream openarm_bimanual_moveit_config (v2.0 SRDF, kinematics,
joint limits, moveit_controllers.yaml, OMPL) + the Gazebo robot Xacro + the runtime overrides
in config/moveit_overrides.yaml (upstream files are not modified).

  ros2 launch openarm_sorting_bringup moveit.launch.py     # simulation.launch.py must be running

Included by cup_sorting.launch.py after all controllers are active.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from openarm_sorting_bringup.launch_support import moveit_configs, share_path


def launch_setup(context):
    controllers_file = LaunchConfiguration('controllers_file').perform(context) or None
    config = moveit_configs(controllers_file)
    return [Node(
        package='moveit_ros_move_group', executable='move_group', name='move_group', output='log',
        parameters=[config.to_dict(), share_path('config', 'moveit_overrides.yaml'),
                    {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
        on_exit=Shutdown(reason='move_group exited'),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='controller YAML used in the robot Xacro '
                                          '(empty = openarm_sorting_description default)'),
        OpaqueFunction(function=launch_setup),
    ])

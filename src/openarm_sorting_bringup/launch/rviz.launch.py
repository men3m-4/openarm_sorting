"""RViz for the cup-sorting simulation.

  ros2 launch openarm_sorting_bringup rviz.launch.py                               # MoveIt + camera + detections
  ros2 launch openarm_sorting_bringup rviz.launch.py rviz_config:=camera_view.rviz # camera / point cloud only

`rviz_config` is a file name in this package's rviz/ directory or an absolute path.
RViz gets the MoveIt parameters (robot model, SRDF, kinematics) for the MotionPlanning
display. Closing RViz does NOT stop the simulation.

Included by cup_sorting.launch.py (rviz:=true).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from openarm_sorting_bringup.launch_support import environment_actions, moveit_configs, share_path


def launch_setup(context):
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not os.path.isabs(rviz_config):
        rviz_config = share_path('rviz', rviz_config)
    controllers_file = LaunchConfiguration('controllers_file').perform(context) or None
    config = moveit_configs(controllers_file)
    return environment_actions(LaunchConfiguration('nvidia_offload').perform(context)) + [Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', rviz_config],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, share_path('config', 'moveit_overrides.yaml'),
                    {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value='cup_sorting.rviz',
                              description='cup_sorting.rviz | camera_view.rviz | absolute path'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('controllers_file', default_value=''),
        DeclareLaunchArgument('nvidia_offload', default_value='auto', description='auto|true|false'),
        OpaqueFunction(function=launch_setup),
    ])

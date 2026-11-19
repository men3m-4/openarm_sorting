"""Perception: the red/blue cube detector (OpenCV HSV segmentation + RGB-D depth + TF).

  ros2 launch openarm_sorting_bringup perception.launch.py       # simulation.launch.py must be running
  ros2 launch openarm_sorting_bringup perception.launch.py output:=own_log

Publishes /cup_sorting/{red,blue}_cube/pose (world frame, stable estimates only),
/cup_sorting/{red,blue}_cube/raw, /cup_sorting/debug_image, /cup_sorting/detection_markers.
Parameters: openarm_sorting_vision/config/cube_detector.yaml (or params_file:=...).

`output`: screen | log | own_log. The detector reports 'cube STABLE' every 2 s per cube;
cup_sorting.launch.py uses own_log (the node's own log file in the ROS log directory) so the
operator prompt is not scrolled away.

Included by cup_sorting.launch.py (perception:=true).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('openarm_sorting_vision'),
                                  'config', 'cube_detector.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='cube_detector parameter YAML'),
        DeclareLaunchArgument('output', default_value='screen', description='screen | log | own_log'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='openarm_sorting_vision', executable='cube_detector', name='cube_detector',
            output=LaunchConfiguration('output'),
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            on_exit=Shutdown(reason='cube_detector exited'),
        ),
    ])

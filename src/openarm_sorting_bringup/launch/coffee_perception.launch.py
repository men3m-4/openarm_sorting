"""Perception of the coffee task: ArUco detector + fiducial object tracker (generic nodes of
visual_manipulation_perception) configured for the simulated OpenArm camera.

  ros2 launch openarm_sorting_bringup coffee_perception.launch.py     # simulation must be running

Publishes /aruco_detector/detections, /aruco_detector/debug_image, TF aruco_<id>,
/fiducial_object_tracker/objects, /fiducial_object_tracker/{portafilter,machine_mount}/pose and
TF world -> portafilter, world -> machine_mount.
Parameters: config/coffee/coffee_geometry.yaml (markers, marker -> object transforms) +
config/coffee/perception.yaml (camera topics / frames, filter), or perception_params:=...
Included by coffee_portafilter.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import share_path


def launch_setup(context):
    params = [LaunchConfiguration('geometry_file').perform(context),
              LaunchConfiguration('perception_params').perform(context),
              {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')}]
    out = LaunchConfiguration('output').perform(context)
    return [
        Node(package='visual_manipulation_perception', executable='aruco_detector', name='aruco_detector',
             output=out, parameters=params, on_exit=Shutdown(reason='aruco_detector exited')),
        Node(package='visual_manipulation_perception', executable='fiducial_object_tracker',
             name='fiducial_object_tracker', output=out, parameters=params,
             on_exit=Shutdown(reason='fiducial_object_tracker exited')),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file', default_value=share_path('config', 'coffee', 'coffee_geometry.yaml')),
        DeclareLaunchArgument('perception_params', default_value=share_path('config', 'coffee', 'perception.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output', default_value='screen', description='screen | log | own_log'),
        OpaqueFunction(function=launch_setup),
    ])

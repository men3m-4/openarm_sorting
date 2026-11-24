"""Generic fiducial perception: aruco_detector + fiducial_object_tracker.

  ros2 launch visual_manipulation_perception fiducial_perception.launch.py \
      params_file:=my_camera.yaml [geometry_file:=objects.yaml] [use_sim_time:=false]

`params_file` configures camera topics / frames and the tracker; `geometry_file` (optional)
supplies marker sizes and marker -> object transforms shared with the task (any ROS 2 parameter
file, applied before params_file so params_file can override it).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    files = [f for f in (LaunchConfiguration('geometry_file').perform(context),
                         LaunchConfiguration('params_file').perform(context)) if f]
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    out = LaunchConfiguration('output').perform(context)
    common = files + [{'use_sim_time': sim}]
    return [
        Node(package='visual_manipulation_perception', executable='aruco_detector', name='aruco_detector',
             output=out, parameters=common),
        Node(package='visual_manipulation_perception', executable='fiducial_object_tracker',
             name='fiducial_object_tracker', output=out, parameters=common),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='', description='perception parameter YAML'),
        DeclareLaunchArgument('geometry_file', default_value='', description='object geometry parameter YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('output', default_value='screen'),
        OpaqueFunction(function=launch_setup),
    ])

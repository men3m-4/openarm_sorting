"""rqt_image_view on a camera topic of the running simulation.

  ros2 launch openarm_sorting_bringup image_view.launch.py                              # RGB image
  ros2 launch openarm_sorting_bringup image_view.launch.py image:=/openarm_zed/depth_image
  ros2 launch openarm_sorting_bringup image_view.launch.py image:=/cup_sorting/debug_image

Replaces the `rqt` / `depth` modes of the original view_camera.sh. Started through launch so
the VS Code snap terminal environment is fixed for it (plain `ros2 run rqt_image_view ...`
aborts there with a GLib-GIO settings-schema error).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import snap_terminal_fix_actions


def launch_setup(context):
    return snap_terminal_fix_actions() + [Node(
        package='rqt_image_view', executable='rqt_image_view', name='image_view', output='log',
        arguments=[LaunchConfiguration('image').perform(context)],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image', default_value='/openarm_zed/image',
                              description='image topic, e.g. /openarm_zed/depth_image, /cup_sorting/debug_image'),
        OpaqueFunction(function=launch_setup),
    ])

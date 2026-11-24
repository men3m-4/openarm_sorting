"""robot_state_publisher for the simulated OpenArm v2.0 (Gazebo variant of the description).

Publishes /robot_description (the topic Gazebo spawns the robot from and gz_ros2_control
reads) and the robot TF. The finger collision meshes are replaced by boxes for Gazebo only
(see openarm_sorting_description/gripper_collision_patch.py); MoveIt keeps the exact meshes.

  ros2 launch openarm_sorting_bringup robot.launch.py

Included by simulation.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from openarm_sorting_description import robot_description


def launch_setup(context):
    controllers_file = LaunchConfiguration('controllers_file').perform(context) or None
    assist = LaunchConfiguration('grasp_assist_child_model').perform(context)
    urdf, patch_report = robot_description.gazebo_robot_description(controllers_file, assist)
    return [
        LogInfo(msg='[robot] Gazebo gripper collision patch: ' + '; '.join(patch_report)
                + (f'; grasp assist (DetachableJoint) for model "{assist}"' if assist else '')),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='log',
            parameters=[{
                'robot_description': ParameterValue(urdf, value_type=str),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            on_exit=Shutdown(reason='robot_state_publisher exited'),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='gz_ros2_control controller YAML written into the URDF '
                                          '(empty = openarm_sorting_description/config/gazebo_controllers.yaml)'),
        DeclareLaunchArgument('grasp_assist_child_model', default_value='',
                              description='Gazebo-only grasp assist (DetachableJoint) child model; "" = off'),
        OpaqueFunction(function=launch_setup),
    ])

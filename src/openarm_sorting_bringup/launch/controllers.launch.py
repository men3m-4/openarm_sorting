"""Load and activate the ros2_control controllers of the simulated OpenArm.

ONE spawner process loads + activates every controller declared in the gz_ros2_control YAML
(joint_state_broadcaster, left/right joint trajectory controllers, left/right gripper
controllers) on Gazebo's controller manager. No ros2_control_node is started: the
gz_ros2_control plugin inside Gazebo IS the controller manager. The spawner itself waits for
the /controller_manager services (readiness check, not a sleep).

  ros2 launch openarm_sorting_bringup controllers.launch.py      # Gazebo + robot must be running

Included by simulation.launch.py once the robot has been spawned.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from openarm_sorting_bringup.launch_support import controller_names


def launch_setup(context):
    controllers_file = LaunchConfiguration('controllers_file').perform(context) or None
    manager = LaunchConfiguration('controller_manager').perform(context)
    timeout = LaunchConfiguration('controller_manager_timeout').perform(context)
    names = controller_names(controllers_file)

    spawner = Node(
        package='controller_manager', executable='spawner', name='controller_spawner', output='screen',
        arguments=names + ['--controller-manager', manager, '--controller-manager-timeout', timeout],
    )

    def on_spawner_exit(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg=f'[controllers] active: {", ".join(names)}')]
        return [LogInfo(msg=f'[controllers] ERROR: the controller spawner failed (exit code '
                            f'{event.returncode}); see its output above.')]

    return [
        LogInfo(msg=f'[controllers] loading {len(names)} controllers: {", ".join(names)}'),
        spawner,
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=on_spawner_exit)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='controller YAML (empty = openarm_sorting_description default)'),
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='180',
                              description='s to wait for the controller manager services'),
        OpaqueFunction(function=launch_setup),
    ])

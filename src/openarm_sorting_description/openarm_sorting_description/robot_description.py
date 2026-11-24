"""Build the robot description of the simulated OpenArm v2.0 (used by the bringup launch files).

Two variants of the SAME Xacro are used in the system:

* Gazebo / robot_state_publisher: `gazebo_robot_description()` - the expanded Xacro with the
  finger collision meshes replaced by boxes (see gripper_collision_patch.py for why).
* MoveIt: the unpatched Xacro (`xacro_file()` + `xacro_mappings()`), so collision checking in
  MoveIt keeps the exact finger meshes.
"""
import os
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import xacro

from .gripper_collision_patch import patch_gripper_collisions

PACKAGE = 'openarm_sorting_description'


def share_dir() -> str:
    return get_package_share_directory(PACKAGE)


def xacro_file() -> str:
    """Absolute path of the installed robot Xacro."""
    return os.path.join(share_dir(), 'urdf', 'openarm_v20_gazebo.urdf.xacro')


def default_controllers_file() -> str:
    """ros2_control controller configuration loaded by gz_ros2_control inside Gazebo."""
    return os.path.join(share_dir(), 'config', 'gazebo_controllers.yaml')


def xacro_mappings(controllers_file: Optional[str] = None) -> Dict[str, str]:
    """Xacro arguments shared by the Gazebo and the MoveIt variant of the description."""
    return {'controllers_file': controllers_file or default_controllers_file()}


def gazebo_robot_description(controllers_file: Optional[str] = None,
                             grasp_assist_child_model: str = '') -> Tuple[str, List[str]]:
    """Return (urdf_xml, patch_report) for robot_state_publisher -> Gazebo.
    grasp_assist_child_model: Gazebo-only DetachableJoint grasp assist for that model ('' = off)."""
    mappings = xacro_mappings(controllers_file)
    if grasp_assist_child_model:
        mappings['grasp_assist_child_model'] = grasp_assist_child_model
    urdf = xacro.process_file(xacro_file(), mappings=mappings).toxml()
    return patch_gripper_collisions(
        urdf, {'openarm_description': get_package_share_directory('openarm_description')})

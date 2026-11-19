"""Helpers shared by the launch files of openarm_sorting_bringup.

Nothing here runs robot logic; it only prepares process environments and configuration:

* `controller_names()`        controllers defined in the gz_ros2_control YAML (single source)
* `moveit_configs()`          MoveIt configuration = upstream openarm_bimanual_moveit_config
                              (v2.0) + the Gazebo robot Xacro of openarm_sorting_description
* `environment_actions()`     process environment for Gazebo / RViz (resource path, optional
                              NVIDIA PRIME offload, VS Code snap terminal fix)
* `running_gazebo_servers()`  pre-flight check: only one Gazebo may own /controller_manager
"""
import os
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder
from openarm_sorting_description import robot_description
import yaml

PACKAGE = 'openarm_sorting_bringup'
MOVEIT_PKG = 'openarm_bimanual_moveit_config'
MOVEIT_CFG = 'config/openarm_v2.0'
NVIDIA_EGL_VENDOR = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
# Packages whose meshes the spawned robot references with package:// URIs.
MESH_PACKAGES = ('openarm_description', 'zed_description')


def share_dir() -> str:
    return get_package_share_directory(PACKAGE)


def share_path(*parts: str) -> str:
    return os.path.join(share_dir(), *parts)


# ----------------------------------------------------------------------------- controllers
def controller_names(controllers_file: Optional[str] = None) -> List[str]:
    """Controllers declared under controller_manager in the gz_ros2_control YAML, in file order."""
    path = controllers_file or robot_description.default_controllers_file()
    with open(path) as f:
        params = yaml.safe_load(f)['controller_manager']['ros__parameters']
    return [name for name, value in params.items() if isinstance(value, dict) and 'type' in value]


# ----------------------------------------------------------------------------- MoveIt
def moveit_configs(controllers_file: Optional[str] = None):
    """Existing OpenArm v2.0 MoveIt config (SRDF, kinematics, joint limits, controllers, OMPL)
    with the Gazebo robot description. Runtime overrides live in config/moveit_overrides.yaml."""
    return (
        MoveItConfigsBuilder('openarm', package_name=MOVEIT_PKG)
        .robot_description(file_path=robot_description.xacro_file(),
                           mappings=robot_description.xacro_mappings(controllers_file))
        .robot_description_semantic(file_path=f'{MOVEIT_CFG}/openarm_bimanual.srdf')
        .robot_description_kinematics(file_path=f'{MOVEIT_CFG}/kinematics.yaml')
        .joint_limits(file_path=f'{MOVEIT_CFG}/joint_limits.yaml')
        .trajectory_execution(file_path=f'{MOVEIT_CFG}/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        # robot_state_publisher stays the only /robot_description publisher
        # (gz_ros2_control's controller manager listens to that topic)
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


# ----------------------------------------------------------------------------- environment
def gz_resource_path() -> str:
    """GZ_SIM_RESOURCE_PATH: world models of this package + parents of the mesh packages."""
    paths = [share_path('models')]
    for pkg in MESH_PACKAGES:
        parent = os.path.dirname(get_package_share_directory(pkg))
        if parent not in paths:
            paths.append(parent)
    # keep what the user already had (de-duplicated: launch files may call this more than once)
    for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':'):
        if p and p not in paths:
            paths.append(p)
    return ':'.join(paths)


def nvidia_offload_wanted(mode: str) -> bool:
    """'true' / 'false' / 'auto' (= only if the NVIDIA EGL vendor file is installed)."""
    mode = mode.strip().lower()
    if mode == 'auto':
        return os.path.exists(NVIDIA_EGL_VENDOR)
    return mode in ('true', '1', 'yes')


def environment_actions(nvidia_offload: str = 'auto') -> list:
    """Launch actions that prepare the environment of every process started afterwards."""
    actions = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path())]
    if nvidia_offload_wanted(nvidia_offload):
        # Hybrid-GPU laptops: render Gazebo (camera sensor) and RViz on the NVIDIA GPU.
        actions += [SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
                    SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
                    SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', NVIDIA_EGL_VENDOR)]
    actions += snap_terminal_fix_actions()
    return actions


def snap_terminal_fix_actions() -> list:
    """Terminals inside the VS Code *snap* inherit snap GTK/GIO/locale paths that make rviz2
    fail (/snap/core20 libpthread symbol error) and rqt abort (GLib-GIO settings schema).
    Restore a normal desktop environment for the child processes. No-op outside a snap."""
    if not os.environ.get('SNAP'):
        return []
    actions = []
    for var in ('XDG_DATA_DIRS', 'XDG_CONFIG_DIRS'):
        orig = os.environ.get(f'{var}_VSCODE_SNAP_ORIG')
        if orig:
            actions.append(SetEnvironmentVariable(var, orig))
    for var in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR',
                'GSETTINGS_SCHEMA_DIR', 'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'LOCPATH'):
        if var in os.environ:
            actions.append(UnsetEnvironmentVariable(var))
    actions += [UnsetEnvironmentVariable(v) for v in os.environ if v.startswith('SNAP')]
    return actions


# ----------------------------------------------------------------------------- Gazebo cleanup
# `ros2 launch` signals only its direct child, the `gz` (ruby) front-end. When that front-end
# does not exit within launch's SIGINT timeout it gets SIGTERM and dies WITHOUT stopping the
# `gz sim` server / GUI it started, which then keep running (and keep owning
# /controller_manager). Every process of one simulation run therefore carries a run marker in
# its environment; when the front-end has exited, the marked Gazebo processes are stopped.
RUN_MARKER_VAR = 'OPENARM_SORTING_SIM_RUN'


def _proc_environ(pid: str) -> dict:
    with open(f'/proc/{pid}/environ', 'rb') as f:
        items = [e.split(b'=', 1) for e in f.read().split(b'\0') if b'=' in e]
    return {k.decode(errors='replace'): v.decode(errors='replace') for k, v in items}


def marked_gazebo_processes(run_id: str) -> List[int]:
    pids = []
    uid = os.getuid()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            if int(pid) == os.getpid() or os.stat(f'/proc/{pid}').st_uid != uid:
                continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
            if 'gz sim' in cmd and _proc_environ(pid).get(RUN_MARKER_VAR) == run_id:
                pids.append(int(pid))
        except OSError:
            continue
    return pids


def stop_marked_gazebo_processes(run_id: str, timeout: float = 5.0) -> List[str]:
    """SIGTERM (then SIGKILL after `timeout`) the Gazebo processes of this run. Returns a report."""
    import signal
    import time
    pids = marked_gazebo_processes(run_id)
    if not pids:
        return []
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout
    alive = pids
    while alive and time.monotonic() < deadline:
        time.sleep(0.2)
        alive = [p for p in alive if os.path.exists(f'/proc/{p}') and _is_running(p)]
    for pid in alive:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return [f'stopped {len(pids)} leftover Gazebo process(es) {pids}'
            + (f', {len(alive)} needed SIGKILL' if alive else '')]


def _is_running(pid: int) -> bool:
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'     # zombie = already exited
    except OSError:
        return False


# ----------------------------------------------------------------------------- pre-flight
def running_gazebo_servers() -> List[str]:
    """Command lines of Gazebo server processes of the current user that are already running."""
    found = []
    uid = os.getuid()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            if os.stat(f'/proc/{pid}').st_uid != uid:
                continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                args = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        cmd = ' '.join(args)
        # `gz sim server ...` (split server) or `gz sim [-r] [-s] world.sdf` (front-end / server)
        if 'gz sim' in cmd and ('server' in args or any(a.endswith('.sdf') for a in args)):
            found.append(f'{pid}: {cmd}')
    return found

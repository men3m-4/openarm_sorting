"""Robot-independent client for the standard move_group ROS 2 interfaces (no moveit_py).

  <ns>/compute_ik               moveit_msgs/srv/GetPositionIK        collision-aware IK
  <ns>/compute_cartesian_path   moveit_msgs/srv/GetCartesianPath     straight-line tip paths
  <ns>/move_action              moveit_msgs/action/MoveGroup         OMPL planning (plan only)
  <ns>/execute_trajectory       moveit_msgs/action/ExecuteTrajectory execution
  <ns>/apply_planning_scene     moveit_msgs/srv/ApplyPlanningScene   collision objects

Generalised from openarm_sorting_manipulation/moveit_interface.py: the planning group, tip
link, joint names and frame are parameters, poses are `Transform`s of the TIP LINK.
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PlanningScene, PlanningSceneComponents, RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK
import numpy as np
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from visual_manipulation_core.ros_conversions import pose_from_transform, rotation_to_quaternion_msg
from visual_manipulation_core.se3 import Transform

from .ros_utils import call_service, run_action

ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}


def err_name(code):
    return ERROR_NAMES.get(code, str(code))


def trajectory_duration(traj: RobotTrajectory) -> float:
    pts = traj.joint_trajectory.points
    if not pts:
        return 0.0
    d = pts[-1].time_from_start
    return d.sec + d.nanosec * 1e-9


class MoveItClient:
    def __init__(self, node, group: str, tip_link: str, joint_names, frame: str = 'world', ns: str = '',
                 pipeline: str = 'ompl', seed: int = 7):
        self.node = node
        self.group = group
        self.tip_link = tip_link
        self.joint_names = list(joint_names)
        self.frame = frame
        self.pipeline = pipeline
        self.log = node.get_logger()
        ns = ns.rstrip('/')
        self.ik_cli = node.create_client(GetPositionIK, f'{ns}/compute_ik')
        self.cart_cli = node.create_client(GetCartesianPath, f'{ns}/compute_cartesian_path')
        self.scene_cli = node.create_client(ApplyPlanningScene, f'{ns}/apply_planning_scene')
        self.get_scene_cli = node.create_client(GetPlanningScene, f'{ns}/get_planning_scene')
        self.move_ac = ActionClient(node, MoveGroup, f'{ns}/move_action')
        self.exec_ac = ActionClient(node, ExecuteTrajectory, f'{ns}/execute_trajectory')
        self.velocity_scaling = 0.2
        self.acceleration_scaling = 0.2
        self.cartesian_scaling = 0.6          # extra factor for Cartesian paths
        self.limits = {}                      # joint -> (lower, upper)
        self._rng = np.random.default_rng(seed)

    def ready(self, timeout=1.0) -> bool:
        return (self.ik_cli.wait_for_service(timeout_sec=timeout)
                and self.cart_cli.wait_for_service(timeout_sec=timeout)
                and self.scene_cli.wait_for_service(timeout_sec=timeout)
                and self.move_ac.wait_for_server(timeout_sec=timeout)
                and self.exec_ac.wait_for_server(timeout_sec=timeout))

    # ------------------------------------------------------------------ IK
    def margin(self, sol: dict) -> float:
        """Smallest distance to a joint limit, normalised by the joint range (0 = at a limit)."""
        m = 1.0
        for j, v in sol.items():
            lo, hi = self.limits.get(j, (-math.pi, math.pi))
            m = min(m, (v - lo) / (hi - lo), (hi - v) / (hi - lo))
        return m

    def _seed(self, i, current: JointState) -> JointState:
        if i == 0 or not self.limits:
            return current
        js = JointState()
        js.name = list(self.joint_names)
        js.position = [float(self._rng.uniform(*self.limits[j])) if j in self.limits else 0.0 for j in self.joint_names]
        return js

    def compute_ik(self, tip_pose: Transform, current: JointState, attempts=12, timeout=0.2,
                   avoid_collisions=True, prefer=None):
        """Collision-aware IK; first seed = current state, then random seeds inside the limits.
        Among all solutions returns the one with the largest joint-limit margin, or (if
        `prefer` = {joint: value}) the one closest to it with a usable margin."""
        sols = []
        for i in range(attempts):
            req = GetPositionIK.Request()
            ik = req.ik_request
            ik.group_name = self.group
            ik.robot_state.joint_state = self._seed(i, current)
            ik.robot_state.is_diff = True
            ik.avoid_collisions = avoid_collisions
            ik.ik_link_name = self.tip_link
            ik.pose_stamped = PoseStamped()
            ik.pose_stamped.header.frame_id = self.frame
            ik.pose_stamped.pose = pose_from_transform(tip_pose)
            ik.timeout = Duration(sec=0, nanosec=int(timeout * 1e9))
            res = call_service(self.ik_cli, req, 10.0, 'compute_ik')
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            sols.append({j: sol[j] for j in self.joint_names})
        if not sols:
            return None
        if prefer:
            usable = [s for s in sols if self.margin(s) > 0.05] or sols
            return min(usable, key=lambda s: sum((s[j] - prefer.get(j, s[j])) ** 2 for j in s))
        return max(sols, key=self.margin)

    # ------------------------------------------------------------------ planning
    def plan_to_joints(self, joints: dict, planning_time=5.0, attempts=5, retries=4, path_constraints=None):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.group
        r.num_planning_attempts = attempts
        r.allowed_planning_time = planning_time
        r.max_velocity_scaling_factor = self.velocity_scaling
        r.max_acceleration_scaling_factor = self.acceleration_scaling
        r.pipeline_id = self.pipeline
        r.start_state.is_diff = True
        c = Constraints()
        for name, val in joints.items():
            jc = JointConstraint()
            jc.joint_name, jc.position = name, float(val)
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        if path_constraints is not None:
            r.path_constraints = path_constraints
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        status = 'not planned'
        for attempt in range(1, retries + 1):
            res = run_action(self.move_ac, goal, planning_time + 20.0, 'move_action').result
            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                return res.planned_trajectory, f'SUCCESS (attempt {attempt})'
            status = err_name(res.error_code.val)
            self.log.warn(f'planning attempt {attempt}/{retries} failed: {status}')
        return None, status

    def orientation_constraint(self, tip_orientation: Transform, tol_xyz=(0.3, 0.3, math.pi)) -> Constraints:
        """Path constraint keeping the tip orientation within tolerances (rad, about the tip axes)."""
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.tip_link
        oc.orientation = rotation_to_quaternion_msg(tip_orientation.R)
        oc.absolute_x_axis_tolerance, oc.absolute_y_axis_tolerance, oc.absolute_z_axis_tolerance = tol_xyz
        oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        oc.weight = 1.0
        c = Constraints()
        c.orientation_constraints.append(oc)
        return c

    def plan_cartesian(self, tip_waypoints, max_step=0.005, avoid_collisions=True, min_fraction=0.98,
                       jump_threshold=0.0, path_constraints=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.start_state.is_diff = True
        req.group_name = self.group
        req.link_name = self.tip_link
        req.waypoints = [pose_from_transform(T) for T in tip_waypoints]
        req.max_step = max_step
        req.jump_threshold = jump_threshold
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = self.velocity_scaling * self.cartesian_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling * self.cartesian_scaling
        if path_constraints is not None:
            req.path_constraints = path_constraints
        res = call_service(self.cart_cli, req, 30.0, 'compute_cartesian_path')
        if res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < min_fraction:
            return None, f'{err_name(res.error_code.val)}, fraction {res.fraction:.2f}'
        return res.solution, f'fraction {res.fraction:.2f}'

    def execute(self, trajectory: RobotTrajectory, timeout=90.0):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        res = run_action(self.exec_ac, goal, timeout, 'execute_trajectory').result
        return res.error_code.val == MoveItErrorCodes.SUCCESS, err_name(res.error_code.val)

    def apply_scene(self, scene: PlanningScene) -> bool:
        scene.is_diff = True
        scene.robot_state.is_diff = True
        res = call_service(self.scene_cli, ApplyPlanningScene.Request(scene=scene), 10.0, 'apply_planning_scene')
        return res.success

    def set_allowed_collision(self, name_a: str, name_b: str, allowed: bool) -> bool:
        """Allow / forbid contact between two named bodies (links or collision objects) in the
        Allowed Collision Matrix. A planning-scene diff REPLACES the whole ACM, so the current
        matrix is fetched first (/get_planning_scene), modified and sent back complete."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = call_service(self.get_scene_cli, req, 10.0, 'get_planning_scene').scene.allowed_collision_matrix
        names = list(acm.entry_names)
        for n in (name_a, name_b):
            if n not in names:
                names.append(n)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
        acm.entry_names = names
        i, j = names.index(name_a), names.index(name_b)
        acm.entry_values[i].enabled[j] = allowed
        acm.entry_values[j].enabled[i] = allowed
        scene = PlanningScene()
        scene.allowed_collision_matrix = acm
        return self.apply_scene(scene)

"""Thin client for the standard move_group ROS 2 interfaces (no moveit_py needed).

  /compute_ik                (moveit_msgs/srv/GetPositionIK)       collision-aware IK
  /compute_cartesian_path    (moveit_msgs/srv/GetCartesianPath)    straight-line TCP moves
  /move_action               (moveit_msgs/action/MoveGroup)        OMPL planning (plan only)
  /execute_trajectory        (moveit_msgs/action/ExecuteTrajectory) execution via the
                                                                     existing JTC controller
  /apply_planning_scene      (moveit_msgs/srv/ApplyPlanningScene)  collision objects
"""
import math

import numpy as np
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes, PlanningScene,
                             RobotState, RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState

from .ros_utils import call_service, run_action

ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}


def err_name(code):
    return ERROR_NAMES.get(code, str(code))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def grasp_pose(tcp_xyz, yaw, pitch, tcp_offset):
    """ee_base_link pose that puts the TCP (the fingertip-pad centre, `tcp_offset` along
    ee_base_link -z) at tcp_xyz.

    Orientation = Rz(yaw) * Ry(-pitch):
      * at yaw = pitch = 0 the fingers point straight down (the zero configuration of the
        OpenArm arm hangs with ee_base_link aligned to `world`) and close along world y;
      * `pitch` tilts the fingers forward (towards +x after the yaw) by that angle from vertical;
        the closing axis (ee y) stays horizontal, so the pads stay parallel to the cube faces.
    """
    q = _qmul((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
              (0.0, math.sin(-pitch / 2), 0.0, math.cos(-pitch / 2)))
    # ee z axis in world = Rz(yaw) * (-sin p, 0, cos p); ee_base = tcp + tcp_offset * z_axis
    zx, zy, zz = -math.sin(pitch) * math.cos(yaw), -math.sin(pitch) * math.sin(yaw), math.cos(pitch)
    p = Pose()
    p.position.x = float(tcp_xyz[0] + tcp_offset * zx)
    p.position.y = float(tcp_xyz[1] + tcp_offset * zy)
    p.position.z = float(tcp_xyz[2] + tcp_offset * zz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


class MoveItInterface:
    def __init__(self, node, group, ee_link, frame='world'):
        self.node = node
        self.group = group
        self.ee_link = ee_link
        self.frame = frame
        self.log = node.get_logger()
        self.ik_cli = node.create_client(GetPositionIK, '/compute_ik')
        self.cart_cli = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene_cli = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.move_ac = ActionClient(node, MoveGroup, '/move_action')
        self.exec_ac = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.joint_names = [f'openarm_left_joint{i}' for i in range(1, 8)] if 'left' in group else \
                           [f'openarm_right_joint{i}' for i in range(1, 8)]
        self.velocity_scaling = 0.25
        self.acceleration_scaling = 0.25
        self.limits = {}          # joint -> (lower, upper), filled from /robot_description
        self._rng = np.random.default_rng(7)

    # ---------------------------------------------------------------- readiness
    def ready(self, timeout=1.0):
        return (self.ik_cli.wait_for_service(timeout_sec=timeout)
                and self.cart_cli.wait_for_service(timeout_sec=timeout)
                and self.scene_cli.wait_for_service(timeout_sec=timeout)
                and self.move_ac.wait_for_server(timeout_sec=timeout)
                and self.exec_ac.wait_for_server(timeout_sec=timeout))

    # ---------------------------------------------------------------- IK
    def margin(self, sol):
        """Smallest distance to a joint limit, normalised by the joint range (0 = at a limit)."""
        m = 1.0
        for j, v in sol.items():
            lo, hi = self.limits.get(j, (-math.pi, math.pi))
            m = min(m, (v - lo) / (hi - lo), (hi - v) / (hi - lo))
        return m

    def compute_ik(self, pose: Pose, seed: JointState, attempts=12, timeout=0.2):
        """Collision-aware IK (move_group /compute_ik, KDL). KDL is deterministic for a given
        seed, so the first attempt uses the current state and the others use random seeds
        inside the joint limits. Among all solutions the one farthest from any joint limit is
        returned (a solution hugging a limit leaves no room for the following straight-line
        moves). Returns {joint: value} or None."""
        best, best_cost = None, None
        for i in range(attempts):
            if i > 0 and self.limits:
                seed = JointState()
                seed.name = list(self.joint_names)
                seed.position = [float(self._rng.uniform(*self.limits[j])) if j in self.limits else 0.0
                                 for j in self.joint_names]
            req = GetPositionIK.Request()
            ik = req.ik_request
            ik.group_name = self.group
            ik.robot_state.joint_state = seed
            ik.robot_state.is_diff = True
            ik.avoid_collisions = True
            ik.ik_link_name = self.ee_link
            ik.pose_stamped = PoseStamped()
            ik.pose_stamped.header.frame_id = self.frame
            ik.pose_stamped.pose = pose
            ik.timeout = Duration(sec=0, nanosec=int(timeout * 1e9))
            res = call_service(self.ik_cli, req, 10.0, 'compute_ik')
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            sol = {j: sol[j] for j in self.joint_names}
            cost = -self.margin(sol)
            if best is None or cost < best_cost:
                best, best_cost = sol, cost
        if best is not None:
            self.log.info(f'IK: best normalised joint-limit margin {-best_cost:.2f}')
        return best

    # ---------------------------------------------------------------- planning
    def plan_to_joints(self, joints: dict, planning_time=5.0, attempts=5, retries=5):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.group
        r.num_planning_attempts = attempts
        r.allowed_planning_time = planning_time
        r.max_velocity_scaling_factor = self.velocity_scaling
        r.max_acceleration_scaling_factor = self.acceleration_scaling
        r.pipeline_id = 'ompl'
        r.start_state.is_diff = True
        c = Constraints()
        for name, val in joints.items():
            jc = JointConstraint()
            jc.joint_name, jc.position = name, float(val)
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        # OMPL is randomised: a path may fail MoveIt's post-planning collision validation
        # (e.g. the hand clipping the table edge when leaving the hanging 'home' pose).
        # Such paths are rejected by move_group; simply plan again (never execute an invalid one).
        status = 'not planned'
        for attempt in range(1, retries + 1):
            res = run_action(self.move_ac, goal, planning_time + 20.0, 'move_action').result
            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                return res.planned_trajectory, f'SUCCESS (attempt {attempt})'
            status = err_name(res.error_code.val)
            self.log.warn(f'OMPL attempt {attempt}/{retries} failed: {status}')
        return None, status

    def plan_cartesian(self, waypoints, min_fraction=0.98, avoid_collisions=True, max_step=0.005):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.start_state.is_diff = True
        req.group_name = self.group
        req.link_name = self.ee_link
        req.waypoints = list(waypoints)
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = self.velocity_scaling * 0.6
        req.max_acceleration_scaling_factor = self.acceleration_scaling * 0.6
        res = call_service(self.cart_cli, req, 20.0, 'compute_cartesian_path')
        if res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < min_fraction:
            return None, f'{err_name(res.error_code.val)}, fraction {res.fraction:.2f}'
        return res.solution, f'fraction {res.fraction:.2f}'

    def execute(self, trajectory: RobotTrajectory, timeout=60.0):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        res = run_action(self.exec_ac, goal, timeout, 'execute_trajectory').result
        return res.error_code.val == MoveItErrorCodes.SUCCESS, err_name(res.error_code.val)

    # ---------------------------------------------------------------- scene
    def apply_scene(self, scene: PlanningScene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        res = call_service(self.scene_cli, ApplyPlanningScene.Request(scene=scene), 10.0, 'apply_planning_scene')
        return res.success


def trajectory_duration(traj: RobotTrajectory):
    pts = traj.joint_trajectory.points
    if not pts:
        return 0.0
    d = pts[-1].time_from_start
    return d.sec + d.nanosec * 1e-9


def robot_state_from(js: JointState):
    rs = RobotState()
    rs.joint_state = js
    rs.is_diff = True
    return rs

"""TCP-level arm interface on top of MoveItClient: collision-aware moves of the TOOL CENTRE
POINT with verified execution. Robot-independent (group, tip link, joints, tool transform and
tolerances are configuration).

  * move_tcp(T)        collision-aware IK + OMPL plan + execute + verify
  * cartesian(Ts)      straight-line TCP path through waypoints (collision checked) + verify
  * tcp_pose()         current TCP from TF (planning frame <- tip link) @ flange_to_tcp

"Verified" = after execution every joint is within `tracking_tolerance` of the planned final
point. Many controllers (e.g. Gazebo JTCs with zero goal tolerances) report SUCCEEDED even
if an obstacle blocked the arm, so the joint error is the reliable signal for contact /
collision during insertion moves.
"""
import threading
import time

import rclpy
from sensor_msgs.msg import JointState

from visual_manipulation_core.ros_conversions import transform_from_tf
from visual_manipulation_core.se3 import Transform
from visual_manipulation_core.state_machine import StepFailed

from .moveit_client import MoveItClient, trajectory_duration


class Arm:
    def __init__(self, node, moveit: MoveItClient, flange_to_tcp: Transform, tf_buffer,
                 tracking_tolerance=0.03, settle_timeout=3.0, joint_states_topic='/joint_states'):
        self.node = node
        self.moveit = moveit
        self.flange_to_tcp = flange_to_tcp
        self.tcp_to_flange = flange_to_tcp.inverse()
        self.tf = tf_buffer
        self.track_tol = tracking_tolerance
        self.settle_timeout = settle_timeout
        self.log = node.get_logger()
        self._lock = threading.Lock()
        self._js = {}
        node.create_subscription(JointState, joint_states_topic, self._on_js, 20)
        self.last_tracking_error = 0.0
        # last COMMANDED tcp pose. Under load / with calibration errors the arm settles slightly
        # off the command; closed-loop (visual) corrections must be applied to the command, not
        # to the measured pose, or a constant offset can never be removed.
        self.last_command = None

    # ------------------------------------------------------------------ state
    def _on_js(self, msg):
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._js[n] = p

    def joint_state(self) -> JointState:
        js = JointState()
        with self._lock:
            js.name = list(self._js.keys())
            js.position = list(self._js.values())
        return js

    def joints(self) -> dict:
        with self._lock:
            return {j: self._js[j] for j in self.moveit.joint_names if j in self._js}

    def tcp_pose(self) -> Transform:
        tf = self.tf.lookup_transform(self.moveit.frame, self.moveit.tip_link, rclpy.time.Time())
        return transform_from_tf(tf) @ self.flange_to_tcp

    def command_pose(self) -> Transform:
        """Last commanded TCP pose (falls back to the measured one)."""
        return self.last_command if self.last_command is not None else self.tcp_pose()

    def flange(self, T_tcp: Transform) -> Transform:
        return T_tcp @ self.tcp_to_flange

    # ------------------------------------------------------------------ IK / motions
    def ik_tcp(self, T_tcp: Transform, attempts=12, prefer=None, avoid_collisions=True):
        return self.moveit.compute_ik(self.flange(T_tcp), self.joint_state(), attempts=attempts,
                                      prefer=prefer, avoid_collisions=avoid_collisions)

    def move_joints(self, joints: dict, label: str, path_constraints=None, planning_time=5.0):
        traj, status = self.moveit.plan_to_joints(joints, planning_time=planning_time, path_constraints=path_constraints)
        if traj is None:
            raise StepFailed(f'{label}: planning failed ({status})')
        self.log.info(f'{label}: planned {len(traj.joint_trajectory.points)} pts, '
                      f'{trajectory_duration(traj):.1f} s -> executing')
        self.execute_verified(traj, label)

    def move_tcp(self, T_tcp: Transform, label: str, attempts=12, path_constraints=None, planning_time=5.0):
        sol = self.ik_tcp(T_tcp, attempts)
        if sol is None:
            raise StepFailed(f'{label}: no collision-free IK solution')
        self.log.info(f'{label}: IK joint-limit margin {self.moveit.margin(sol):.2f}')
        self.move_joints(sol, label, path_constraints, planning_time)
        self.last_command = T_tcp

    def cartesian(self, waypoints_tcp, label: str, avoid_collisions=True, min_fraction=0.98, max_step=0.004,
                  path_constraints=None):
        traj, status = self.moveit.plan_cartesian([self.flange(T) for T in waypoints_tcp], max_step=max_step,
                                                  avoid_collisions=avoid_collisions, min_fraction=min_fraction,
                                                  path_constraints=path_constraints)
        if traj is None:
            raise StepFailed(f'{label}: Cartesian path failed ({status})')
        self.execute_verified(traj, label)
        self.last_command = waypoints_tcp[-1]
        self.log.info(f'{label}: done ({status}, {trajectory_duration(traj):.1f} s, '
                      f'tracking error {self.last_tracking_error:.4f} rad)')

    def execute_verified(self, traj, label: str):
        ok, status = self.moveit.execute(traj)
        if not ok:
            raise StepFailed(f'{label}: execution failed ({status})')
        target = dict(zip(traj.joint_trajectory.joint_names, traj.joint_trajectory.points[-1].positions))
        err, t0 = 1e9, time.monotonic()
        while time.monotonic() - t0 < self.settle_timeout:
            cur = self.joints()
            err = max(abs(cur.get(j, 1e9) - v) for j, v in target.items())
            if err < self.track_tol:
                self.last_tracking_error = err
                return
            time.sleep(0.05)
        self.last_tracking_error = err
        raise StepFailed(f'{label}: arm did not reach the planned goal (max joint error {err:.3f} rad > '
                         f'{self.track_tol}); blocked by contact / collision?')

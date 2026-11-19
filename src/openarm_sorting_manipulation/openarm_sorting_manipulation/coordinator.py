"""Cup-sorting coordinator: perception -> MoveIt 2 -> LEFT arm pick-and-place.

Services (std_srvs/srv/Trigger):
  /cup_sorting/plan_check          plan (do NOT execute) the approach to the red cube: Phase-5 test 8
  /cup_sorting/pick_place_red      pick the red cube and place it in the cup
  /cup_sorting/pick_place_blue     pick the blue cube and place it in the cup
  /cup_sorting/run                 red then blue
  /cup_sorting/go_home             plan + execute a collision-free move to the SRDF 'home' state
  /cup_sorting/grasp_test_red|blue approach, grasp, lift (all verified), put the cube back: Phase-5 test 9
  /cup_sorting/go_ready            plan + execute to the ready pose above the table
  /cup_sorting/approach_test_red   ready -> pre-grasp above the red cube -> free-space gripper cycle -> ready
  /cup_sorting/ready               read-only status: success = MoveIt, controllers, gripper and joint
                                   limits available and no sequence running (never moves the robot)
With auto_start:=true the full run starts by itself once everything is ready; otherwise
the operator triggers /cup_sorting/run (task_prompt does this after a y/N confirmation).

Parameters: config/coordinator.yaml (motion, gripper, verification) and
config/cup_sorting_scene.yaml (table / cup geometry, must match the Gazebo world).

Sequence per cube (every step is verified; any failure aborts the sequence safely):
  0. wait for a STABLE camera detection of the cube (frame `world`)
  1. update the planning scene (table, cup, both cubes at their detected poses)
  2. open the gripper (q = open_position), check the measured finger angle
  3. pre-grasp: collision-aware IK for the pitched grasp pose `approach_height` above the cube,
     OMPL plan to that joint goal, execute
  4. remove the target cube from the scene, Cartesian straight-line descent to the grasp pose
  5. close the gripper, VERIFY the grasp from the finger angle (cube between fingers stops
     them at ~0.22 rad for the 4 cm cube; an empty gripper closes to ~0)
  6. attach the cube to the hand in the planning scene, Cartesian lift, VERIFY again
  7. collision-aware IK + OMPL plan to a pose above the cup, execute
  8. Cartesian lowering until the cube bottom is `release_clearance` above the rim
  9. open the gripper, VERIFY that the fingers opened, detach the cube in the scene
 10. Cartesian retreat up, OMPL plan back to the 'ready' pose above the table
 11. VERIFY with the camera that the cube is now inside the cup (or no longer on the table)
"""
import math
import threading
import time
import traceback

import xml.etree.ElementTree as ET

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .gripper import Gripper
from .moveit_interface import MoveItInterface, grasp_pose, trajectory_duration
from . import planning_scene as ps
from .ros_utils import call_service

LEFT_HAND_LINKS = ['openarm_left_ee_base_link', 'openarm_left_ee_link1', 'openarm_left_ee_link2']
# The five controllers loaded into Gazebo's controller manager (all must be active).
REQUIRED_CONTROLLERS = ['joint_state_broadcaster', 'left_joint_trajectory_controller',
                        'right_joint_trajectory_controller', 'left_gripper_controller',
                        'right_gripper_controller']
HOME = {f'openarm_left_joint{i}': 0.0 for i in range(1, 8)}   # SRDF group_state 'home' (left_arm)
# TCP positions (world frame) of the ready pose above the table, tried in this order.
DEFAULT_READY_TCPS = [0.20, 0.28, 0.52, 0.22, 0.22, 0.50, 0.25, 0.30, 0.55]


class StepFailed(RuntimeError):
    pass


class CupSortingCoordinator(Node):

    def __init__(self):
        super().__init__('cup_sorting_coordinator')
        P = self.declare_parameter
        self.frame = P('planning_frame', 'world').value
        self.table_center = list(P('table.center', [0.37, 0.05, 0.2875]).value)
        self.table_size = list(P('table.size', [0.50, 0.80, 0.025]).value)
        self.cup_base = list(P('cup.base', [0.36, 0.32, 0.30]).value)
        self.cup_inner_r = P('cup.inner_radius', 0.055).value
        self.cup_outer_r = P('cup.outer_radius', 0.060).value
        self.cup_h = P('cup.height', 0.095).value
        self.cup_floor = P('cup.floor_thickness', 0.008).value
        self.release_offset = {'red': list(P('cup.release_offset_red', [-0.02, 0.0]).value),
                               'blue': list(P('cup.release_offset_blue', [0.02, 0.0]).value)}
        self.cube = P('cube_size', 0.040).value
        self.auto_start = P('auto_start', False).value
        # gripper / motion parameters
        self.tcp_offset = P('tcp_offset', 0.160).value           # fingertip-pad centre along ee -z (m)
        self.pitch = math.radians(P('grasp_pitch_deg', 60.0).value)  # fingers tilted forward from vertical
        self.grasp_dz = P('grasp_z_offset', 0.013).value         # TCP above cube centre (table clearance)
        self.open_q = P('open_position', 0.45).value
        self.release_q = P('release_position', 0.35).value
        self.closed_q = P('close_position', 0.0).value
        self.grasp_q_min = P('grasp_min_position', 0.12).value   # finger angle window that means
        self.grasp_q_max = P('grasp_max_position', 0.32).value   # "a 4 cm object is held" (~0.22 rad)
        self.approach_h = P('approach_height', 0.10).value
        self.lift_h = P('lift_height', 0.12).value
        self.release_clearance = P('release_clearance', 0.015).value  # cube bottom above rim
        self.velocity_scaling = P('velocity_scaling', 0.20).value
        self.track_tol = P('tracking_tolerance', 0.03).value      # rad, max joint error after a motion
        self.settle_timeout = P('settle_timeout', 3.0).value
        self.finger_contact_tol = P('finger_contact_tolerance', 0.05).value
        self.gripper_max_effort = P('gripper_max_effort', 7.0).value
        flat = [float(v) for v in P('ready_tcp_candidates', DEFAULT_READY_TCPS).value]
        if not flat or len(flat) % 3:
            raise ValueError(f'ready_tcp_candidates needs x, y, z triples, got {flat}')
        self.ready_tcps = [flat[i:i + 3] for i in range(0, len(flat), 3)]
        self.required_controllers = list(P('required_controllers', REQUIRED_CONTROLLERS).value)
        det_ns = P('detection_namespace', '/cup_sorting').value.rstrip('/')

        cb = ReentrantCallbackGroup()
        self.moveit = MoveItInterface(self, 'left_arm', 'openarm_left_ee_base_link', self.frame)
        self.moveit.velocity_scaling = self.moveit.acceleration_scaling = self.velocity_scaling
        self.gripper = Gripper(self, max_effort=self.gripper_max_effort)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.list_ctrl = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._js_lock = threading.Lock()
        self._js = {}
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.create_subscription(String, '/robot_description', self._on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.detections = {}
        for c in ('red', 'blue'):
            self.create_subscription(PoseStamped, f'{det_ns}/{c}_cube/pose',
                                     lambda m, c=c: self._on_det(c, m), 10)

        self._busy = threading.Lock()
        self.attached = set()      # cube ids currently attached to the hand in the planning scene
        for name, fn in [('plan_check', self.plan_check), ('pick_place_red', lambda: self.sort(['red'])),
                         ('pick_place_blue', lambda: self.sort(['blue'])), ('run', lambda: self.sort(['red', 'blue'])),
                         ('go_home', self.go_home), ('go_ready', self.go_ready),
                         ('approach_test_red', lambda: self.approach_test('red')),
                         ('grasp_test_red', lambda: self.grasp_test('red')),
                         ('grasp_test_blue', lambda: self.grasp_test('blue'))]:
            self.create_service(Trigger, f'/cup_sorting/{name}',
                                lambda req, res, fn=fn, name=name: self._trigger(name, fn, res), callback_group=cb)
        self.create_service(Trigger, '/cup_sorting/ready', self._on_ready, callback_group=cb)
        self.get_logger().info('cup sorting coordinator started (LEFT arm). Waiting for MoveIt, controllers, gripper, camera.')
        if self.auto_start:
            threading.Thread(target=self._auto, daemon=True).start()

    # ------------------------------------------------------------------ inputs
    def _on_js(self, msg):
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._js[n] = p

    def _on_description(self, msg):
        limits = {}
        for j in ET.fromstring(msg.data).findall('joint'):
            lim = j.find('limit')
            if j.get('name') in self.moveit.joint_names and lim is not None:
                limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        self.moveit.limits = limits
        self.get_logger().info(f'joint limits loaded for {len(limits)} {self.moveit.group} joints')

    def joint_state(self):
        js = JointState()
        with self._js_lock:
            js.name = list(self._js.keys())
            js.position = list(self._js.values())
        return js

    def _on_det(self, color, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.detections[color] = (time.monotonic(), np.array([p.x, p.y, p.z]), yaw)

    def fresh_detection(self, color, max_age=1.0, timeout=15.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            d = self.detections.get(color)
            if d and time.monotonic() - d[0] < max_age:
                return d[1], d[2]
            time.sleep(0.1)
        return None

    # ------------------------------------------------------------------ readiness
    def missing_conditions(self, timeout=0.5):
        """Readiness conditions that are not met yet (empty list = ready)."""
        missing = []
        if not self.moveit.ready(timeout):
            missing.append('move_group interfaces')
        if not self.gripper.ready(timeout):
            missing.append('left gripper action/joint state')
        if not self.controllers_active():
            missing.append('active controllers')
        if len(self.moveit.limits) < 7:
            missing.append('/robot_description joint limits')
        return missing

    def _on_ready(self, _req, res):
        """/cup_sorting/ready: status query only (used by task_prompt before asking the operator)."""
        missing = self.missing_conditions()
        busy = self._busy.locked()
        res.success = not missing and not busy
        res.message = ('ready' if res.success else
                       'busy: a sequence is running' if not missing else 'waiting for: ' + ', '.join(missing))
        return res

    def wait_ready(self, timeout=300.0):
        t0 = time.monotonic()
        last = ''
        while time.monotonic() - t0 < timeout and rclpy.ok():
            missing = self.missing_conditions()
            if not missing:
                self.get_logger().info(f'READY: move_group, {len(self.required_controllers)} active '
                                       'controllers, gripper available')
                return True
            msg = ', '.join(missing)
            if msg != last:
                self.get_logger().info(f'waiting for: {msg}')
                last = msg
            time.sleep(1.0)
        return False

    def controllers_active(self):
        try:
            res = call_service(self.list_ctrl, ListControllers.Request(), 5.0, 'list_controllers')
        except Exception:  # noqa: BLE001 - controller manager not reachable yet
            return False
        active = {c.name for c in res.controller if c.state == 'active'}
        return all(c in active for c in self.required_controllers)

    # ------------------------------------------------------------------ service plumbing
    def _trigger(self, name, fn, res):
        if not self._busy.acquire(blocking=False):
            res.success, res.message = False, 'busy: another sequence is running'
            return res
        try:
            if not self.wait_ready(timeout=5.0):
                res.success, res.message = False, 'not ready (see coordinator log)'
                return res
            ok, msg = fn()
            res.success, res.message = bool(ok), msg
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'{name} failed: {exc}\n{traceback.format_exc()}')
            res.success, res.message = False, f'{type(exc).__name__}: {exc}'
            self.safe_recover()
        finally:
            self._busy.release()
        return res

    def _auto(self):
        if not self.wait_ready():
            self.get_logger().error('auto_start: system never became ready')
            return
        with self._busy:
            ok, msg = self.sort(['red', 'blue'])
            self.get_logger().info(f'auto_start finished: success={ok}: {msg}')

    # ------------------------------------------------------------------ scene
    def base_scene(self, exclude=()):
        objs = [ps.box('table', self.frame, self.table_center, self.table_size)]
        # keep-out band above the table's front edge (towards the robot): Gazebo position control
        # lags the plan slightly, so planned paths must not skim this edge
        front = self.table_center[0] - self.table_size[0] / 2
        top = self.table_center[2] + self.table_size[2] / 2
        objs.append(ps.box('table_edge_guard', self.frame, [front + 0.01, self.table_center[1], top + 0.02],
                           [0.06, self.table_size[1], 0.04]))
        cup_center = [self.cup_base[0], self.cup_base[1], self.cup_base[2] + self.cup_h / 2]
        # solid cylinder (+5 mm padding): the hand never needs to enter the cup
        objs.append(ps.cylinder('cup', self.frame, cup_center, self.cup_h, self.cup_outer_r + 0.005))
        for c in ('red', 'blue'):
            d = self.detections.get(c)
            if d is None or c in exclude or self.in_cup(d[1]):
                continue
            objs.append(ps.box(f'{c}_cube', self.frame, d[1], [self.cube] * 3,
                               qz=math.sin(d[2] / 2), qw=math.cos(d[2] / 2)))
        return objs

    def in_cup(self, xyz):
        r = math.hypot(xyz[0] - self.cup_base[0], xyz[1] - self.cup_base[1])
        return r < self.cup_inner_r and self.cup_base[2] < xyz[2] < self.cup_base[2] + self.cup_h

    # ------------------------------------------------------------------ motion helpers
    def move_to_pose(self, pose, label):
        sol = self.moveit.compute_ik(pose, self.joint_state())
        if sol is None:
            raise StepFailed(f'{label}: no collision-free IK solution')
        traj, status = self.moveit.plan_to_joints(sol)
        if traj is None:
            raise StepFailed(f'{label}: OMPL planning failed ({status})')
        self.get_logger().info(f'{label}: planned {len(traj.joint_trajectory.points)} points, '
                               f'{trajectory_duration(traj):.1f} s -> executing')
        self.execute_verified(traj, label)

    def move_cartesian(self, pose, label, avoid_collisions=True, watch_fingers=False):
        traj, status = self.moveit.plan_cartesian([pose], avoid_collisions=avoid_collisions)
        if traj is None:
            raise StepFailed(f'{label}: Cartesian path failed ({status})')
        q0 = self.gripper.position
        self.execute_verified(traj, label)
        if watch_fingers and q0 is not None and abs(self.gripper.position - q0) > self.finger_contact_tol:
            raise StepFailed(f'{label}: unexpected finger contact (finger moved {q0:.3f} -> '
                             f'{self.gripper.position:.3f} rad during the approach)')
        self.get_logger().info(f'{label}: done ({status}, {trajectory_duration(traj):.1f} s)')

    def execute_verified(self, traj, label):
        """Execute and VERIFY the robot really arrived. The Gazebo trajectory controllers have
        zero goal tolerances, so a SUCCEEDED result alone does not prove the arm got there (an
        arm blocked by an obstacle also 'succeeds')."""
        ok, status = self.moveit.execute(traj)
        if not ok:
            raise StepFailed(f'{label}: execution failed ({status})')
        target = dict(zip(traj.joint_trajectory.joint_names, traj.joint_trajectory.points[-1].positions))
        err = 1.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.settle_timeout:
            js = self.joint_state()
            cur = dict(zip(js.name, js.position))
            err = max(abs(cur.get(j, 1e9) - v) for j, v in target.items())
            if err < self.track_tol:
                return
            time.sleep(0.1)
        raise StepFailed(f'{label}: arm did not reach the planned goal (max joint error {err:.3f} rad '
                         f'> {self.track_tol}); possible collision or blocked joint')

    def go_home(self):
        traj, status = self.moveit.plan_to_joints(HOME)
        if traj is None:
            return False, f'planning to home failed ({status})'
        try:
            self.execute_verified(traj, 'home')
        except StepFailed as exc:
            return False, str(exc)
        return True, 'home: reached (verified)'

    def go_ready(self):
        """Rest pose ABOVE the table (hand clear of the table edge and outside the camera's view
        of the cubes). Reached by collision-aware IK of a Cartesian target, then OMPL."""
        last = None
        for tcp in self.ready_tcps:
            try:
                self.move_to_pose(self.pose_at(np.array(tcp), 0.0), f'ready {tcp}')
                return True, f'ready: reached {tcp} (verified)'
            except StepFailed as exc:
                last = exc
        return False, f'ready pose unreachable: {last}'

    def grasp_yaws(self, cube_yaw):
        """Closing axis (ee y) along one of the cube's face normals. The pitched grasp approaches
        roughly along +x, so only yaws near 0 are kinematically comfortable; the cube's 90-degree
        symmetry always provides one candidate in (-45, 45] deg."""
        cands = [cube_yaw, cube_yaw + math.pi / 2, cube_yaw - math.pi / 2]
        cands = [(a + math.pi) % (2 * math.pi) - math.pi for a in cands]
        return sorted(set(round(a, 4) for a in cands if abs(a) <= math.radians(50)), key=abs)

    def pose_at(self, tcp, yaw):
        return grasp_pose(tcp, yaw, self.pitch, self.tcp_offset)

    # ------------------------------------------------------------------ Phase 5 test 8
    def plan_check(self):
        """Plan (no execution) the pre-grasp and grasp approach for the red cube."""
        det = self.fresh_detection('red')
        if det is None:
            return False, 'no stable red cube detection'
        xyz, yaw = det
        self.moveit.apply_scene(ps.scene_with(self.base_scene()))
        report = []
        for gy in self.grasp_yaws(yaw):
            pre = self.pose_at(xyz + [0, 0, self.approach_h + self.grasp_dz], gy)
            sol = self.moveit.compute_ik(pre, self.joint_state())
            if sol is None:
                report.append(f'yaw {math.degrees(gy):.0f}: no IK')
                continue
            traj, status = self.moveit.plan_to_joints(sol)
            if traj is None:
                report.append(f'yaw {math.degrees(gy):.0f}: plan failed {status}')
                continue
            q = ', '.join(f'{v:+.2f}' for v in sol.values())
            msg = (f'red cube at ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}); pre-grasp yaw {math.degrees(gy):.0f} deg; '
                   f'IK [{q}]; OMPL plan {len(traj.joint_trajectory.points)} pts / '
                   f'{trajectory_duration(traj):.1f} s. NOT executed.')
            self.get_logger().info('PLAN CHECK OK: ' + msg)
            return True, msg
        return False, '; '.join(report)

    # ------------------------------------------------------------------ main sequence
    def sort(self, colors):
        results = []
        for color in colors:
            try:
                self.pick_and_place(color)
                results.append(f'{color}: placed in cup (verified)')
            except StepFailed as exc:
                self.get_logger().error(f'{color}: {exc}')
                results.append(f'{color}: FAILED - {exc}')
                self.safe_recover()
                return False, ' | '.join(results)
        return True, ' | '.join(results)

    def safe_recover(self):
        """After a failure: keep the gripper as it is (a held cube is not dropped), move the
        hand straight up 6 cm if possible, then plan to the ready pose above the table."""
        log = self.get_logger()
        try:
            self.moveit.apply_scene(ps.scene_with(self.base_scene()))
            fk_pose = self.current_tcp_pose()
            if fk_pose is not None:
                try:
                    up = fk_pose
                    up.position.z += 0.06
                    traj, st = self.moveit.plan_cartesian([up], min_fraction=0.5)
                    if traj is not None:
                        self.moveit.execute(traj)
                except Exception as exc:  # noqa: BLE001
                    log.warn(f'recovery: retreat up skipped ({exc})')
            ok, st = self.go_ready()
            log.warn(f'recovery: {st}')
        except Exception as exc:  # noqa: BLE001
            log.error(f'recovery failed: {exc}')

    def current_tcp_pose(self):
        """Current ee_base_link pose in the planning frame (TF from robot_state_publisher)."""
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, 'openarm_left_ee_base_link', rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF not available yet
            return None
        p = Pose()
        t, q = tf.transform.translation, tf.transform.rotation
        p.position.x, p.position.y, p.position.z = t.x, t.y, t.z
        p.orientation = q
        return p

    def approach_test(self, color):
        """Stage test: ready pose -> pre-grasp above the cube (verified), gripper open/close
        cycle in free space (verified), back to ready. Touches nothing."""
        try:
            det = self.fresh_detection(color)
            if det is None:
                return False, 'no stable detection'
            xyz, yaw = det
            self.moveit.apply_scene(ps.scene_with(self.base_scene()))
            ok, st = self.go_ready()
            if not ok:
                return False, st
            gy = self.grasp_yaws(yaw)[0]
            self.move_to_pose(self.pose_at(xyz + [0, 0, self.approach_h + self.grasp_dz], gy), f'[{color}] pre-grasp')
            q_open, _, _ = self.gripper.command(self.open_q)
            q_closed = self.gripper.squeeze(self.closed_q, settle=2.5)
            q_open2, _, _ = self.gripper.command(self.open_q)
            ok, st = self.go_ready()
            msg = (f'pre-grasp reached (verified); free-space gripper cycle open {q_open:.3f} -> closed '
                   f'{q_closed:.3f} -> open {q_open2:.3f} rad; {st}')
            return (abs(q_closed) < 0.05 and q_open2 > self.open_q - 0.05 and ok), msg
        except StepFailed as exc:
            self.safe_recover()
            return False, f'approach test FAILED - {exc}'

    def pick_and_place(self, color):
        picked = self.pick(color)
        if picked is not None:
            self.place_in_cup(color, picked[1])

    def grasp_test(self, color):
        """Phase-5 test 9: approach, grasp, verify, lift, verify - then put the cube back
        where it was, release it and return home. Nothing is placed in the cup."""
        try:
            picked = self.pick(color)
            if picked is None:
                return False, f'{color} cube is already in the cup'
            xyz, grasp_yaw = picked
            self.put_back(color, xyz, grasp_yaw)
        except StepFailed as exc:
            self.get_logger().error(f'grasp test {color}: {exc}')
            self.safe_recover()
            return False, f'grasp test FAILED - {exc}'
        return True, f'{color}: grasp + lift verified, cube put back and released, arm at ready pose'

    def put_back(self, color, xyz, grasp_yaw):
        log = self.get_logger()
        cube_id = f'{color}_cube'
        self.move_cartesian(self.pose_at(xyz + [0, 0, self.grasp_dz + 0.003], grasp_yaw), f'[{color}] T1. lower back to the table')
        self.gripper.command(self.open_q)
        time.sleep(0.8)
        if self.gripper.position < self.open_q - 0.05:
            raise StepFailed(f'gripper did not open (finger {self.gripper.position:.3f})')
        self.moveit.apply_scene(ps.scene_with(attached=[ps.detach(cube_id, LEFT_HAND_LINKS[0], self.frame)],
                                              objects=[ps.remove(cube_id, self.frame)]))
        self.attached.discard(cube_id)
        self.move_cartesian(self.pose_at(xyz + [0, 0, self.approach_h + self.grasp_dz], grasp_yaw), f'[{color}] T2. retreat up')
        ok, st = self.go_ready()
        if not ok:
            raise StepFailed(st)
        log.info(f'[{color}] T3. cube released on the table, arm at the ready pose')

    def pick(self, color):
        """Steps 0-6. Returns (cube_xyz, grasp_yaw) with the cube held and lifted, or None
        if the cube is already in the cup."""
        log = self.get_logger()
        cube_id = f'{color}_cube'

        # 0. perception ------------------------------------------------------------
        det = self.fresh_detection(color)
        if det is None:
            raise StepFailed('no stable camera detection')
        xyz, yaw = det
        if self.in_cup(xyz):
            log.info(f'{color} cube already inside the cup - skipping')
            return None
        log.info(f'[{color}] 0. detected at ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}), yaw {math.degrees(yaw):.1f} deg')

        # 1. planning scene ----------------------------------------------------------
        stale = [ps.detach(c, LEFT_HAND_LINKS[0], self.frame) for c in self.attached]   # e.g. after a failed run
        self.attached.clear()
        self.moveit.apply_scene(ps.scene_with(self.base_scene(), attached=stale))

        # 1b. start from the ready pose above the table (never from below the table edge)
        ok, st = self.go_ready()
        if not ok:
            raise StepFailed(st)

        # 2. open gripper --------------------------------------------------------------
        q, _, _ = self.gripper.command(self.open_q)
        time.sleep(0.3)
        if self.gripper.position is None or self.gripper.position < self.open_q - 0.05:
            raise StepFailed(f'gripper did not open (finger at {self.gripper.position})')
        log.info(f'[{color}] 2. gripper open, finger {self.gripper.position:.3f} rad')

        # 3. pre-grasp -----------------------------------------------------------------
        last_err = None
        for gy in self.grasp_yaws(yaw):
            try:
                self.move_to_pose(self.pose_at(xyz + [0, 0, self.approach_h + self.grasp_dz], gy),
                                  f'[{color}] 3. pre-grasp (yaw {math.degrees(gy):.0f} deg)')
                grasp_yaw = gy
                break
            except StepFailed as exc:
                last_err = exc
        else:
            raise StepFailed(f'pre-grasp unreachable: {last_err}')

        # 4. descend (cube removed from the collision world so the fingers may surround it) ---
        det = self.fresh_detection(color, timeout=3.0)   # re-measure if visible; else keep the old one
        if det is not None and np.linalg.norm(det[0][:2] - xyz[:2]) < 0.02:
            xyz = det[0]
        self.moveit.apply_scene(ps.scene_with([ps.remove(cube_id, self.frame)]))
        self.move_cartesian(self.pose_at(xyz + [0, 0, self.grasp_dz], grasp_yaw), f'[{color}] 4. descend to grasp',
                            watch_fingers=True)

        # 5. close + verify -----------------------------------------------------------
        qm = self.gripper.squeeze(self.closed_q, settle=3.0)
        log.info(f'[{color}] 5. close: finger settled at {qm:.3f} rad')
        if not (self.grasp_q_min <= qm <= self.grasp_q_max):
            self.gripper.command(self.open_q)
            raise StepFailed(f'grasp NOT verified: finger angle {qm:.3f} rad outside '
                             f'[{self.grasp_q_min}, {self.grasp_q_max}] (empty gripper closes to ~0)')
        log.info(f'[{color}] 5. GRASP VERIFIED (finger stopped at {qm:.3f} rad, expected ~0.22 for 4 cm)')

        # 6. attach + lift + verify ------------------------------------------------------
        held = ps.box(cube_id, self.frame, xyz, [self.cube - 0.005] * 3,  # 5 mm smaller: not touching the table
                      qz=math.sin(grasp_yaw / 2), qw=math.cos(grasp_yaw / 2))
        self.moveit.apply_scene(ps.scene_with(attached=[ps.attach(held, LEFT_HAND_LINKS[0], LEFT_HAND_LINKS)]))
        self.attached.add(cube_id)
        self.move_cartesian(self.pose_at(xyz + [0, 0, self.lift_h + self.grasp_dz], grasp_yaw), f'[{color}] 6. lift')
        time.sleep(0.5)
        qm2 = self.gripper.position
        if not (self.grasp_q_min <= qm2 <= self.grasp_q_max):
            raise StepFailed(f'cube lost during lift (finger closed to {qm2:.3f} rad)')
        log.info(f'[{color}] 6. lift verified, still holding (finger {qm2:.3f} rad)')
        return xyz, grasp_yaw

    def place_in_cup(self, color, grasp_yaw):
        """Steps 7-11 (cube held and lifted)."""
        log = self.get_logger()
        cube_id = f'{color}_cube'
        last_err = None

        # 7. above the cup -----------------------------------------------------------------
        off = self.release_offset[color]
        rim = self.cup_base[2] + self.cup_h
        release_tcp = np.array([self.cup_base[0] + off[0], self.cup_base[1] + off[1],
                                rim + self.release_clearance + self.cube / 2 + self.grasp_dz])
        above = release_tcp + [0, 0, 0.06]
        moved = False
        for yaw_c in [grasp_yaw] + [a for a in (0.0, math.radians(20), math.radians(-20)) if abs(a - grasp_yaw) > 0.1]:
            try:
                self.move_to_pose(self.pose_at(above, yaw_c), f'[{color}] 7. above cup (yaw {math.degrees(yaw_c):.0f})')
                place_yaw, moved = yaw_c, True
                break
            except StepFailed as exc:
                last_err = exc
        if not moved:
            raise StepFailed(f'cannot reach above the cup: {last_err}')
        if not (self.grasp_q_min <= self.gripper.position <= self.grasp_q_max):
            raise StepFailed(f'cube lost during transport (finger {self.gripper.position:.3f} rad)')

        # 8. lower into the cup -------------------------------------------------------------
        self.move_cartesian(self.pose_at(release_tcp, place_yaw), f'[{color}] 8. lower into cup')

        # 9. release + verify ----------------------------------------------------------------
        q, reached, _ = self.gripper.command(self.release_q)
        time.sleep(0.8)
        if self.gripper.position < self.release_q - 0.05:
            raise StepFailed(f'gripper did not open for release (finger {self.gripper.position:.3f})')
        self.moveit.apply_scene(ps.scene_with(attached=[ps.detach(cube_id, LEFT_HAND_LINKS[0], self.frame)],
                                              objects=[ps.remove(cube_id, self.frame)]))
        self.attached.discard(cube_id)
        log.info(f'[{color}] 9. released (finger {self.gripper.position:.3f} rad)')

        # 10. retreat + home -------------------------------------------------------------------
        self.move_cartesian(self.pose_at(release_tcp + [0, 0, 0.08], place_yaw), f'[{color}] 10. retreat')
        ok, st = self.go_ready()
        if not ok:
            raise StepFailed(st)
        log.info(f'[{color}] 10. back at the ready pose')

        # 11. verify with the camera -----------------------------------------------------------
        self.detections.pop(color, None)
        det = self.fresh_detection(color, timeout=6.0)
        if det is not None and not self.in_cup(det[0]):
            raise StepFailed(f'cube still seen OUTSIDE the cup at {np.round(det[0], 3).tolist()}')
        where = 'seen inside the cup' if det is not None else 'no longer visible on the table (inside cup, occluded by the rim)'
        log.info(f'[{color}] 11. PLACEMENT VERIFIED by camera: {where}')


def main(args=None):
    rclpy.init(args=args)
    node = CupSortingCoordinator()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""portafilter_task: vision-guided installation and removal of an espresso portafilter.

Robot-, camera- and simulator-independent task coordinator. Everything specific is a ROS
parameter (robot: planning group, tip link, joints, tool transform, gripper action; objects:
marker -> object transforms, collision shapes, bayonet description; task: grasp candidates,
approach / retreat motions, servo and verification tolerances). It talks only to standard
interfaces:
  perception   visual_manipulation_interfaces/TrackedObjectArray  (objects_topic)
               std_srvs/Trigger <tracker>/reset                     (fresh measurements)
  motion       move_group: /compute_ik, /compute_cartesian_path, /move_action,
               /execute_trajectory, /apply_planning_scene
  gripper      control_msgs/action/GripperCommand + sensor_msgs/JointState
  robot state  TF2 (planning frame <- tip link), /joint_states, /robot_description (limits)
  controllers  controller_manager_msgs/srv/ListControllers (readiness only, optional)

State machine (each state verified; failures retry or go to RECOVER):

  DETECT -> ESTIMATE_POSE -> PLAN_GRASP -> APPROACH -> GRASP -> VERIFY_GRASP -> TRANSPORT
  -> ALIGN -> INSERT -> ROTATE_TO_LOCK -> VERIFY_INSTALLATION -> RESEAT -> UNLOCK -> REMOVE
  -> RETURN -> RELEASE -> DONE
  (~/remove: DETECT_INSTALLED -> REGRASP -> RESEAT -> UNLOCK -> ...; RETRY_GRASP and RECOVER
  are entered on failures)

Services (std_srvs/Trigger, relative to the node):
  ~/run         complete sequence: install, verify, remove, put back
  ~/install     DETECT .. VERIFY_INSTALLATION (the portafilter stays locked in the machine)
  ~/remove      portafilter locked in the machine -> unlock, remove, put down at the place pose
  ~/detect      perception only: report portafilter / mount poses and the bayonet state
  ~/plan_check  perception + grasp planning + IK of every key pose; NOTHING moves
  ~/go_ready    move to the ready pose
  ~/ready       readiness query (never moves)
  ~/cancel      stop at the next state transition (then RECOVER)
Topic: ~/status (visual_manipulation_interfaces/TaskStatus, latched)
"""
import math
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional

from controller_manager_msgs.srv import ListControllers
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from visual_manipulation_core.bayonet import BayonetPlanner, BayonetSpec
from visual_manipulation_core.grasp import GraspSpec, Motion, generate_grasps
from visual_manipulation_core.ros_conversions import tf_from_transform, transform_from_pose
from visual_manipulation_core.se3 import Transform, pose_error
from visual_manipulation_core.servo import pbvs_step
from visual_manipulation_core.state_machine import DONE, FAILED, StepFailed, TaskStateMachine
from visual_manipulation_interfaces.msg import TaskStatus, TrackedObjectArray
from visual_manipulation_motion.arm import Arm
from visual_manipulation_motion.gripper_client import GripperClient
from visual_manipulation_motion.moveit_client import MoveItClient
from visual_manipulation_motion import scene as sc
from visual_manipulation_motion.ros_utils import call_service

PF, MOUNT, DOCK, TABLE = 'portafilter', 'machine_mount', 'portafilter_dock', 'workcell_table'


@dataclass
class Ctx:
    """Run-time knowledge of one sequence."""
    mode: str = 'run'                         # run | install | remove
    T_B_P0: Optional[Transform] = None        # portafilter pose where it was picked (= dock)
    T_B_M: Optional[Transform] = None         # measured mount frame
    T_B_P: Optional[Transform] = None         # latest portafilter measurement
    grasp: object = None                      # chosen GraspCandidate
    grasp_spec: Optional[GraspSpec] = None
    T_P_tcp: Optional[Transform] = None       # in-hand transform, ACTUAL tcp (planning scene)
    T_P_cmd: Optional[Transform] = None       # in-hand transform w.r.t. the COMMANDED tcp: absorbs
                                              # arm sag / calibration offsets, used for commands
    location: str = 'unknown'                 # dock | hand | machine_inserted | machine_locked | placed
    holding: bool = False
    notes: list = field(default_factory=list)


class PortafilterTask(Node):

    def __init__(self):
        super().__init__('portafilter_task')
        P = self.declare_parameter
        self.frame = P('planning_frame', 'world').value
        # ---------------- robot
        joints = [j for j in P('arm.joints', ['']).value if j]
        self.hand_links = [j for j in P('arm.hand_links', ['']).value if j]
        moveit = MoveItClient(self, P('arm.group', 'manipulator').value, P('arm.tip_link', 'tool0').value, joints,
                              self.frame, P('arm.moveit_namespace', '').value)
        moveit.velocity_scaling = moveit.acceleration_scaling = P('motion.velocity_scaling', 0.2).value
        self.insert_scaling = P('motion.insertion_velocity_scaling', 0.1).value
        self.default_scaling = moveit.velocity_scaling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm = Arm(self, moveit, Transform.from_flat(P('arm.flange_to_tcp', [0.0] * 6).value), self.tf_buffer,
                       P('motion.tracking_tolerance', 0.03).value, P('motion.settle_timeout', 3.0).value)
        self.ik_attempts = P('motion.ik_attempts', 12).value
        self.cart_step = P('motion.cartesian_max_step', 0.004).value
        flat = [float(v) for v in P('motion.ready_tcp_poses', [0.0] * 6).value]
        self.ready_poses = [Transform.from_flat(flat[i:i + 6]) for i in range(0, len(flat) - 5, 6)]
        self.gripper = GripperClient(
            self, P('gripper.action', 'gripper_controller/gripper_cmd').value, P('gripper.joint', 'finger_joint').value,
            P('gripper.open_position', 0.04).value, P('gripper.close_position', 0.0).value,
            P('gripper.grasp_min_position', 0.005).value, P('gripper.grasp_max_position', 0.03).value,
            P('gripper.max_effort', 0.0).value)
        self.release_q = P('gripper.release_position', self.gripper.open_position).value
        self.squeeze_settle = P('gripper.settle_time', 2.5).value
        self.contact_tol = P('gripper.contact_tolerance', 0.05).value
        self.required_controllers = [c for c in P('required_controllers', ['']).value if c]

        # ---------------- perception
        self.objects_topic = P('perception.objects_topic', '/fiducial_object_tracker/objects').value
        self.reset_srv_name = P('perception.reset_service', '/fiducial_object_tracker/reset').value
        self.det_timeout = P('perception.timeout', 8.0).value
        self.inhand_tol = P('perception.in_hand_tolerance', 0.02).value

        # ---------------- objects (coffee_geometry.yaml)
        self.pf_shapes = sc.shapes_from_parameters(self, f'objects.{PF}')
        self.mount_shapes = sc.shapes_from_parameters(self, f'objects.{MOUNT}')
        self.dock_shapes = sc.shapes_from_parameters(self, f'objects.{DOCK}')
        self.table_center = list(P('workcell.table_center', [0.0, 0.0, -1.0]).value)
        self.table_size = list(P('workcell.table_size', [0.0, 0.0, 0.0]).value)
        self.bayonet = BayonetPlanner(BayonetSpec(
            Transform.from_flat(P('bayonet.locked_offset', [0.0] * 6).value),
            math.radians(P('bayonet.lock_angle_deg', 45.0).value),
            P('bayonet.insertion_depth', 0.04).value,
            list(P('install.staging_offset', [0.10, 0.0, 0.0]).value),
            math.radians(P('install.rotation_step_deg', 5.0).value)))

        # ---------------- grasp
        names = [n for n in P('grasp.candidates', ['default']).value if n]
        approach = Motion(list(P('grasp.approach.direction', [0.0, 0.0, -1.0]).value),
                          P('grasp.approach.distance', 0.08).value, P('grasp.approach.frame', 'base').value)
        retreat = Motion(list(P('grasp.retreat.direction', [0.0, 0.0, 1.0]).value),
                         P('grasp.retreat.distance', 0.08).value, P('grasp.retreat.frame', 'base').value)
        self.grasp_specs = [GraspSpec(n, Transform.from_flat(P(f'grasp.{n}.object_to_tcp', [0.0] * 6).value),
                                      approach, retreat) for n in names]

        # ---------------- servo / verification / release
        self.servo_enabled = P('servo.enabled', True).value
        self.servo_gain = P('servo.gain', 0.9).value
        self.servo_iters = P('servo.max_iterations', 6).value
        self.servo_pos_tol = P('servo.position_tolerance', 0.0015).value
        self.servo_ang_tol = math.radians(P('servo.angle_tolerance_deg', 1.5).value)
        self.servo_axial_tol = P('servo.axial_tolerance', 0.003).value
        self.servo_tilt_tol = math.radians(P('servo.tilt_tolerance_deg', 1.5).value)
        self.servo_accept_factor = P('servo.accept_factor', 1.6).value
        self.coarse_factor = P('servo.coarse_factor', 2.5).value
        self.insert_axial_tol = P('servo.insert_axial_tolerance', 0.001).value
        self.holding_scaling = P('motion.holding_velocity_scaling', 0.1).value
        self.servo_tilt = P('servo.correct_tilt', False).value
        self.servo_max_step = P('servo.max_translation', 0.02).value
        self.v_radial = P('verify.radial_tolerance', 0.004).value
        self.v_axial = P('verify.axial_tolerance', 0.006).value
        self.v_angle = math.radians(P('verify.angle_tolerance_deg', 6.0).value)
        self.release_test = P('verify.release_test', True).value
        self.release_test_q = P('verify.release_test_position', 0.30).value
        self.release_test_wait = P('verify.release_test_wait', 1.5).value
        self.place_tol = P('verify.place_tolerance', 0.008).value
        self.place_clearance = P('release.clearance', 0.002).value
        self.place_near = P('release.fine_align_height', 0.03).value
        place = list(P('release.place_pose', [0.0] * 6).value)
        self.fixed_place_pose = Transform.from_flat(place) if any(abs(v) > 0 for v in place) else None
        self.auto_start = P('auto_start', False).value

        # ---------------- ROS plumbing
        cb = ReentrantCallbackGroup()
        self._objects = {}
        self._obj_lock = threading.Lock()
        self.create_subscription(TrackedObjectArray, self.objects_topic, self._on_objects, 10, callback_group=cb)
        self.create_subscription(String, '/robot_description', self._on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=cb)
        self.reset_cli = self.create_client(Trigger, self.reset_srv_name, callback_group=cb)
        self.list_ctrl = self.create_client(ListControllers, '/controller_manager/list_controllers', callback_group=cb)
        self.status_pub = self.create_publisher(TaskStatus, '~/status',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.static_tf = StaticTransformBroadcaster(self)
        self._debug_frames = {}
        self._busy = threading.Lock()
        self.sm = None
        self.ctx = Ctx()
        for name, fn in [('run', lambda: self.run_sequence('run')), ('install', lambda: self.run_sequence('install')),
                         ('remove', lambda: self.run_sequence('remove')), ('detect', self.srv_detect),
                         ('plan_check', self.srv_plan_check), ('go_ready', self.srv_go_ready)]:
            self.create_service(Trigger, f'~/{name}', lambda req, res, fn=fn, name=name: self._trigger(name, fn, res),
                                callback_group=cb)
        self.create_service(Trigger, '~/ready', self._on_ready, callback_group=cb)
        self.create_service(Trigger, '~/cancel', self._on_cancel, callback_group=cb)
        self.publish_status('IDLE', '', 'waiting for MoveIt, controllers, gripper and perception')
        self.get_logger().info(f'portafilter task ready to configure: group {moveit.group}, tip {moveit.tip_link}, '
                               f'{len(self.grasp_specs)} grasp candidates, frame {self.frame}')
        if self.auto_start:
            threading.Thread(target=self._auto, daemon=True).start()

    # ================================================================== inputs
    def _on_objects(self, msg: TrackedObjectArray):
        now = time.monotonic()
        with self._obj_lock:
            for o in msg.objects:
                self._objects[o.name] = (now, o)

    def _on_description(self, msg):
        limits = {}
        for j in ET.fromstring(msg.data).findall('joint'):
            lim = j.find('limit')
            if j.get('name') in self.arm.moveit.joint_names and lim is not None:
                limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        self.arm.moveit.limits = limits

    def tracked(self, name):
        with self._obj_lock:
            return self._objects.get(name)

    def measure(self, name, timeout=None, fresh=True, required=True) -> Optional[Transform]:
        """Stable pose of a tracked object. fresh=True resets the tracker filter first, so only
        samples taken after this call (i.e. after the last robot motion) are used."""
        timeout = self.det_timeout if timeout is None else timeout
        t_req = time.monotonic()
        if fresh and self.reset_cli.service_is_ready():
            try:
                call_service(self.reset_cli, Trigger.Request(), 3.0, 'tracker reset')
                t_req = time.monotonic()
            except Exception:  # noqa: BLE001 - a missing reset only makes the wait longer
                pass
        while time.monotonic() - t_req < timeout:
            e = self.tracked(name)
            if e is not None and e[0] > t_req + 0.05 and e[1].stable:
                return transform_from_pose(e[1].pose.pose)
            time.sleep(0.05)
        if required:
            raise StepFailed(f'no stable camera estimate of "{name}" within {timeout:.0f} s '
                             '(marker hidden or moving?)')
        return None

    # ================================================================== readiness
    def missing(self):
        m = []
        if not self.arm.moveit.ready(0.5):
            m.append('move_group interfaces')
        if not self.gripper.ready(0.5):
            m.append('gripper action / finger joint state')
        if len(self.arm.moveit.limits) < len(self.arm.moveit.joint_names):
            m.append('joint limits from /robot_description')
        if self.required_controllers and not self.controllers_active():
            m.append('active controllers')
        if self.tracked(MOUNT) is None and self.tracked(PF) is None:
            m.append('perception (no tracked object yet)')
        return m

    def controllers_active(self):
        try:
            res = call_service(self.list_ctrl, ListControllers.Request(), 3.0, 'list_controllers')
        except Exception:  # noqa: BLE001
            return False
        active = {c.name for c in res.controller if c.state == 'active'}
        return all(c in active for c in self.required_controllers)

    def _on_ready(self, _req, res):
        miss = self.missing()
        res.success = not miss and not self._busy.locked()
        res.message = 'ready' if res.success else ('busy' if not miss else 'waiting for: ' + ', '.join(miss))
        return res

    def _on_cancel(self, _req, res):
        if self.sm is not None and self._busy.locked():
            self.sm.cancel()
            res.success, res.message = True, 'cancel requested'
        else:
            res.success, res.message = False, 'nothing running'
        return res

    def wait_ready(self, timeout=300.0):
        t0, last = time.monotonic(), ''
        while time.monotonic() - t0 < timeout and rclpy.ok():
            miss = self.missing()
            if not miss:
                return True
            if ', '.join(miss) != last:
                last = ', '.join(miss)
                self.get_logger().info(f'waiting for: {last}')
            time.sleep(1.0)
        return False

    def _trigger(self, name, fn, res):
        if not self._busy.acquire(blocking=False):
            res.success, res.message = False, 'busy: another sequence is running'
            return res
        try:
            if not self.wait_ready(10.0):
                res.success, res.message = False, 'not ready: ' + ', '.join(self.missing())
                return res
            ok, msg = fn()
            res.success, res.message = bool(ok), msg
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'{name}: {exc}\n{traceback.format_exc()}')
            res.success, res.message = False, f'{type(exc).__name__}: {exc}'
        finally:
            self._busy.release()
        return res

    def _auto(self):
        if not self.wait_ready():
            self.get_logger().error('auto_start: system never became ready')
            return
        with self._busy:
            ok, msg = self.run_sequence('run')
            self.get_logger().info(f'auto_start finished: success={ok}: {msg}')

    # ================================================================== status / debug
    def publish_status(self, state, previous, detail, attempt=0, failed=False):
        m = TaskStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.task, m.state, m.previous_state, m.detail = 'portafilter', state, previous, detail or ''
        m.active = state not in ('IDLE', DONE, FAILED)
        m.failed, m.attempt = failed or state == FAILED, int(attempt)
        self.status_pub.publish(m)

    def _listener(self, state, previous, detail, attempt):
        log = self.get_logger()
        txt = f'[{state}] {detail}' if detail else f'[{state}]'
        line = f'==== {txt}' + (f' (attempt {attempt})' if attempt > 1 else '')
        # (rclpy: one call site must always use the same severity)
        if state == FAILED or 'failed' in (detail or ''):
            log.error(line)
        else:
            log.info(line)
        self.publish_status(state, previous, detail, attempt)

    def show(self, name, T: Transform):
        """Debug TF frame (latched) for RViz: <planning frame> -> task/<name>."""
        self._debug_frames[name] = tf_from_transform(T, self.frame, f'task/{name}', self.get_clock().now().to_msg())
        self.static_tf.sendTransform(list(self._debug_frames.values()))

    # ================================================================== planning scene
    def scene_world(self, ctx: Ctx, include_pf=True, include_dock=True):
        objs = []
        if self.table_size[0] > 0:
            objs.append(sc.box(TABLE, self.frame, self.table_center, self.table_size))
        if ctx.T_B_M is not None:
            objs.append(sc.collision_object(MOUNT, self.frame, ctx.T_B_M, self.mount_shapes))
        if ctx.T_B_P0 is not None and self.dock_shapes:
            objs.append(sc.collision_object(DOCK, self.frame, ctx.T_B_P0, self.dock_shapes) if include_dock
                        else sc.remove(DOCK, self.frame))
        if include_pf and ctx.T_B_P is not None and not ctx.holding:
            objs.append(sc.collision_object(PF, self.frame, ctx.T_B_P, self.pf_shapes))
        self.arm.moveit.apply_scene(sc.scene_diff(objs))

    def set_dock(self, ctx, present: bool):
        if ctx.T_B_P0 is None or not self.dock_shapes:
            return
        o = sc.collision_object(DOCK, self.frame, ctx.T_B_P0, self.dock_shapes) if present else sc.remove(DOCK, self.frame)
        self.arm.moveit.apply_scene(sc.scene_diff([o]))

    def attach_pf(self, ctx, T_B_P):
        obj = sc.collision_object(PF, self.frame, T_B_P, self.pf_shapes)
        self.arm.moveit.apply_scene(sc.scene_diff([sc.remove(PF, self.frame)],
                                                  [sc.attach(obj, self.arm.moveit.tip_link, self.hand_links)]))

    def detach_pf(self, ctx, T_B_P=None):
        objs = [sc.collision_object(PF, self.frame, T_B_P, self.pf_shapes)] if T_B_P is not None else []
        self.arm.moveit.apply_scene(sc.scene_diff(objs, [sc.detach(PF, self.arm.moveit.tip_link, self.frame)]))

    def allow_mount_contact(self, allowed: bool):
        """Moves INSIDE the group head are deliberately contact-close (mm clearances, measured
        poses with mm errors): only for those, contact between the held portafilter and the
        mount object is allowed in planning. Everything else stays collision-checked, and real
        contact is detected by the joint-tracking verification of every motion."""
        try:
            self.arm.moveit.set_allowed_collision(PF, MOUNT, allowed)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'could not update the allowed collision matrix: {exc}')

    def remove_pf_from_world(self):
        self.arm.moveit.apply_scene(sc.scene_diff([sc.remove(PF, self.frame)],
                                                  [sc.detach(PF, self.arm.moveit.tip_link, self.frame)]))

    # ================================================================== motion helpers
    def go_ready(self):
        last = None
        for T in self.ready_poses:
            try:
                self.arm.move_tcp(T, f'ready {np.round(T.t, 3).tolist()}', self.ik_attempts)
                return
            except StepFailed as exc:
                last = exc
        raise StepFailed(f'ready pose unreachable: {last}')

    def near_ready(self, tol=0.03):
        try:
            tcp = self.arm.tcp_pose()
        except Exception:  # noqa: BLE001
            return False
        return any(pose_error(tcp, T)[0] < tol for T in self.ready_poses)

    def slow(self, mode):
        """True: insertion speed, 'holding': payload transport speed, False: default."""
        s = self.insert_scaling if mode is True else self.holding_scaling if mode == 'holding' else self.default_scaling
        self.arm.moveit.velocity_scaling = self.arm.moveit.acceleration_scaling = s

    def tcp_for(self, ctx, T_B_P: Transform) -> Transform:
        """TCP command that brings the held object to T_B_P."""
        return T_B_P @ (ctx.T_P_cmd if ctx.T_P_cmd is not None else ctx.T_P_tcp)

    def check_holding(self, label):
        q = self.gripper.position
        if not self.gripper.holding(q):
            raise StepFailed(f'{label}: portafilter lost (finger {q:.3f} rad outside '
                             f'[{self.gripper.grasp_min}, {self.gripper.grasp_max}])', recoverable=False)

    def update_in_hand(self, ctx, label, required=False):
        """Measure the held portafilter and update the in-hand transform T_P_tcp. The camera
        must see it MOVE WITH the hand (compares with the prediction of the old transform)."""
        T = self.measure(PF, required=required)
        if T is None:
            self.get_logger().warn(f'{label}: portafilter marker not visible, keeping the in-hand transform')
            return None
        tcp = self.arm.tcp_pose()
        pred = tcp @ ctx.T_P_tcp.inverse()
        d, a = pose_error(pred, T)
        if d > self.inhand_tol:
            raise StepFailed(f'{label}: camera sees the portafilter {d * 1000:.0f} mm from where the hand '
                             'holds it (not lifted / slipped?)', recoverable=False)
        ctx.T_P_tcp = T.inverse() @ tcp
        ctx.T_P_cmd = T.inverse() @ self.arm.command_pose()
        ctx.T_B_P = T
        self.attach_pf(ctx, T)                     # planning scene follows the measured pose
        self.get_logger().info(f'{label}: in-hand transform updated (slip {d * 1000:.1f} mm, {math.degrees(a):.1f} deg)')
        return T

    # ================================================================== state handlers
    def prepare(self, ctx: Ctx, label: str):
        """Start-of-sequence hygiene: never start with something in the hand, forget stale
        planning-scene attachments of an earlier (failed) run, clear the camera's view."""
        q = self.gripper.position
        if self.gripper.holding(q):
            raise StepFailed(f'{label}: the gripper is holding something (finger {q:.3f} rad) - '
                             'remove it / recover manually first', recoverable=False)
        self.remove_pf_from_world()
        self.allow_mount_contact(False)
        if not self.near_ready():
            self.go_ready()

    def s_detect(self, ctx: Ctx):
        self.prepare(ctx, 'DETECT')
        ctx.T_B_M = self.measure(MOUNT)
        ctx.T_B_P = self.measure(PF, fresh=False)
        tilt_m = math.degrees(math.acos(min(1.0, ctx.T_B_M.R[2, 2])))
        tilt_p = math.degrees(math.acos(min(1.0, ctx.T_B_P.R[2, 2])))
        self.get_logger().info(f'mount  at {np.round(ctx.T_B_M.t, 4).tolist()} rpy '
                               f'{np.round(np.degrees(ctx.T_B_M.rpy()), 1).tolist()} deg (axis tilt {tilt_m:.1f} deg)')
        self.get_logger().info(f'portafilter at {np.round(ctx.T_B_P.t, 4).tolist()} rpy '
                               f'{np.round(np.degrees(ctx.T_B_P.rpy()), 1).tolist()} deg (tilt {tilt_p:.1f} deg)')
        if tilt_m > 10.0:
            raise StepFailed(f'mount axis tilted {tilt_m:.1f} deg from vertical: implausible', recoverable=True)
        return 'ESTIMATE_POSE'

    def s_estimate(self, ctx: Ctx):
        state = self.bayonet.classify(ctx.T_B_M, ctx.T_B_P, self.v_radial, self.v_axial, self.v_angle)
        if state != 'out':
            raise StepFailed(f'the portafilter is in / at the machine (bayonet state "{state}"); '
                             + ('use ~/remove' if state == 'locked' else 'it must be freed manually'),
                             recoverable=False)
        ctx.T_B_P0 = ctx.T_B_P
        ctx.location = 'dock'
        self.allow_mount_contact(False)
        for n, T in [('mount', ctx.T_B_M), ('portafilter', ctx.T_B_P), ('locked_goal', self.bayonet.locked(ctx.T_B_M)),
                     ('pre_insert', self.bayonet.pre_insert(ctx.T_B_M)), ('staging', self.bayonet.staging(ctx.T_B_M))]:
            self.show(n, T)
        self.scene_world(ctx)
        return 'PLAN_GRASP'

    def choose_grasp(self, ctx, T_B_P, label):
        best = None
        for cand, spec in zip(generate_grasps(T_B_P, self.grasp_specs), self.grasp_specs):
            pre = self.arm.ik_tcp(cand.pregrasp, self.ik_attempts)
            grasp = self.arm.ik_tcp(cand.grasp, self.ik_attempts, avoid_collisions=False) if pre else None
            if pre is None or grasp is None:
                self.get_logger().info(f'{label}: candidate {cand.name}: no IK')
                continue
            m = min(self.arm.moveit.margin(pre), self.arm.moveit.margin(grasp))
            self.get_logger().info(f'{label}: candidate {cand.name}: joint-limit margin {m:.2f}')
            if best is None or m > best[0]:
                best = (m, cand, spec)
        if best is None:
            raise StepFailed(f'{label}: no reachable grasp candidate', recoverable=False)
        ctx.grasp, ctx.grasp_spec = best[1], best[2]
        ctx.T_P_tcp = best[2].object_to_tcp
        ctx.T_P_cmd = None
        self.show('grasp_tcp', ctx.grasp.grasp)
        self.show('pregrasp_tcp', ctx.grasp.pregrasp)
        self.get_logger().info(f'{label}: using grasp "{ctx.grasp.name}" (margin {best[0]:.2f})')

    def s_plan_grasp(self, ctx: Ctx):
        self.choose_grasp(ctx, ctx.T_B_P, 'PLAN_GRASP')
        return 'APPROACH'

    def approach(self, ctx, label):
        if not self.gripper.open():
            raise StepFailed(f'{label}: gripper did not open (finger {self.gripper.position})')
        self.arm.move_tcp(ctx.grasp.pregrasp, f'{label}: pre-grasp', self.ik_attempts)
        # the fingers must be allowed to surround the handle: portafilter out of the collision world
        self.remove_pf_from_world()
        q0 = self.gripper.position
        self.arm.cartesian([ctx.grasp.grasp], f'{label}: approach the handle', max_step=self.cart_step)
        if abs(self.gripper.position - q0) > self.contact_tol:
            raise StepFailed(f'{label}: unexpected finger contact during the approach')

    def s_approach(self, ctx: Ctx):
        if not self.near_ready():
            self.go_ready()
        self.approach(ctx, 'APPROACH')
        return 'GRASP'

    def s_grasp(self, ctx: Ctx):
        q = self.gripper.squeeze(settle=self.squeeze_settle)
        self.get_logger().info(f'GRASP: fingers settled at {q:.3f} rad')
        return 'VERIFY_GRASP'

    def retreat_open(self, ctx, label):
        """Open the gripper and back off along the approach direction (failed grasp)."""
        self.gripper.open()
        try:
            self.arm.cartesian([ctx.grasp.pregrasp], f'{label}: back off', max_step=self.cart_step)
        except StepFailed as exc:
            self.get_logger().warn(f'{label}: back off failed: {exc}')

    def s_verify_grasp(self, ctx: Ctx):
        q = self.gripper.position
        if not self.gripper.holding(q):
            self.retreat_open(ctx, 'VERIFY_GRASP')
            raise StepFailed(f'grasp NOT verified: finger at {q:.3f} rad, expected '
                             f'[{self.gripper.grasp_min}, {self.gripper.grasp_max}]', recoverable=True)
        ctx.holding = True
        ctx.location = 'hand'
        tcp = self.arm.tcp_pose()
        T_B_P = ctx.T_B_P
        ctx.T_P_tcp = T_B_P.inverse() @ tcp                    # grasp as executed
        ctx.T_P_cmd = T_B_P.inverse() @ self.arm.command_pose()
        self.attach_pf(ctx, T_B_P)
        self.set_dock(ctx, False)                              # lift straight out of the holder ring
        self.slow(True)
        try:
            # from here on the part is (being) lifted: a failure must go to RECOVER (which handles
            # a held part), never to the empty-hand grasp retry
            self.arm.cartesian([ctx.grasp.lift], 'VERIFY_GRASP: lift out of the dock', max_step=self.cart_step)
            time.sleep(0.5)
            self.check_holding('VERIFY_GRASP')
            self.update_in_hand(ctx, 'VERIFY_GRASP', required=True)
        except StepFailed as exc:
            raise StepFailed(str(exc), recoverable=False)
        finally:
            self.slow(False)
            self.set_dock(ctx, True)
        self.get_logger().info(f'VERIFY_GRASP: GRASP VERIFIED (finger {self.gripper.position:.3f} rad, '
                               'portafilter seen moving with the hand)')
        return 'TRANSPORT'

    def s_retry_grasp(self, ctx: Ctx):
        """Re-plan from a new measurement after a failed grasp (from the ready pose: at the
        pre-grasp pose the hand may hide the marker from the camera)."""
        self.go_ready()
        ctx.T_B_P = self.measure(PF)
        ctx.T_B_P0 = ctx.T_B_P
        self.scene_world(ctx)
        self.choose_grasp(ctx, ctx.T_B_P, 'RETRY_GRASP')
        return 'APPROACH'

    def s_transport(self, ctx: Ctx):
        self.check_holding('TRANSPORT')
        stage = self.bayonet.staging(ctx.T_B_M)
        self.slow('holding')
        try:
            self.arm.move_tcp(self.tcp_for(ctx, stage), 'TRANSPORT: to the staging pose in front of the group head',
                              self.ik_attempts)
        finally:
            self.slow(False)
        self.check_holding('TRANSPORT')
        return 'ALIGN'

    def servo_object(self, ctx, T_des: Transform, label: str, axis, tol, iterations, accept_factor):
        """Position-based visual servoing of the HELD portafilter to T_des (see core.servo).
        Errors are split w.r.t. `axis` (unit vector, base frame): radial (perpendicular),
        axial (along), angle (rotation about the axis) and tilt (between the z axes).
        Corrections are applied to the COMMANDED tcp (integral action against arm sag)."""
        e = None
        for it in range(1, iterations + 1):
            T = self.measure(PF)
            cmd = self.arm.command_pose()
            ctx.T_B_P = T
            ctx.T_P_tcp = T.inverse() @ self.arm.tcp_pose()
            ctx.T_P_cmd = T.inverse() @ cmd
            self.attach_pf(ctx, T)
            e = self.servo_error(T, T_des, axis)
            ok = self.within(e, tol, 1.0)
            self.get_logger().info(
                f'{label} servo {it}: radial {e["radial"] * 1000:.1f} mm, axial {e["axial"] * 1000:.1f} mm, '
                f'angle {math.degrees(e["angle"]):.2f} deg, tilt {math.degrees(e["tilt"]):.2f} deg' + (' -> OK' if ok else ''))
            if ok:
                return e
            r = pbvs_step(cmd, T, T_des, self.servo_gain, self.servo_max_step, math.radians(8.0), 0.0, 0.0,
                          self.servo_tilt, axis)
            self.arm.cartesian([r.target_tcp], f'{label}: servo step {it}', max_step=self.cart_step)
        if e is not None and self.within(e, tol, accept_factor):
            self.get_logger().warn(f'{label}: servo not fully converged; residual within the acceptance limits')
            return e
        raise StepFailed(f'{label}: visual servoing did not converge (radial {e["radial"] * 1000:.1f} mm, '
                         f'axial {e["axial"] * 1000:.1f} mm, angle {math.degrees(e["angle"]):.1f} deg, '
                         f'tilt {math.degrees(e["tilt"]):.1f} deg)')

    @staticmethod
    def servo_error(T: Transform, T_des: Transform, axis):
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        d = T.t - T_des.t
        axial = float(d @ a)
        xh = T.R[:, 0] - (T.R[:, 0] @ a) * a
        xd = T_des.R[:, 0] - (T_des.R[:, 0] @ a) * a
        ang = math.atan2(float(np.cross(xd, xh) @ a), float(xd @ xh))
        tilt = math.acos(max(-1.0, min(1.0, float(T.R[:, 2] @ T_des.R[:, 2]))))
        return {'radial': float(np.linalg.norm(d - axial * a)), 'axial': axial, 'angle': ang, 'tilt': tilt}

    def within(self, e, tol, factor):
        r, ax, ang, tilt = tol
        return (e['radial'] <= r * factor and abs(e['axial']) <= ax * factor and abs(e['angle']) <= ang * factor
                and (not self.servo_tilt or e['tilt'] <= tilt * factor))

    def s_align(self, ctx: Ctx):
        self.check_holding('ALIGN')
        axis = ctx.T_B_M.R[:, 2]
        fine = (self.servo_pos_tol, self.servo_axial_tol, self.servo_ang_tol, self.servo_tilt_tol)
        coarse = tuple(v * self.coarse_factor for v in fine)
        self.slow(True)
        try:
            if self.servo_enabled:      # 1. free space in front of the machine: remove grasp slip
                self.servo_object(ctx, self.bayonet.staging(ctx.T_B_M), 'ALIGN (staging)', axis, coarse,
                                  self.servo_iters, self.servo_accept_factor)
            else:
                self.update_in_hand(ctx, 'ALIGN (staging)')
            self.arm.cartesian([self.tcp_for(ctx, self.bayonet.pre_insert(ctx.T_B_M))],
                               'ALIGN: under the group head (pre-insert)', max_step=self.cart_step)
            if self.servo_enabled:      # 2. fine alignment with the bayonet axis
                self.servo_object(ctx, self.bayonet.pre_insert(ctx.T_B_M), 'ALIGN (pre-insert)', axis, fine,
                                  self.servo_iters, self.servo_accept_factor)
            return 'INSERT'
        finally:
            self.slow(False)

    def back_to_staging(self, ctx, label):
        """From anywhere under / in the group head back to the staging pose (straight moves)."""
        self.slow(True)
        try:
            self.arm.cartesian([self.tcp_for(ctx, self.bayonet.pre_insert(ctx.T_B_M)),
                                self.tcp_for(ctx, self.bayonet.staging(ctx.T_B_M))], label, max_step=self.cart_step,
                               min_fraction=0.9)
        finally:
            self.slow(False)

    def s_insert(self, ctx: Ctx):
        self.check_holding('INSERT')
        self.allow_mount_contact(True)
        self.slow(True)
        try:
            ctx.location = 'machine_inserted'
            self.arm.cartesian([self.tcp_for(ctx, T) for T in self.bayonet.insertion_path(ctx.T_B_M)],
                               'INSERT: lift the portafilter into the group head', max_step=self.cart_step)
            if self.servo_enabled:
                # the lugs must sit between lip and shower screen (mm gaps) before turning:
                # measure and correct the insertion depth (radial / tilt are constrained by contact)
                self.servo_object(ctx, self.bayonet.inserted(ctx.T_B_M), 'INSERT (depth check)', ctx.T_B_M.R[:, 2],
                                  (0.006, self.insert_axial_tol, math.radians(4.0), math.radians(6.0)),
                                  4, 1.0)
        except StepFailed as exc:
            # blocked (lug on the lip?): go back down to the pre-insert pose and re-align
            try:
                self.arm.cartesian([self.tcp_for(ctx, self.bayonet.pre_insert(ctx.T_B_M))],
                                   'INSERT: back down after a failed insertion', max_step=self.cart_step)
                ctx.location = 'hand'
            except StepFailed:
                pass
            raise StepFailed(f'insertion failed: {exc}')
        finally:
            self.slow(False)
        self.check_holding('INSERT')
        return 'ROTATE_TO_LOCK'

    def s_rotate(self, ctx: Ctx):
        self.slow(True)
        try:
            self.arm.cartesian([self.tcp_for(ctx, T) for T in self.bayonet.rotation_path(ctx.T_B_M, lock=True)],
                               f'ROTATE_TO_LOCK: turn {math.degrees(self.bayonet.spec.lock_angle):.0f} deg about the '
                               'group-head axis', max_step=self.cart_step)
        finally:
            self.slow(False)
        ctx.location = 'machine_locked'
        return 'VERIFY_INSTALLATION'

    def classify_now(self, ctx, label):
        T = self.measure(PF)
        ctx.T_B_P = T
        e = self.bayonet.error(ctx.T_B_M, T, 0.0, 0.0)
        st = self.bayonet.classify(ctx.T_B_M, T, self.v_radial, self.v_axial, self.v_angle)
        self.get_logger().info(f'{label}: bayonet state "{st}" - vs locked pose: radial {e["radial"] * 1000:.1f} mm, '
                               f'axial {e["axial"] * 1000:.1f} mm, angle {math.degrees(e["angle"]):.1f} deg, '
                               f'tilt {math.degrees(e["tilt"]):.1f} deg')
        return st, e

    def s_verify_installation(self, ctx: Ctx):
        st, e = self.classify_now(ctx, 'VERIFY_INSTALLATION')
        if st != 'locked':
            raise StepFailed(f'installation NOT verified: portafilter state "{st}"', recoverable=False)
        if ctx.mode == 'install':
            # leave it in the machine: release, back off, check from a clear viewpoint
            self.gripper.open(self.release_q)
            ctx.holding = False
            self.detach_pf(ctx, ctx.T_B_P)
            back = self.arm.tcp_pose()
            back = Transform(back.R, back.t - ctx.grasp_spec.approach.vector_in_base(back, ctx.T_B_P))
            self.arm.cartesian([back], 'VERIFY_INSTALLATION: release and back off', max_step=self.cart_step)
            self.go_ready()                     # the hand must not hide the marker from the camera
            st2, e2 = self.classify_now(ctx, 'VERIFY_INSTALLATION (hand off)')
            if st2 != 'locked':
                raise StepFailed(f'portafilter did not stay locked without the gripper (state "{st2}")',
                                 recoverable=False)
            ctx.location = 'machine_locked'
            ctx.notes.append(f'locked and released: radial {e2["radial"] * 1000:.1f} mm, axial '
                             f'{e2["axial"] * 1000:.1f} mm, angle {math.degrees(e2["angle"]):.1f} deg, '
                             f'tilt {math.degrees(e2["tilt"]):.1f} deg')
            self.scene_world(ctx)
            return DONE
        if not self.release_test:
            ctx.notes.append('locked (verified by camera while held)')
            return 'UNLOCK'
        # release test IN PLACE: the fingers open around the handle without the hand moving, so
        # only the bayonet holds the portafilter (an unlocked one would drop out). The hand stays
        # at the handle: no approach onto a freely pivoting part is needed afterwards.
        self.gripper.open(self.release_test_q)
        ctx.holding = False
        time.sleep(self.release_test_wait)
        st2, e2 = self.classify_now(ctx, 'VERIFY_INSTALLATION (fingers open)')
        if st2 != 'locked':
            raise StepFailed(f'portafilter did not stay locked with the fingers open (state "{st2}")',
                             recoverable=False)
        ctx.notes.append(f'locked: radial {e2["radial"] * 1000:.1f} mm, axial {e2["axial"] * 1000:.1f} mm, '
                         f'angle {math.degrees(e2["angle"]):.1f} deg, tilt {math.degrees(e2["tilt"]):.1f} deg, '
                         'held by the bayonet alone')
        self.get_logger().info('VERIFY_INSTALLATION: INSTALLATION VERIFIED - the portafilter stays locked with '
                               'the fingers open')
        q = self.gripper.squeeze(settle=self.squeeze_settle)
        if not self.gripper.holding(q):
            self.get_logger().warn(f'VERIFY_INSTALLATION: closing again did not capture the handle ({q:.3f} rad) '
                                   '-> full re-grasp')
            self.gripper.open()
            self.detach_pf(ctx, ctx.T_B_P)
            return 'REGRASP'
        ctx.holding = True
        ctx.T_P_tcp = ctx.T_B_P.inverse() @ self.arm.tcp_pose()
        ctx.T_P_cmd = ctx.T_B_P.inverse() @ self.arm.command_pose()
        self.attach_pf(ctx, ctx.T_B_P)
        self.get_logger().info(f'VERIFY_INSTALLATION: handle gripped again in place (finger {q:.3f} rad)')
        return 'RESEAT'

    def s_detect_installed(self, ctx: Ctx):
        self.prepare(ctx, 'DETECT_INSTALLED')
        ctx.T_B_M = self.measure(MOUNT)
        st, e = self.classify_now(ctx, 'DETECT_INSTALLED')
        if st != 'locked':
            raise StepFailed(f'no locked portafilter in the machine (state "{st}")', recoverable=False)
        ctx.location = 'machine_locked'
        ctx.holding = False
        self.scene_world(ctx)
        return 'REGRASP'

    def s_regrasp(self, ctx: Ctx):
        if ctx.holding:
            return 'UNLOCK'
        ctx.T_B_P = self.measure(PF)
        self.scene_world(ctx)
        self.choose_grasp(ctx, ctx.T_B_P, 'REGRASP')
        self.approach(ctx, 'REGRASP')
        q = self.gripper.squeeze(settle=self.squeeze_settle)
        if not self.gripper.holding(q):
            self.retreat_open(ctx, 'REGRASP')
            raise StepFailed(f're-grasp NOT verified (finger {q:.3f} rad)')
        ctx.holding = True
        ctx.T_P_tcp = ctx.T_B_P.inverse() @ self.arm.tcp_pose()
        ctx.T_P_cmd = ctx.T_B_P.inverse() @ self.arm.command_pose()
        self.attach_pf(ctx, ctx.T_B_P)
        self.get_logger().info(f'REGRASP: handle grasped again (finger {q:.3f} rad)')
        return 'RESEAT'

    def s_reseat(self, ctx: Ctx):
        """A released part hangs tilted / off-centre in the bayonet clearances. Bring it back to
        the exact locked pose (centred: the basket must not rub on the lip while turning)
        before the unlock arc, which starts from there."""
        self.check_holding('RESEAT')
        if not self.servo_enabled:
            return 'UNLOCK'
        self.allow_mount_contact(True)
        self.slow(True)
        try:
            self.servo_object(ctx, self.bayonet.locked(ctx.T_B_M), 'RESEAT', ctx.T_B_M.R[:, 2],
                              (self.servo_pos_tol, self.insert_axial_tol * 1.5, self.servo_ang_tol, self.servo_tilt_tol),
                              self.servo_iters, self.servo_accept_factor)
        finally:
            self.slow(False)
        return 'UNLOCK'

    def s_unlock(self, ctx: Ctx):
        self.check_holding('UNLOCK')
        self.allow_mount_contact(True)
        self.slow(True)
        try:
            self.arm.cartesian([self.tcp_for(ctx, T) for T in self.bayonet.rotation_path(ctx.T_B_M, lock=False)],
                               f'UNLOCK: turn back {math.degrees(self.bayonet.spec.lock_angle):.0f} deg',
                               max_step=self.cart_step)
        except StepFailed as exc:
            # blocked while turning: turn back to the locked pose, then RESEAT + retry
            try:
                self.arm.cartesian([self.tcp_for(ctx, self.bayonet.locked(ctx.T_B_M))], 'UNLOCK: back to locked',
                                   max_step=self.cart_step)
            except StepFailed:
                pass
            self.check_holding('UNLOCK')
            raise StepFailed(f'unlock blocked: {exc}')
        finally:
            self.slow(False)
        ctx.location = 'machine_inserted'
        return 'REMOVE'

    def s_remove(self, ctx: Ctx):
        self.check_holding('REMOVE')
        self.slow(True)
        try:
            self.arm.cartesian([self.tcp_for(ctx, T) for T in self.bayonet.extraction_path(ctx.T_B_M)],
                               'REMOVE: pull the portafilter down out of the group head', max_step=self.cart_step)
            ctx.location = 'hand'
            self.arm.cartesian([self.tcp_for(ctx, self.bayonet.staging(ctx.T_B_M))], 'REMOVE: out from under the head',
                               max_step=self.cart_step)
            self.allow_mount_contact(False)
        finally:
            self.slow(False)
        self.check_holding('REMOVE')
        self.update_in_hand(ctx, 'REMOVE')
        return 'RETURN'

    def place_pose(self, ctx) -> Transform:
        """Where the part is put down: the pose it was picked from (or release.place_pose),
        LEVELLED (only x, y, z and the heading are kept) so that small tilts of a previous
        placement do not accumulate."""
        T = ctx.T_B_P0 if ctx.T_B_P0 is not None else self.fixed_place_pose
        if T is None:
            raise StepFailed('no place pose (no pick pose recorded and release.place_pose not set)', recoverable=False)
        heading = math.atan2(T.R[1, 0], T.R[0, 0])
        return Transform.from_xyz_rpy(T.t, [0.0, 0.0, heading])

    def s_return(self, ctx: Ctx):
        self.check_holding('RETURN')
        place = self.place_pose(ctx)
        if ctx.T_B_P0 is None:
            ctx.T_B_P0 = place                               # dock geometry is defined at the place pose
            self.scene_world(ctx)
        spec = ctx.grasp_spec or self.grasp_specs[0]
        lift = spec.retreat.vector_in_base(place @ ctx.T_P_tcp, place)
        above = Transform(place.R, place.t + lift)
        self.show('place_goal', place)
        self.slow('holding')
        try:
            self.arm.move_tcp(self.tcp_for(ctx, above), 'RETURN: above the place pose', self.ik_attempts)
        finally:
            self.slow(False)
        self.check_holding('RETURN')
        self.slow(True)
        try:
            fine = (self.servo_pos_tol, self.servo_axial_tol, self.servo_ang_tol, self.servo_tilt_tol)
            if self.servo_enabled:
                self.servo_object(ctx, above, 'RETURN (above the dock)', [0.0, 0.0, 1.0],
                                  tuple(v * 2 for v in fine), self.servo_iters, self.servo_accept_factor)
            self.set_dock(ctx, False)
            near = Transform(place.R, place.t + [0.0, 0.0, self.place_near])
            if self.servo_enabled:      # fine alignment just above the holder (lowering drift)
                self.arm.cartesian([self.tcp_for(ctx, near)], 'RETURN: just above the holder', max_step=self.cart_step)
                self.servo_object(ctx, near, 'RETURN (above the holder)', [0.0, 0.0, 1.0], fine,
                                  self.servo_iters, self.servo_accept_factor)
            down = Transform(place.R, place.t + [0.0, 0.0, self.place_clearance])
            self.arm.cartesian([self.tcp_for(ctx, down)], 'RETURN: lower into the dock', max_step=self.cart_step)
        finally:
            self.slow(False)
        return 'RELEASE'

    def s_release(self, ctx: Ctx):
        if not self.gripper.open(self.release_q):
            raise StepFailed(f'gripper did not open (finger {self.gripper.position})')
        ctx.holding = False
        place = self.place_pose(ctx)
        self.detach_pf(ctx, place)
        tcp = self.arm.tcp_pose()
        spec = ctx.grasp_spec or self.grasp_specs[0]
        up = Transform(tcp.R, tcp.t - spec.approach.vector_in_base(tcp, place))
        self.arm.cartesian([up], 'RELEASE: retreat', max_step=self.cart_step)
        self.set_dock(ctx, True)
        self.go_ready()
        T = self.measure(PF)
        d, a = pose_error(T, place)
        ctx.location = 'placed'
        ctx.T_B_P = T
        if d > self.place_tol:
            raise StepFailed(f'portafilter is {d * 1000:.0f} mm from the place pose after release', recoverable=False)
        ctx.notes.append(f'placed {d * 1000:.1f} mm / {math.degrees(a):.1f} deg from the place pose')
        self.get_logger().info(f'RELEASE: PLACEMENT VERIFIED by camera ({d * 1000:.1f} mm, {math.degrees(a):.1f} deg)')
        return DONE

    def s_recover(self, ctx: Ctx):
        """Best effort: never drop a held portafilter, get it out of the machine if needed and put
        it down at the place pose; then park the arm. Always ends in FAILED."""
        log = self.get_logger()
        log.warn(f'RECOVER: after "{self.sm.error}" (portafilter location: {ctx.location}, holding {ctx.holding})')
        self.slow(False)
        steps = []
        if ctx.location.startswith('machine') or ctx.holding:
            self.allow_mount_contact(True)
        try:
            if ctx.holding and ctx.location.startswith('machine') and ctx.T_B_M is not None:
                st, _ = self.classify_now(ctx, 'RECOVER')
                if st in ('locked', 'partially_rotated'):
                    self.s_unlock(ctx)
                    steps.append('unlocked')
                self.s_remove(ctx)
                steps.append('removed from the machine')
            if ctx.holding and ctx.location == 'hand':
                if ctx.T_B_M is not None:
                    try:
                        tcp = self.arm.tcp_pose()
                        if pose_error(tcp, self.tcp_for(ctx, self.bayonet.pre_insert(ctx.T_B_M)))[0] < 0.08:
                            self.back_to_staging(ctx, 'RECOVER: away from the group head')
                    except StepFailed:
                        pass
                self.s_return(ctx)
                self.s_release(ctx)
                steps.append('put back at the place pose')
            elif not ctx.holding:
                self.gripper.open()
                try:
                    tcp = self.arm.tcp_pose()
                    self.arm.cartesian([Transform(tcp.R, tcp.t + [0, 0, 0.05])], 'RECOVER: retreat up', min_fraction=0.5)
                except StepFailed:
                    pass
                self.go_ready()
                steps.append('arm parked')
        except Exception as exc:  # noqa: BLE001
            log.error(f'RECOVER incomplete: {exc}')
            steps.append(f'recovery incomplete: {exc}')
        self.allow_mount_contact(False)
        ctx.notes.append('recovery: ' + (', '.join(steps) or 'nothing to do'))
        return FAILED

    # ================================================================== sequences
    def build(self):
        sm = TaskStateMachine('portafilter', listener=self._listener)
        add = sm.add_state
        add('DETECT', self.s_detect, 3, description='locate portafilter and machine mount with the camera')
        add('ESTIMATE_POSE', self.s_estimate, 1, description='poses -> grasp / bayonet goals, planning scene')
        add('PLAN_GRASP', self.s_plan_grasp, 1, description='grasp candidates from the portafilter pose')
        add('APPROACH', self.s_approach, 2, on_failure='RECOVER', description='pre-grasp, straight approach to the handle')
        add('GRASP', self.s_grasp, 1, on_failure='RECOVER', description='close the gripper')
        add('VERIFY_GRASP', self.s_verify_grasp, 2, retry_state='RETRY_GRASP', on_failure='RECOVER',
            description='finger angle, lift, camera sees it move with the hand')
        add('RETRY_GRASP', self.s_retry_grasp, 1, on_failure='RECOVER', description='re-measure and re-plan the grasp')
        add('TRANSPORT', self.s_transport, 2, on_failure='RECOVER', description='collision-free move to the staging pose')
        add('ALIGN', self.s_align, 3, on_failure='RECOVER', description='under the group head + visual servoing')
        add('INSERT', self.s_insert, 2, retry_state='ALIGN', on_failure='RECOVER',
            description='straight up into the group head (lugs through the notches)')
        add('ROTATE_TO_LOCK', self.s_rotate, 1, on_failure='RECOVER', description='turn about the group-head axis')
        add('VERIFY_INSTALLATION', self.s_verify_installation, 1, on_failure='RECOVER',
            description='camera: locked pose; release test')
        add('DETECT_INSTALLED', self.s_detect_installed, 2, description='find the locked portafilter')
        add('REGRASP', self.s_regrasp, 2, on_failure='RECOVER', description='grasp the handle at the machine')
        add('RESEAT', self.s_reseat, 2, on_failure='RECOVER', description='servo the held part to the exact locked pose')
        add('UNLOCK', self.s_unlock, 2, retry_state='RESEAT', on_failure='RECOVER',
            description='turn back to the unlocked angle')
        add('REMOVE', self.s_remove, 1, on_failure='RECOVER', description='pull down and out of the group head')
        add('RETURN', self.s_return, 1, on_failure='RECOVER', description='carry to the place pose and lower')
        add('RELEASE', self.s_release, 1, on_failure='RECOVER', description='open, retreat, camera check')
        add('RECOVER', self.s_recover, 1, description='safe state (never drops a held part)')
        return sm

    def run_sequence(self, mode):
        self.sm = self.build()
        self.ctx = ctx = Ctx(mode=mode)
        self.slow(False)
        t0 = time.monotonic()
        res = self.sm.run(ctx, 'DETECT_INSTALLED' if mode == 'remove' else 'DETECT')
        dt = time.monotonic() - t0
        path = ' -> '.join(dict.fromkeys(t.state for t in res.history))
        msg = (f'{mode}: {"SUCCESS" if res.success else "FAILED"} in {dt:.0f} s. '
               f'{res.message if not res.success else ""} {" | ".join(ctx.notes)} | states: {path}')
        if res.success:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        self.publish_status(res.final_state, '', msg, failed=not res.success)
        return res.success, msg

    # ================================================================== tool services
    def srv_detect(self):
        T_M = self.measure(MOUNT, required=False)
        T_P = self.measure(PF, fresh=False, required=False)
        parts = []
        if T_M is not None:
            parts.append(f'mount at {np.round(T_M.t, 4).tolist()} yaw {math.degrees(T_M.rpy()[2]):.1f} deg')
        if T_P is not None:
            parts.append(f'portafilter at {np.round(T_P.t, 4).tolist()} yaw {math.degrees(T_P.rpy()[2]):.1f} deg')
        if T_M is not None and T_P is not None:
            e = self.bayonet.error(T_M, T_P)
            parts.append(f'bayonet state: {self.bayonet.classify(T_M, T_P, self.v_radial, self.v_axial, self.v_angle)} '
                         f'(vs locked: radial {e["radial"] * 1000:.1f} mm, axial {e["axial"] * 1000:.1f} mm, '
                         f'angle {math.degrees(e["angle"]):.1f} deg)')
        return T_M is not None and T_P is not None, '; '.join(parts) or 'nothing detected'

    def srv_plan_check(self):
        """IK feasibility of every key pose of the full sequence; nothing moves."""
        ctx = Ctx()
        try:
            self.s_detect(ctx)
            self.s_estimate(ctx)
            self.choose_grasp(ctx, ctx.T_B_P, 'PLAN_CHECK')
        except StepFailed as exc:
            return False, f'plan check failed: {exc}'
        b, M = self.bayonet, ctx.T_B_M
        spec = ctx.grasp_spec
        lift = Transform(ctx.T_B_P.R, ctx.T_B_P.t + spec.retreat.vector_in_base(ctx.grasp.grasp, ctx.T_B_P))
        poses = [('lift', lift), ('staging', b.staging(M)), ('pre_insert', b.pre_insert(M)), ('inserted', b.inserted(M))]
        poses += [(f'rotate_{i + 1}', T) for i, T in enumerate(b.rotation_path(M, lock=True))]
        report, ok = [], True
        for name, T_obj in poses:
            sol = self.arm.ik_tcp(T_obj @ ctx.T_P_tcp, self.ik_attempts, avoid_collisions=False)
            report.append(f'{name}: ' + (f'{self.arm.moveit.margin(sol):.2f}' if sol else 'NO IK'))
            ok &= sol is not None
        return ok, f'grasp "{ctx.grasp.name}"; IK joint-limit margins: ' + ', '.join(report) + '. Nothing was moved.'

    def srv_go_ready(self):
        try:
            self.go_ready()
            return True, 'ready pose reached'
        except StepFailed as exc:
            return False, str(exc)


def main(args=None):
    rclpy.init(args=args)
    node = PortafilterTask()
    ex = MultiThreadedExecutor(num_threads=6)
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

"""grasp_assist: Gazebo-only stand-in for the friction of a real pinch grip (SIMULATION ONLY).

Why: the bullet-featherstone engine (the only Gazebo Harmonic engine that enforces the OpenArm
finger mimic joint) models the pad contacts with box primitives; a 0.4 kg portafilter held
12 cm from its centre of mass creeps / rotates in the grasp (measured 0.06-0.35 deg/s, up to
8 deg during a lift), which a rubber-padded real gripper does not do.

What: the robot description (xacro arg grasp_assist_child_model) carries a
gz::sim::systems::DetachableJoint between the left hand and the object. This node
  * detaches it right after start (the plugin attaches when it loads),
  * ATTACHES only when the physical grasp conditions hold: the finger joint is stalled inside
    the closed-on-object window and the TCP lies on the object's grasp region (ground truth),
  * DETACHES as soon as the fingers open beyond `release_position` (or close past the window).
The fixed joint is created at the CURRENT relative pose (no snapping). The task software does
not know about this node: it still verifies every grasp with the finger angle and the camera,
and the bayonet / dock contacts stay fully physical. Run without it with grasp_assist:=false.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from visual_manipulation_core.ros_conversions import transform_from_tf
from visual_manipulation_core.se3 import Transform

from .gz_world import GzWorld, empty


class GraspAssist(Node):
    def __init__(self):
        super().__init__('grasp_assist')
        P = self.declare_parameter
        self.world = GzWorld(P('world', 'openarm_coffee').value)
        self.model = P('child_model', 'portafilter').value
        self.frame = P('planning_frame', 'world').value
        self.tip = P('tip_link', 'openarm_left_ee_base_link').value
        self.flange_to_tcp = Transform.from_flat(P('flange_to_tcp', [0.0, 0.0, -0.16, 0.0, 0.0, 0.0]).value)
        self.joint = P('finger_joint', 'openarm_left_finger_joint1').value
        self.q_min = P('closed_min', 0.10).value
        self.q_max = P('closed_max', 0.22).value
        self.q_release = P('release_position', 0.26).value
        self.settle = P('settle_time', 0.6).value
        self.max_rate = P('max_finger_rate', 0.02).value
        # grasp region in the OBJECT frame: x range, |y| and |z - z0| limits
        self.region = list(P('grasp_region', [0.06, 0.20, 0.025, -0.034, 0.02]).value)
        self.attach_pub = self.world.publisher(P('attach_topic', '/grasp_assist/attach').value)
        self.detach_pub = self.world.publisher(P('detach_topic', '/grasp_assist/detach').value)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.q = None
        self.q_hist = []
        self.attached = True             # the plugin attaches on load
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.first_js = None
        self.create_timer(0.05, self.step)
        self.create_timer(0.005, self.startup_detach)     # 200 Hz until the robot is up
        self.get_logger().info(f'grasp assist for "{self.model}" (SIMULATION ONLY): attach when {self.joint} is stalled '
                               f'in [{self.q_min}, {self.q_max}] rad on the grasp region, detach above {self.q_release} rad')

    def startup_detach(self):
        """The plugin attaches as soon as it loads (robot spawn). Until joint states have been
        flowing for 2 s (controllers active - nothing can move before), keep requesting detach
        at 200 Hz so the weld exists only for a few physics steps."""
        if self.first_js is None or time.monotonic() - self.first_js < 2.0:
            self.detach_pub.publish(empty())
            self.attached = False

    def on_js(self, msg):
        if self.first_js is None:
            self.first_js = time.monotonic()
        if self.joint in msg.name:
            self.q = msg.position[msg.name.index(self.joint)]
            now = time.monotonic()
            self.q_hist = [(t, q) for t, q in self.q_hist if now - t < self.settle] + [(now, self.q)]

    def tcp_in_object(self):
        T_obj = self.world.pose(self.model)
        if T_obj is None:
            return None
        try:
            tf = self.tf.lookup_transform(self.frame, self.tip, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        tcp = transform_from_tf(tf) @ self.flange_to_tcp
        return T_obj.inverse() @ tcp.t

    def on_region(self, p):
        x0, x1, ymax, z0, dz = self.region
        return p is not None and x0 <= p[0] <= x1 and abs(p[1]) <= ymax and abs(p[2] - z0) <= dz

    def set(self, attach: bool, why: str):
        (self.attach_pub if attach else self.detach_pub).publish(empty())
        if attach != self.attached:
            self.get_logger().info(f'{"ATTACH" if attach else "DETACH"} {self.model}: {why}')
        self.attached = attach
        self.state_pub.publish(String(data='attached' if attach else 'detached'))

    def step(self):
        if self.first_js is None or time.monotonic() - self.first_js < 2.0 or self.q is None:
            return
        if self.attached:
            if self.q > self.q_release or self.q < self.q_min - 0.05:
                self.set(False, f'fingers opened / closed past the object ({self.q:.3f} rad)')
            return
        if not (self.q_min <= self.q <= self.q_max) or len(self.q_hist) < 3:
            return
        span = self.q_hist[-1][0] - self.q_hist[0][0]
        if span < self.settle * 0.8:
            return
        rate = abs(self.q_hist[-1][1] - self.q_hist[0][1]) / max(span, 1e-3)
        p = self.tcp_in_object()
        if rate <= self.max_rate and self.on_region(p):
            self.set(True, f'fingers stalled at {self.q:.3f} rad on the grasp region '
                           f'(tcp in object frame {np.round(p, 3).tolist()})')


def main(args=None):
    rclpy.init(args=args)
    node = GraspAssist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

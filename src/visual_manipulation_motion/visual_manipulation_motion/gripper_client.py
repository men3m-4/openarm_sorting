"""Parallel / pinch gripper through the standard control_msgs/action/GripperCommand interface
plus the measured finger joint (sensor_msgs/JointState). Robot-independent: action name,
finger joint and all positions are parameters.

Grasp verification uses the measured finger position: closing on an object of the expected
width stops the fingers inside [grasp_min, grasp_max]; an empty gripper closes past grasp_min.
With position-controlled grippers the controller never reports 'reached' while squeezing an
object, so `squeeze()` sends the goal and waits for the fingers to settle instead of waiting
for the action result (the goal stays active and keeps the grip force).
"""
import threading
import time

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from .ros_utils import run_action, wait_future


class GripperClient:
    def __init__(self, node, action: str, joint: str, open_position: float, close_position: float,
                 grasp_min: float, grasp_max: float, max_effort: float = 0.0,
                 joint_states_topic: str = '/joint_states'):
        self.node = node
        self.joint = joint
        self.open_position = float(open_position)
        self.close_position = float(close_position)
        self.grasp_min = float(grasp_min)
        self.grasp_max = float(grasp_max)
        self.max_effort = float(max_effort)
        self.client = ActionClient(node, GripperCommand, action)
        self._lock = threading.Lock()
        self._q = None
        self._stamp = 0.0
        node.create_subscription(JointState, joint_states_topic, self._on_js, 20)

    def _on_js(self, msg):
        if self.joint in msg.name:
            with self._lock:
                self._q = msg.position[msg.name.index(self.joint)]
                self._stamp = time.monotonic()

    @property
    def position(self):
        with self._lock:
            return self._q

    def ready(self, timeout=1.0) -> bool:
        return self.client.wait_for_server(timeout_sec=timeout) and self._q is not None

    def _goal(self, position, max_effort=None):
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = float(self.max_effort if max_effort is None else max_effort)
        return g

    def move(self, position, timeout=10.0):
        """Move and wait for the result: (final_position, reached_goal, stalled)."""
        r = run_action(self.client, self._goal(position), timeout, 'gripper_command').result
        return r.position, r.reached_goal, r.stalled

    def squeeze(self, position=None, settle=2.5):
        """Close onto an object; returns the settled finger position."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('gripper action server not available')
        h = wait_future(self.client.send_goal_async(self._goal(self.close_position if position is None else position)),
                        5.0, 'gripper goal')
        if not h.accepted:
            raise RuntimeError('gripper goal rejected')
        time.sleep(settle)
        return self.position

    def open(self, position=None, settle=0.6):
        target = self.open_position if position is None else float(position)
        self.move(target)
        time.sleep(settle)
        return self.position is not None and self.position >= target - 0.05

    def holding(self, q=None) -> bool:
        q = self.position if q is None else q
        return q is not None and self.grasp_min <= q <= self.grasp_max

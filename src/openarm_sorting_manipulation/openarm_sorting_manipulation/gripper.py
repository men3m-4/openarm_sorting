"""LEFT pinch-gripper client: existing GripperActionController + measured finger angle.

Gripper geometry (from the OpenArm pinch_gripper collision meshes, see docs/packages/MANIPULATION.md):
  finger_joint1 = 0 rad      fingertip pads touch (closed)
  fingertip-pad gap          ~ 3.5 cm at 0.19 rad, ~ 4 cm at 0.22 rad (the cube), ~ 8.8 cm at 0.45 rad
  finger_joint2 is a URDF mimic joint, enforced by the bullet-featherstone physics engine
"""
import threading
import time

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

from .ros_utils import run_action, wait_future


class Gripper:
    def __init__(self, node, action: str = '/left_gripper_controller/gripper_cmd',
                 joint: str = 'openarm_left_finger_joint1', max_effort: float = 7.0):
        self.node = node
        self.joint = joint
        self.max_effort = float(max_effort)
        self.client = ActionClient(node, GripperCommand, action)
        self._lock = threading.Lock()
        self._q = None
        node.create_subscription(JointState, '/joint_states', self._on_js, 20)

    def _on_js(self, msg):
        if self.joint in msg.name:
            with self._lock:
                self._q = msg.position[msg.name.index(self.joint)]

    @property
    def position(self):
        with self._lock:
            return self._q

    def ready(self, timeout=1.0):
        return self.client.wait_for_server(timeout_sec=timeout) and self._q is not None

    def squeeze(self, position, settle=3.0):
        """Close onto an object. With position control the fingers stop on the object and the
        goal is never 'reached' - the controller keeps commanding `position`, which is what
        produces the grip force. So do NOT wait for (or cancel) the result: send the goal, let
        the fingers settle, and return the measured finger angle."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.max_effort
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('gripper action server not available')
        handle = wait_future(self.client.send_goal_async(goal), 5.0, 'gripper goal')
        if not handle.accepted:
            raise RuntimeError('gripper goal rejected')
        time.sleep(settle)
        return self.position

    def command(self, position, max_effort=None, timeout=10.0):
        """Returns (final_position, reached_goal, stalled). With position control and an
        object between the fingers the goal is not reached; the result still reports the
        position where the fingers stopped, which is what grasp verification uses."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.max_effort if max_effort is None else max_effort)
        wrapped = run_action(self.client, goal, timeout, 'gripper_cmd')
        r = wrapped.result
        return r.position, r.reached_goal, r.stalled

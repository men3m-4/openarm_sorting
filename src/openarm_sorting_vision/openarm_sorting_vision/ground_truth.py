"""Gazebo ground-truth model poses, read straight from Gazebo Transport (TEST TOOLS ONLY).

ros_gz_bridge's Pose_V -> tf2_msgs/TFMessage conversion drops the entity names, so the
named poses are read with the Gazebo Python bindings instead. Nothing in the robot's
decision making uses this module; it exists to *validate* perception and grasps.
"""
import math
import threading

import numpy as np

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
except ImportError:  # pragma: no cover - only on systems without Gazebo Harmonic
    GzNode = None


class GroundTruth:
    def __init__(self, world='openarm_cup_sorting'):
        if GzNode is None:
            raise RuntimeError('Gazebo Python bindings (gz.transport13) not available')
        self._lock = threading.Lock()
        self._poses = {}
        self._node = GzNode()
        topic = f'/world/{world}/dynamic_pose/info'
        if not self._node.subscribe(Pose_V, topic, self._cb):
            raise RuntimeError(f'could not subscribe to {topic}')

    def _cb(self, msg):
        with self._lock:
            for p in msg.pose:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self._poses[p.name] = (np.array([p.position.x, p.position.y, p.position.z]), yaw)

    def get(self, name):
        """(xyz ndarray, yaw) of a dynamic model, or None if not seen yet."""
        with self._lock:
            return self._poses.get(name)

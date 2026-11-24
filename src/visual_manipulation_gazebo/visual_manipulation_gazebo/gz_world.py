"""Gazebo Transport access shared by the Gazebo-only tools (ground truth, grasp assist).

ros_gz_bridge's Pose_V -> TFMessage conversion drops the entity names, so named model poses
are read with the Gazebo Python bindings (gz.transport13 / gz.msgs10) instead.
NOTHING in the core framework imports this module.
"""
import threading

from visual_manipulation_core.se3 import Transform

try:
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node as GzNode
except ImportError:  # pragma: no cover - only without Gazebo Harmonic
    GzNode = None


class GzWorld:
    def __init__(self, world: str):
        if GzNode is None:
            raise RuntimeError('Gazebo Python bindings (gz.transport13 / gz.msgs10) not available')
        self.node = GzNode()
        self._lock = threading.Lock()
        self._poses = {}
        topic = f'/world/{world}/dynamic_pose/info'
        if not self.node.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f'could not subscribe to {topic}')

    def _on_poses(self, msg):
        with self._lock:
            for p in msg.pose:
                q = p.orientation
                self._poses[p.name] = Transform.from_xyz_quat([p.position.x, p.position.y, p.position.z],
                                                              [q.x, q.y, q.z, q.w])

    def pose(self, model: str):
        """World pose (Transform) of a dynamic model, or None if not seen yet."""
        with self._lock:
            return self._poses.get(model)

    def publisher(self, topic: str, msg_type=None):
        return self.node.advertise(topic, msg_type or Empty)

    def subscribe_string(self, topic: str, cb):
        return self.node.subscribe(StringMsg, topic, lambda m: cb(m.data))


def empty():
    return Empty()

"""Planning-scene shape conversion (needs the ROS message Python types only)."""
import math

import numpy as np

from visual_manipulation_core.se3 import Transform
from visual_manipulation_motion.scene import collision_object
from shape_msgs.msg import SolidPrimitive


def test_box_and_cylinder_placed_at_object_pose():
    T = Transform.from_xyz_rpy([1.0, 2.0, 3.0], [0, 0, math.pi / 2])
    o = collision_object('o', 'world', T, [
        {'type': 'box', 'pose': [0.1, 0, 0, 0, 0, 0], 'dimensions': [0.1, 0.2, 0.3]},
        {'type': 'cylinder', 'pose': [0, 0, 0.05, 0, 0, 0], 'dimensions': [0.1, 0.02]}])
    assert [p.type for p in o.primitives] == [SolidPrimitive.BOX, SolidPrimitive.CYLINDER]
    p = o.primitive_poses[0].position
    assert np.allclose([p.x, p.y, p.z], [1.0, 2.1, 3.0])        # +x rotated to +y
    assert o.header.frame_id == 'world'


def test_ring_with_gap():
    o = collision_object('r', 'world', Transform(), [
        {'type': 'ring', 'pose': [0] * 6, 'dimensions': [0.03, 0.04, 0.05, 12, 60]}])
    # 12 segments of 30 deg; the 60 deg gap at +x removes the segments at 0 and +-30 deg
    assert len(o.primitives) == 9
    for pose in o.primitive_poses:
        r = math.hypot(pose.position.x, pose.position.y)
        assert math.isclose(r, 0.045, abs_tol=1e-9)
        assert abs(math.atan2(pose.position.y, pose.position.x)) > math.radians(40)

"""Conversions between the core `Transform` and ROS geometry messages (needs only the
geometry_msgs Python types, not rclpy; the rest of this package has no ROS dependency)."""
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped

from visual_manipulation_core.se3 import Transform, matrix_to_quat


def rotation_to_quaternion_msg(R) -> Quaternion:
    x, y, z, w = (float(v) for v in matrix_to_quat(R))
    return Quaternion(x=x, y=y, z=z, w=w)


def transform_from_pose(p: Pose) -> Transform:
    q = p.orientation
    return Transform.from_xyz_quat([p.position.x, p.position.y, p.position.z], [q.x, q.y, q.z, q.w])


def transform_from_tf(tf: TransformStamped) -> Transform:
    t, q = tf.transform.translation, tf.transform.rotation
    return Transform.from_xyz_quat([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def pose_from_transform(T: Transform) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T.t)
    p.orientation = rotation_to_quaternion_msg(T.R)
    return p


def pose_stamped(T: Transform, frame: str, stamp=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose = pose_from_transform(T)
    return ps


def tf_from_transform(T: Transform, parent: str, child: str, stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = stamp
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in T.t)
    t.transform.rotation = rotation_to_quaternion_msg(T.R)
    return t

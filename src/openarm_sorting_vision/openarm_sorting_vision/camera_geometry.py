"""Camera geometry helpers: pinhole back-projection and rigid transforms (NumPy only).

Coordinate conventions (verified numerically, see docs/packages/PERCEPTION.md):

* Image pixels (u right, v down) + Gazebo depth (metres, Z distance along the optical axis,
  NOT the Euclidean ray length) back-project with the CameraInfo K matrix into the ROS
  OPTICAL frame  openarm_zed_sim_camera_optical_frame  (x right, y down, z forward):
        x = (u - cx) * Z / fx,   y = (v - cy) * Z / fy,   z = Z
* Gazebo stamps the images with  openarm_zed_sim_camera_link  (x forward, y left, z up),
  which is the frame the /openarm_zed/points cloud is expressed in. The two frames differ
  by the fixed rotation  R_link_from_optical  below (URDF joint rpy = -pi/2, 0, -pi/2):
        x_link = z_opt,   y_link = -x_opt,   z_link = -y_opt
"""
import numpy as np

R_LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])


def back_project(us: np.ndarray, vs: np.ndarray, depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Pixels (u, v) with z-depth -> Nx3 points in the optical frame."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (us - cx) * depth / fx
    y = (vs - cy) * depth / fy
    return np.stack([x, y, depth], axis=1)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit (or non-unit) quaternion -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def transform_to_matrix(tf_msg) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    t, q = tf_msg.transform.translation, tf_msg.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def apply(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to Nx3 points."""
    return pts @ T[:3, :3].T + T[:3, 3]


def yaw_to_quaternion(yaw: float) -> tuple:
    """Rotation about z -> (x, y, z, w)."""
    return (0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0)))

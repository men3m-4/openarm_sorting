"""Rigid transforms (SE(3)) with NumPy only.

Conventions used throughout the framework:
  * quaternions are (x, y, z, w) like geometry_msgs/Quaternion
  * rpy = fixed-axis roll, pitch, yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (URDF / tf2 convention)
  * `Transform` T_a_b maps coordinates in frame b to frame a (p_a = T_a_b * p_b); it is also
    "the pose of frame b expressed in frame a". Composition: T_a_c = T_a_b @ T_b_c.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence

import numpy as np


# ---------------------------------------------------------------------------- rotations
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R: np.ndarray):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: put everything in roll
        roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return roll, pitch, yaw


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError('zero quaternion')
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (x, y, z, w) with w >= 0."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def axis_angle_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def rotation_angle(R: np.ndarray) -> float:
    """Angle of a rotation matrix [rad, 0..pi]."""
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))


def rotation_log(R: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle) of R."""
    ang = rotation_angle(R)
    if ang < 1e-9:
        return np.zeros(3)
    if abs(ang - math.pi) < 1e-6:
        # axis from the largest diagonal element of (R + I) / 2
        B = (R + np.eye(3)) / 2
        i = int(np.argmax(np.diag(B)))
        axis = B[:, i] / math.sqrt(B[i, i])
        return axis * ang
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
    return w * ang


def quat_slerp(q0, q1, t: float) -> np.ndarray:
    q0 = np.asarray(q0, float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, float) / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


def average_quaternions(quats: Iterable[Sequence[float]], weights: Optional[Sequence[float]] = None) -> np.ndarray:
    """Weighted quaternion mean (Markley et al. 2007: principal eigenvector of sum w q q^T).
    Sign-invariant, so q and -q are treated as the same rotation."""
    Q = np.array([np.asarray(q, float) / np.linalg.norm(q) for q in quats])
    w = np.ones(len(Q)) if weights is None else np.asarray(weights, float)
    M = (Q * w[:, None]).T @ Q
    vals, vecs = np.linalg.eigh(M)
    q = vecs[:, int(np.argmax(vals))]
    return -q if q[3] < 0 else q


# ---------------------------------------------------------------------------- Transform
class Transform:
    """Rigid transform; see module docstring for the conventions."""

    __slots__ = ('R', 't')

    def __init__(self, R: Optional[np.ndarray] = None, t: Optional[Sequence[float]] = None):
        self.R = np.eye(3) if R is None else np.array(R, dtype=float).reshape(3, 3)
        self.t = np.zeros(3) if t is None else np.array(t, dtype=float).reshape(3)

    # ---- construction
    @classmethod
    def identity(cls) -> 'Transform':
        return cls()

    @classmethod
    def from_xyz_rpy(cls, xyz=(0, 0, 0), rpy=(0, 0, 0)) -> 'Transform':
        return cls(rpy_to_matrix(*rpy), xyz)

    @classmethod
    def from_xyz_quat(cls, xyz, quat) -> 'Transform':
        return cls(quat_to_matrix(quat), xyz)

    @classmethod
    def from_matrix(cls, M: np.ndarray) -> 'Transform':
        M = np.asarray(M, float)
        return cls(M[:3, :3], M[:3, 3])

    @classmethod
    def from_dict(cls, d) -> 'Transform':
        """{'xyz': [..], 'rpy': [..]} or {'xyz': [..], 'quat': [x, y, z, w]} (missing = zero)."""
        if d is None:
            return cls()
        xyz = d.get('xyz', (0, 0, 0))
        if 'quat' in d:
            return cls.from_xyz_quat(xyz, d['quat'])
        return cls.from_xyz_rpy(xyz, d.get('rpy', (0, 0, 0)))

    @classmethod
    def from_flat(cls, values: Sequence[float]) -> 'Transform':
        """[x, y, z, roll, pitch, yaw] (6) or [x, y, z, qx, qy, qz, qw] (7) - the flat form used
        for ROS parameters (ROS 2 parameters cannot be nested dicts)."""
        v = [float(x) for x in values]
        if len(v) == 6:
            return cls.from_xyz_rpy(v[:3], v[3:])
        if len(v) == 7:
            return cls.from_xyz_quat(v[:3], v[3:])
        raise ValueError(f'a transform needs 6 (xyz rpy) or 7 (xyz quat) values, got {len(v)}')

    @classmethod
    def translation(cls, x=0.0, y=0.0, z=0.0) -> 'Transform':
        return cls(None, (x, y, z))

    @classmethod
    def rotation_about_z(cls, angle: float) -> 'Transform':
        return cls(rot_z(angle))

    # ---- conversion
    def matrix(self) -> np.ndarray:
        M = np.eye(4)
        M[:3, :3] = self.R
        M[:3, 3] = self.t
        return M

    def quat(self) -> np.ndarray:
        return matrix_to_quat(self.R)

    def rpy(self):
        return matrix_to_rpy(self.R)

    def to_dict(self, digits=6):
        return {'xyz': [round(float(v), digits) for v in self.t],
                'rpy': [round(float(v), digits) for v in self.rpy()]}

    # ---- algebra
    def inverse(self) -> 'Transform':
        Rt = self.R.T
        return Transform(Rt, -Rt @ self.t)

    def __matmul__(self, other):
        if isinstance(other, Transform):
            return Transform(self.R @ other.R, self.R @ other.t + self.t)
        p = np.asarray(other, float)
        if p.ndim == 1:
            return self.R @ p + self.t
        return p @ self.R.T + self.t

    def apply(self, points) -> np.ndarray:
        return self @ np.asarray(points, float)

    def interpolate(self, other: 'Transform', s: float) -> 'Transform':
        """Linear in translation, slerp in rotation (s = 0 -> self, 1 -> other)."""
        q = quat_slerp(self.quat(), other.quat(), s)
        return Transform(quat_to_matrix(q), (1 - s) * self.t + s * other.t)

    def __repr__(self):
        r, p, y = self.rpy()
        return (f'Transform(xyz=[{self.t[0]:.4f}, {self.t[1]:.4f}, {self.t[2]:.4f}], '
                f'rpy=[{r:.4f}, {p:.4f}, {y:.4f}])')


def pose_error(a: Transform, b: Transform):
    """(translation distance [m], rotation angle [rad]) between two poses in the same frame."""
    return float(np.linalg.norm(a.t - b.t)), rotation_angle(a.R.T @ b.R)


def average_transforms(transforms: Sequence[Transform], weights=None, robust=True) -> Transform:
    """Mean pose: component-wise median (robust=True) or weighted mean of the translations,
    Markley mean of the rotations."""
    ts = np.array([T.t for T in transforms])
    if robust:
        t = np.median(ts, axis=0)
    else:
        w = np.ones(len(ts)) if weights is None else np.asarray(weights, float)
        t = (ts * w[:, None]).sum(axis=0) / w.sum()
    q = average_quaternions([T.quat() for T in transforms], weights)
    return Transform.from_xyz_quat(t, q)


def yaw_of_axis(R: np.ndarray, axis: int = 0) -> float:
    """Heading (atan2 in the xy plane) of one column (axis) of R."""
    v = R[:, axis]
    return math.atan2(v[1], v[0])


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

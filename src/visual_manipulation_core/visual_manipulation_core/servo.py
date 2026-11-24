"""Position-based visual servoing (PBVS) of a HELD object (eye-to-hand camera).

The camera measures the held object T_B_P (via its marker) while the robot reports its TCP
T_B_tcp. Assuming the grasp is rigid for the duration of one correction, the TCP pose that
brings the object to the desired pose T_B_P* is

    T_P_tcp   = inv(T_B_P) @ T_B_tcp            (measured in-hand transform)
    T_B_tcp*  = T_B_P* @ T_P_tcp

One servo step moves a fraction `gain` of the way (clamped to max_translation / max_rotation)
and the loop repeats with a NEW measurement until the object error is below tolerance. This
compensates the camera-to-robot calibration error, the marker-to-object model error and
in-hand slip, because all of them appear in the measured object error.

`correct_tilt=False` corrects position and the rotation about `axis` only (useful when the
marker orientation is less accurate than its position, e.g. small markers).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .se3 import Transform, axis_angle_matrix, pose_error


@dataclass
class ServoResult:
    target_tcp: Transform        # next TCP pose to command
    position_error: float        # current object position error [m]
    angle_error: float           # current object rotation error [rad] (about `axis` if no tilt)
    converged: bool


def heading_difference(R_from: np.ndarray, R_to: np.ndarray, axis: Sequence[float] = (0, 0, 1)) -> float:
    """Signed angle about `axis` from the x axis of R_from to the x axis of R_to, both projected
    onto the plane normal to `axis` (exact even when the frames are tilted)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)

    def proj(v):
        v = v - (v @ a) * a
        n = np.linalg.norm(v)
        if n < 1e-9:
            raise ValueError('x axis parallel to the heading axis')
        return v / n
    u, v = proj(R_from[:, 0]), proj(R_to[:, 0])
    return math.atan2(float(np.cross(u, v) @ a), float(u @ v))


def object_error(T_B_P: Transform, T_B_P_des: Transform, axis: Sequence[float] = (0, 0, 1),
                 correct_tilt: bool = True):
    dpos, dang = pose_error(T_B_P, T_B_P_des)
    if not correct_tilt:
        dang = abs(heading_difference(T_B_P.R, T_B_P_des.R, axis))
    return dpos, dang


def pbvs_step(T_B_tcp: Transform, T_B_P: Transform, T_B_P_des: Transform, gain: float = 1.0,
              max_translation: float = 0.02, max_rotation: float = math.radians(8.0),
              position_tolerance: float = 0.0015, angle_tolerance: float = math.radians(1.5),
              correct_tilt: bool = True, axis: Sequence[float] = (0, 0, 1)) -> ServoResult:
    dpos, dang = object_error(T_B_P, T_B_P_des, axis, correct_tilt)
    converged = dpos <= position_tolerance and dang <= angle_tolerance
    desired = T_B_P_des
    if not correct_tilt:
        # keep the measured tilt, correct only the rotation about `axis` (base frame)
        dyaw = heading_difference(T_B_P.R, T_B_P_des.R, axis)
        desired = Transform(axis_angle_matrix(axis, dyaw) @ T_B_P.R, T_B_P_des.t)
    T_P_tcp = T_B_P.inverse() @ T_B_tcp
    full = desired @ T_P_tcp
    st, sr = pose_error(T_B_tcp, full)
    s = float(gain)
    if st * s > max_translation:
        s = max_translation / st
    if sr * s > max_rotation:
        s = max_rotation / sr
    return ServoResult(T_B_tcp.interpolate(full, min(1.0, s)), dpos, dang, converged)

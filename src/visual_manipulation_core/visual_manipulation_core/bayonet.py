"""Insertion + rotation (bayonet) planning for a held object and a measured mount frame.

A bayonet mount (espresso group head, lens mount, twist-lock connector, ...) is described in
its mount frame M (+z = insertion direction) by:

  locked_offset    T_M_P of the object frame P when fully inserted and LOCKED
  lock_angle       signed rotation about +z_M from 'inserted, unlocked' to 'locked' [rad]
  insertion_depth  travel along +z_M from 'pre-insert' to 'inserted' [m]
  staging_offset   offset (in M) of the 'staging' pose from the pre-insert pose, i.e. where the
                   object waits before the final aligned approach (e.g. in front of the mount)

Object poses along the sequence (d = remaining depth, a = angle relative to locked):

  T_M_P(d, a) = Trans(0, 0, -d) @ Rz(a) @ locked_offset

  staging    : Trans(staging_offset) @ T_M_P(depth, -lock)
  pre-insert : T_M_P(depth, -lock)     aligned with the mount, lugs below the lip
  inserted   : T_M_P(0, -lock)         lugs passed the notches
  locked     : T_M_P(0, 0)             rotated by +lock about the mount axis

The planner produces OBJECT poses; the coordinator converts them into TCP poses with the
(measured) in-hand transform T_P_tcp, so any grasp works and in-hand slip is compensated.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence

import numpy as np

from .se3 import Transform, rot_z, wrap_angle


@dataclass
class BayonetSpec:
    locked_offset: Transform = field(default_factory=Transform)
    lock_angle: float = math.radians(45.0)
    insertion_depth: float = 0.04
    staging_offset: Sequence[float] = (0.10, 0.0, 0.0)
    rotation_step: float = math.radians(5.0)

    def __post_init__(self):
        if self.insertion_depth <= 0:
            raise ValueError('insertion_depth must be > 0')
        if self.rotation_step <= 0:
            raise ValueError('rotation_step must be > 0')
        self.staging_offset = np.asarray(self.staging_offset, float)


class BayonetPlanner:
    def __init__(self, spec: BayonetSpec):
        self.spec = spec

    # ------------------------------------------------------------ object poses in the base frame
    def object_in_mount(self, depth: float, angle: float) -> Transform:
        return Transform.translation(0, 0, -depth) @ Transform(rot_z(angle)) @ self.spec.locked_offset

    def at(self, T_B_M: Transform, depth: float, angle: float) -> Transform:
        return T_B_M @ self.object_in_mount(depth, angle)

    def staging(self, T_B_M: Transform) -> Transform:
        s = self.spec
        return T_B_M @ Transform.translation(*s.staging_offset) @ self.object_in_mount(s.insertion_depth, -s.lock_angle)

    def pre_insert(self, T_B_M: Transform) -> Transform:
        return self.at(T_B_M, self.spec.insertion_depth, -self.spec.lock_angle)

    def inserted(self, T_B_M: Transform) -> Transform:
        return self.at(T_B_M, 0.0, -self.spec.lock_angle)

    def locked(self, T_B_M: Transform) -> Transform:
        return self.at(T_B_M, 0.0, 0.0)

    # ------------------------------------------------------------ paths
    def insertion_path(self, T_B_M: Transform, steps: int = 1) -> List[Transform]:
        """pre-insert -> inserted, straight along +z_M (the end pose is included)."""
        d = self.spec.insertion_depth
        return [self.at(T_B_M, d * (1 - k / steps), -self.spec.lock_angle) for k in range(1, steps + 1)]

    def rotation_path(self, T_B_M: Transform, lock: bool = True) -> List[Transform]:
        """Waypoints of the rotation about the mount axis at full depth (the start pose is NOT
        included). lock=True: inserted -> locked; False: locked -> inserted (unlock)."""
        a0, a1 = (-self.spec.lock_angle, 0.0) if lock else (0.0, -self.spec.lock_angle)
        n = max(1, int(math.ceil(abs(a1 - a0) / self.spec.rotation_step)))
        return [self.at(T_B_M, 0.0, a0 + (a1 - a0) * k / n) for k in range(1, n + 1)]

    def extraction_path(self, T_B_M: Transform, steps: int = 1) -> List[Transform]:
        """inserted -> pre-insert (reverse of the insertion)."""
        d = self.spec.insertion_depth
        return [self.at(T_B_M, d * k / steps, -self.spec.lock_angle) for k in range(1, steps + 1)]

    # ------------------------------------------------------------ verification
    def error(self, T_B_M: Transform, T_B_P: Transform, depth: float = 0.0, angle: float = 0.0):
        """Deviation of a measured object pose from the expected pose T_M_P(depth, angle):
        returns dict(position=[dx, dy, dz] in M, radial, axial, angle (about +z_M), tilt)."""
        T_M_P = T_B_M.inverse() @ T_B_P
        exp = self.object_in_mount(depth, angle)
        dp = T_M_P.t - exp.t
        R_err = exp.R.T @ T_M_P.R
        # rotation about the mount axis = heading change of the object x axis seen in M
        ang = wrap_angle(math.atan2(T_M_P.R[1, 0], T_M_P.R[0, 0]) - math.atan2(exp.R[1, 0], exp.R[0, 0]))
        tilt = math.acos(max(-1.0, min(1.0, float(T_M_P.R[:, 2] @ exp.R[:, 2]))))
        return {'position': dp, 'radial': float(math.hypot(dp[0], dp[1])), 'axial': float(dp[2]),
                'angle': ang, 'tilt': tilt, 'rotation': float(np.degrees(np.arccos(
                    max(-1.0, min(1.0, (np.trace(R_err) - 1) / 2)))))}

    def classify(self, T_B_M: Transform, T_B_P: Transform, radial_tol=0.006, axial_tol=0.006,
                 angle_tol=math.radians(6.0), out_radius=0.03) -> str:
        """Bayonet state from a measured object pose:
          'locked'      at full depth, within the tolerances, at the locked angle
          'inserted'    at full depth, at the unlocked (insertion) angle
          'partially_rotated' at full depth, between the two angles
          'misaligned'  near / partly in the mount but not in a defined state
          'out'         clearly away from the mount (radial > out_radius or below the
                        pre-insert height) - the only state in which it is free to be moved
        The tolerances should reflect the mechanical clearances: a released, locked part hangs
        on its lugs and may sit a few mm off-centre / tilted."""
        e_lock = self.error(T_B_M, T_B_P, 0.0, 0.0)
        if e_lock['radial'] <= radial_tol and abs(e_lock['axial']) <= axial_tol:
            if abs(e_lock['angle']) <= angle_tol:
                return 'locked'
            e_ins = self.error(T_B_M, T_B_P, 0.0, -self.spec.lock_angle)
            if abs(e_ins['angle']) <= angle_tol:
                return 'inserted'
            return 'partially_rotated'
        if e_lock['radial'] > out_radius or e_lock['axial'] < -(self.spec.insertion_depth + 0.01):
            return 'out'
        return 'misaligned'

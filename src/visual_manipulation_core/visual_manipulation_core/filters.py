"""Temporal pose filter with a stability gate.

Fiducial poses of static objects are averaged over a short window; the estimate is reported as
STABLE only when every sample in the window lies within the position / angle tolerance of the
filtered pose. Moving the object (or the camera) therefore drops the stability flag until the
window has refilled, so a stale pose is never used for a grasp.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional

from .se3 import Transform, average_transforms, pose_error


@dataclass
class FilteredPose:
    pose: Transform
    stable: bool
    num_samples: int
    position_spread: float
    angular_spread: float
    stamp: float                 # time of the newest sample [s]
    confidence: float


class PoseFilter:
    def __init__(self, window: int = 8, max_age: float = 1.5, position_tolerance: float = 0.004,
                 angle_tolerance: float = 0.05, min_samples: Optional[int] = None):
        self.window = int(window)
        self.max_age = float(max_age)
        self.pos_tol = float(position_tolerance)
        self.ang_tol = float(angle_tolerance)
        self.min_samples = int(min_samples or window)
        self._samples = deque(maxlen=self.window)     # (stamp, Transform, confidence)

    def reset(self):
        self._samples.clear()

    def add(self, stamp: float, pose: Transform, confidence: float = 1.0):
        if self._samples and stamp < self._samples[-1][0]:
            self._samples.clear()                    # time jumped back (sim reset)
        self._samples.append((float(stamp), pose, float(confidence)))

    def _recent(self, now: float):
        return [s for s in self._samples if now - s[0] <= self.max_age]

    def estimate(self, now: float) -> Optional[FilteredPose]:
        recent = self._recent(now)
        if not recent:
            return None
        poses = [s[1] for s in recent]
        weights = [max(s[2], 1e-3) for s in recent]
        mean = average_transforms(poses, weights)
        errs = [pose_error(mean, p) for p in poses]
        pos_spread = max(e[0] for e in errs)
        ang_spread = max(e[1] for e in errs)
        stable = (len(recent) >= self.min_samples and pos_spread <= self.pos_tol and ang_spread <= self.ang_tol)
        return FilteredPose(mean, stable, len(recent), pos_spread, ang_spread, recent[-1][0], recent[-1][2])

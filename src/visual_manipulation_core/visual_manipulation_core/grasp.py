"""Grasp pose generation from an object pose (robot- and gripper-independent).

The marker frame is NOT the grasp frame: an object is located by its marker (marker -> object
transform, perception config), and every grasp is a configured TCP pose in the OBJECT frame
(object -> TCP transform). The robot-specific flange is reached through the tool transform
(flange -> TCP) of the gripper in use.

    T_base_tcp(grasp)    = T_base_object @ T_object_tcp
    T_base_tcp(pregrasp) = grasp moved back along the approach direction by `approach.distance`
    T_base_tcp(lift)     = grasp moved along the retreat direction by `retreat.distance`
    T_base_flange        = T_base_tcp @ inv(T_flange_tcp)

Directions can be given in the base frame (e.g. "approach straight down"), in the tool frame
(e.g. "along the fingers") or in the object frame (e.g. "along the handle").
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence

import numpy as np

from .se3 import Transform

FRAMES = ('base', 'tool', 'object')


@dataclass
class Motion:
    """A straight TCP displacement: unit `direction` in `frame`, length `distance` [m]."""
    direction: Sequence[float] = (0.0, 0.0, 1.0)
    distance: float = 0.0
    frame: str = 'base'

    def __post_init__(self):
        if self.frame not in FRAMES:
            raise ValueError(f'motion frame must be one of {FRAMES}, got {self.frame!r}')
        d = np.asarray(self.direction, float)
        n = np.linalg.norm(d)
        if n < 1e-9:
            raise ValueError('motion direction must be non-zero')
        self.direction = d / n

    def vector_in_base(self, T_base_tcp: Transform, T_base_object: Transform) -> np.ndarray:
        v = self.direction * self.distance
        if self.frame == 'tool':
            return T_base_tcp.R @ v
        if self.frame == 'object':
            return T_base_object.R @ v
        return v


@dataclass
class GraspSpec:
    name: str
    object_to_tcp: Transform
    approach: Motion = field(default_factory=lambda: Motion((0, 0, -1), 0.08, 'base'))
    retreat: Motion = field(default_factory=lambda: Motion((0, 0, 1), 0.08, 'base'))


@dataclass
class GraspCandidate:
    name: str
    pregrasp: Transform          # TCP poses in the base frame
    grasp: Transform
    lift: Transform

    def flange_poses(self, flange_to_tcp: Transform):
        inv = flange_to_tcp.inverse()
        return {k: getattr(self, k) @ inv for k in ('pregrasp', 'grasp', 'lift')}


def shifted(T: Transform, v: np.ndarray) -> Transform:
    return Transform(T.R.copy(), T.t + np.asarray(v, float))


def make_candidate(T_base_object: Transform, spec: GraspSpec) -> GraspCandidate:
    grasp = T_base_object @ spec.object_to_tcp
    # the approach direction is the direction the TCP MOVES towards the grasp
    pre = shifted(grasp, -spec.approach.vector_in_base(grasp, T_base_object))
    lift = shifted(grasp, spec.retreat.vector_in_base(grasp, T_base_object))
    return GraspCandidate(spec.name, pre, grasp, lift)


def generate_grasps(T_base_object: Transform, specs: Sequence[GraspSpec]) -> List[GraspCandidate]:
    """One candidate per spec, in the configured order (the planner tries them in that order
    and may re-rank by reachability)."""
    return [make_candidate(T_base_object, s) for s in specs]


def flange_from_tcp(T_base_tcp: Transform, flange_to_tcp: Transform) -> Transform:
    return T_base_tcp @ flange_to_tcp.inverse()


def tcp_from_flange(T_base_flange: Transform, flange_to_tcp: Transform) -> Transform:
    return T_base_flange @ flange_to_tcp


def object_to_tcp_measured(T_base_object: Transform, T_base_tcp: Transform) -> Transform:
    """In-hand transform from a measurement of the held object and the current TCP pose."""
    return T_base_object.inverse() @ T_base_tcp

"""ArUco detection and marker pose estimation (OpenCV + NumPy only, no ROS).

* Works with both OpenCV ArUco APIs: the class-based one (>= 4.7, `cv2.aruco.ArucoDetector`)
  and the function-based one (<= 4.6, `cv2.aruco.detectMarkers`).
* Pose: `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)` with the calibrated intrinsics and the
  real marker size. IPPE returns BOTH planar solutions; the ratio of their reprojection errors
  measures the well-known planar flip ambiguity and feeds the confidence value.
* Optional RGB-D refinement: if a depth image registered to the colour image is available,
  a plane is fitted to the depth points inside the marker; the marker centre is moved onto
  that plane along its viewing ray and the marker normal is replaced by the plane normal
  (the in-plane rotation from PnP is kept). Depth is optional - RGB-only cameras work.

Marker frame (OpenCV convention): origin at the centre of the black square, x to the right of
the printed marker, y up, z out of the marker face (towards a camera looking at it).
Camera frame: OpenCV / ROS optical frame (x right, y down, z forward).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

DICTIONARIES = {name: getattr(cv2.aruco, name) for name in dir(cv2.aruco) if name.startswith('DICT_')}


def marker_object_points(size: float) -> np.ndarray:
    """Corners in the marker frame, in the OpenCV detection order TL, TR, BR, BL (the order
    SOLVEPNP_IPPE_SQUARE requires)."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


@dataclass
class MarkerPose:
    marker_id: int
    size: float
    R: np.ndarray                    # camera <- marker rotation
    t: np.ndarray                    # marker centre in the camera frame [m]
    corners: np.ndarray              # 4x2 [px]
    reprojection_error: float        # RMS [px] of the chosen solution
    ambiguity_ratio: float           # err(best) / err(second); 0 = unambiguous, 1 = ambiguous
    side_length_px: float
    view_angle: float                # [rad]
    confidence: float = 0.0
    depth_refined: bool = False
    depth_info: Dict[str, float] = field(default_factory=dict)


class ArucoBackend:
    """Version-agnostic ArUco detector."""

    def __init__(self, dictionary: str = 'DICT_4X4_50', corner_refinement: str = 'apriltag',
                 parameters: Optional[Dict[str, float]] = None):
        if dictionary not in DICTIONARIES:
            raise ValueError(f'unknown ArUco dictionary {dictionary!r}')
        self.dictionary_name = dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[dictionary])
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.params = cv2.aruco.DetectorParameters_create()
        else:
            self.params = cv2.aruco.DetectorParameters()
        methods = {'none': 'CORNER_REFINE_NONE', 'subpix': 'CORNER_REFINE_SUBPIX',
                   'contour': 'CORNER_REFINE_CONTOUR', 'apriltag': 'CORNER_REFINE_APRILTAG'}
        if corner_refinement not in methods:
            raise ValueError(f'corner_refinement must be one of {sorted(methods)}')
        self.params.cornerRefinementMethod = getattr(cv2.aruco, methods[corner_refinement])
        for k, v in (parameters or {}).items():
            if not hasattr(self.params, k):
                raise ValueError(f'unknown ArUco detector parameter {k!r}')
            setattr(self.params, k, type(getattr(self.params, k))(v))
        self._detector = (cv2.aruco.ArucoDetector(self.dictionary, self.params)
                          if hasattr(cv2.aruco, 'ArucoDetector') else None)

    def detect(self, gray: np.ndarray) -> Tuple[List[np.ndarray], List[int]]:
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            return [], []
        return [c.reshape(4, 2).astype(np.float64) for c in corners], [int(i) for i in ids.ravel()]


def _rms_reprojection(obj, img, rvec, tvec, K, D) -> float:
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))


def view_angle(R: np.ndarray, t: np.ndarray) -> float:
    """Angle between the marker normal and the direction from the marker to the camera."""
    to_cam = -t / max(np.linalg.norm(t), 1e-9)
    return math.acos(max(-1.0, min(1.0, float(R[:, 2] @ to_cam))))


def confidence(reproj_px: float, ambiguity: float, side_px: float, angle: float,
               reproj_scale: float = 1.0, min_side_px: float = 12.0, full_side_px: float = 40.0,
               max_angle: float = math.radians(70.0)) -> float:
    """Heuristic detection quality in [0, 1] = product of four factors:
      reprojection  exp(-(e / reproj_scale)^2)        (sub-pixel corners -> ~1)
      ambiguity     1 - ratio                         (IPPE flip ambiguity)
      size          linear from min_side_px (0) to full_side_px (1)
      view angle    cos(angle) rescaled so that max_angle -> 0
    It is meant for ranking / gating detections, not as a calibrated probability."""
    c_rep = math.exp(-(reproj_px / reproj_scale) ** 2)
    c_amb = max(0.0, min(1.0, 1.0 - ambiguity))
    c_size = max(0.0, min(1.0, (side_px - min_side_px) / (full_side_px - min_side_px)))
    c_ang = max(0.0, min(1.0, (math.cos(angle) - math.cos(max_angle)) / (1.0 - math.cos(max_angle))))
    return c_rep * c_amb * c_size * c_ang


def estimate_pose(marker_id: int, corners: np.ndarray, size: float, K: np.ndarray, D: np.ndarray) -> Optional[MarkerPose]:
    obj = marker_object_points(size)
    img = np.ascontiguousarray(corners, dtype=np.float64).reshape(4, 2)
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if n < 1:
        return None
    sols = []
    for rv, tv in zip(rvecs, tvecs):
        e = _rms_reprojection(obj, img, rv, tv, K, D)
        sols.append((e, rv, tv))
    sols.sort(key=lambda s: s[0])
    e0, rv, tv = sols[0]
    # ambiguity: how close the alternative planar solution explains the corners
    ratio = (e0 + 1e-3) / (sols[1][0] + 1e-3) if len(sols) > 1 else 0.0
    R, _ = cv2.Rodrigues(rv)
    t = tv.reshape(3)
    if t[2] <= 0:
        return None
    side = float(np.mean([np.linalg.norm(img[i] - img[(i + 1) % 4]) for i in range(4)]))
    ang = view_angle(R, t)
    mp = MarkerPose(marker_id, size, R, t, img, e0, float(ratio), side, ang)
    mp.confidence = confidence(e0, mp.ambiguity_ratio, side, ang)
    return mp


def refine_with_depth(mp: MarkerPose, depth: np.ndarray, K: np.ndarray, min_points: int = 30,
                      shrink: float = 0.8, max_plane_rms: float = 0.004, max_shift: float = 0.03) -> bool:
    """Refine `mp` in place with a registered depth image (metres, z-depth). Returns True if
    the refinement was applied. The quad is shrunk towards its centre (`shrink`) so that edge
    pixels mixing marker and background depth are not used."""
    c = mp.corners.mean(axis=0)
    quad = (c + (mp.corners - c) * shrink).astype(np.int32)
    h, w = depth.shape[:2]
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, quad, 1)
    vs, us = np.nonzero(mask)
    d = depth[vs, us].astype(np.float64)
    ok = np.isfinite(d) & (d > 0.05) & (d < 10.0)
    info = {'points': int(ok.sum())}
    mp.depth_info = info
    if ok.sum() < min_points:
        return False
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = d[ok]
    P = np.stack([(us[ok] - cx) * z / fx, (vs[ok] - cy) * z / fy, z], axis=1)
    keep = np.ones(len(P), bool)
    for _ in range(3):                                   # plane fit with outlier rejection
        cen = P[keep].mean(axis=0)
        _, _, vt = np.linalg.svd(P[keep] - cen, full_matrices=False)
        n = vt[2]
        res = (P - cen) @ n
        sigma = max(np.std(res[keep]), 1e-4)
        keep = np.abs(res) < 3 * sigma
        if keep.sum() < min_points:
            return False
    rms = float(np.sqrt(np.mean(res[keep] ** 2)))
    info['plane_rms'] = rms
    if rms > max_plane_rms:
        return False
    if n @ (-cen) < 0:                                   # normal towards the camera
        n = -n
    if abs(float(n @ (mp.t / np.linalg.norm(mp.t)))) < 0.2:   # grazing view: unreliable
        return False
    # 3D corners = corner viewing rays intersected with the fitted plane; the marker pose is the
    # rigid (Kabsch) fit of the marker model onto them. Unlike PnP alone, this decouples the
    # in-plane rotation from the (depth-corrected) tilt and scale.
    rays = np.column_stack([(mp.corners[:, 0] - cx) / fx, (mp.corners[:, 1] - cy) / fy, np.ones(4)])
    dn = rays @ n
    if np.any(np.abs(dn) < 1e-6):
        return False
    X = rays * (float(n @ cen) / dn)[:, None]
    obj = marker_object_points(mp.size)
    R_fit, t_fit, fit_rms = kabsch(obj, X)
    info['corner_fit_rms'] = fit_rms
    if fit_rms > 0.25 * mp.size:              # corners inconsistent with the marker size
        return False
    info['normal_change_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, float(R_fit[:, 2] @ mp.R[:, 2])))))
    info['shift'] = float(np.linalg.norm(t_fit - mp.t))
    if info['shift'] > max_shift:
        return False
    mp.R, mp.t = R_fit, t_fit
    mp.view_angle = view_angle(mp.R, mp.t)
    mp.depth_refined = True
    return True


def kabsch(A: np.ndarray, B: np.ndarray):
    """Rigid transform (R, t) minimising |R a_i + t - b_i|; returns (R, t, rms)."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cb - R @ ca
    rms = float(np.sqrt(np.mean(np.sum((A @ R.T + t - B) ** 2, axis=1))))
    return R, t, rms

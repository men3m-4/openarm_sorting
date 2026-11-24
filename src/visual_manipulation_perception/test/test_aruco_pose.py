"""ArUco pose estimation on synthetic images with a known ground-truth pose (no ROS)."""
import math

import cv2
import numpy as np
import pytest

from visual_manipulation_perception.aruco_pose import (ArucoBackend, confidence, estimate_pose, kabsch,
                                                       marker_object_points, refine_with_depth)

K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
D = np.zeros(5)
SIZE = 0.05


def render(marker_id, R, t, size=SIZE, shape=(480, 640)):
    """Synthetic camera image of a marker (with a one-cell white margin) at pose (R, t)."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    cell = 60
    img = (cv2.aruco.generateImageMarker(d, marker_id, 6 * cell) if hasattr(cv2.aruco, 'generateImageMarker')
           else cv2.aruco.drawMarker(d, marker_id, 6 * cell))
    img = cv2.copyMakeBorder(img, cell, cell, cell, cell, cv2.BORDER_CONSTANT, value=255)
    h = size / 2 * 8 / 6
    quad = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    proj, _ = cv2.projectPoints(quad, cv2.Rodrigues(R)[0], t, K, D)
    src = np.array([[0, 0], [8 * cell, 0], [8 * cell, 8 * cell], [0, 8 * cell]], np.float32)
    H = cv2.getPerspectiveTransform(src, proj.reshape(4, 2).astype(np.float32))
    out = cv2.warpPerspective(img, H, (shape[1], shape[0]), flags=cv2.INTER_LINEAR, borderValue=128)
    return out


def depth_of_plane(R, t, shape=(480, 640)):
    n, d0 = R[:, 2], float(R[:, 2] @ t)
    v, u = np.mgrid[0:shape[0], 0:shape[1]]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones(shape)], axis=-1)
    return (d0 / (rays @ n)).astype(np.float32)          # z-depth of the marker plane


def tilted(deg_x, deg_y, yaw):
    rx, ry, rz = math.radians(deg_x), math.radians(deg_y), math.radians(yaw)
    Rx = cv2.Rodrigues(np.array([math.pi + rx, 0, 0]))[0]      # facing the camera
    return Rx @ cv2.Rodrigues(np.array([0, ry, 0]))[0] @ cv2.Rodrigues(np.array([0, 0, rz]))[0]


@pytest.mark.parametrize('ang', [(0, 0, 0), (25, 10, 30), (40, -20, -70)])
def test_detect_and_pose(ang):
    R = tilted(*ang)
    t = np.array([0.03, -0.02, 0.45])
    img = render(7, R, t)
    corners, ids = ArucoBackend('DICT_4X4_50', 'apriltag').detect(img)
    assert ids == [7]
    mp = estimate_pose(7, corners[0], SIZE, K, D)
    # 1 % of the range (frontal: the ambiguous tilt also shifts the centre slightly)
    assert np.linalg.norm(mp.t - t) < (0.006 if ang == (0, 0, 0) else 0.003)
    assert mp.reprojection_error < 1.0 and 0 <= mp.confidence <= 1
    rot_err = math.degrees(math.acos(min(1, (np.trace(mp.R.T @ R) - 1) / 2)))
    if ang != (0, 0, 0):
        assert rot_err < 1.0
    # fronto-parallel: the tilt is nearly unobservable from 4 corners (projection derivative ~0);
    # a few tenths of a pixel of corner bias give several degrees of tilt error and IPPE does
    # NOT flag it as ambiguous. Known RGB-only limitation; depth refinement removes it
    # (test_depth_refinement_fixes_frontal_tilt).


def test_depth_refinement_improves_pose():
    R = tilted(45, 0, 20)
    t = np.array([0.0, 0.05, 0.6])
    img = render(3, R, t, size=0.03)
    corners, ids = ArucoBackend('DICT_4X4_50', 'apriltag').detect(img)
    mp = estimate_pose(3, corners[0], 0.03, K, D)
    e_rgb = np.linalg.norm(mp.t - t)
    assert refine_with_depth(mp, depth_of_plane(R, t), K)
    assert mp.depth_refined
    assert np.linalg.norm(mp.t - t) < max(0.0015, e_rgb)
    assert math.degrees(math.acos(min(1.0, float(mp.R[:, 2] @ R[:, 2])))) < 0.5


def test_depth_refinement_rejects_missing_depth():
    R = tilted(10, 0, 0)
    t = np.array([0.0, 0.0, 0.5])
    corners, _ = ArucoBackend().detect(render(1, R, t))
    mp = estimate_pose(1, corners[0], SIZE, K, D)
    assert not refine_with_depth(mp, np.full((480, 640), np.nan, np.float32), K)
    assert not mp.depth_refined


def test_object_points_order_and_kabsch():
    P = marker_object_points(0.1)
    assert np.allclose(P[0], [-0.05, 0.05, 0]) and np.allclose(P[2], [0.05, -0.05, 0])
    R = tilted(30, 10, 40)
    t = np.array([0.1, 0.2, 0.3])
    R2, t2, rms = kabsch(P, P @ R.T + t)
    assert np.allclose(R2, R) and np.allclose(t2, t) and rms < 1e-12


def test_confidence_monotonic():
    good = confidence(0.1, 0.05, 60, math.radians(10))
    worse = [confidence(1.5, 0.05, 60, math.radians(10)), confidence(0.1, 0.9, 60, math.radians(10)),
             confidence(0.1, 0.05, 15, math.radians(10)), confidence(0.1, 0.05, 60, math.radians(65))]
    assert all(w < good for w in worse) and good > 0.8


def test_depth_refinement_fixes_frontal_tilt():
    R = tilted(0, 0, 0)
    t = np.array([0.03, -0.02, 0.45])
    corners, _ = ArucoBackend('DICT_4X4_50', 'apriltag').detect(render(7, R, t))
    mp = estimate_pose(7, corners[0], SIZE, K, D)
    assert refine_with_depth(mp, depth_of_plane(R, t), K)
    assert math.degrees(math.acos(min(1, (np.trace(mp.R.T @ R) - 1) / 2))) < 0.5
    assert np.linalg.norm(mp.t - t) < 0.0015

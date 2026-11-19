"""Pure-Python parts of the perception pipeline (no ROS graph needed)."""
import math

import cv2
import numpy as np
import pytest

from openarm_sorting_vision import camera_geometry as cg
from openarm_sorting_vision.color_segmentation import (COLOR_SPECS, color_mask, find_blobs,
                                                       flat_hue_ranges, make_spec)


def _scene():
    """Birch-coloured table with a red and a blue 30x30 px square (RGB)."""
    img = np.full((240, 320, 3), (215, 190, 150), np.uint8)      # low saturation table
    img[100:130, 60:90] = (220, 20, 20)                          # red
    img[100:130, 200:230] = (20, 40, 220)                        # blue
    return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


@pytest.mark.parametrize('color,cx', [('red', 74.5), ('blue', 214.5)])
def test_segmentation_finds_each_cube_once(color, cx):
    hsv = _scene()
    blobs = find_blobs(color_mask(hsv, COLOR_SPECS[color]), color, 60, 20000, 0.55)
    assert len(blobs) == 1
    assert blobs[0].area == 900
    assert blobs[0].centroid[0] == pytest.approx(cx, abs=0.5)


def test_table_is_not_segmented():
    hsv = np.full((50, 50, 3), 0, np.uint8)
    hsv[:] = cv2.cvtColor(np.full((1, 1, 3), (215, 190, 150), np.uint8), cv2.COLOR_RGB2HSV)[0, 0]
    for spec in COLOR_SPECS.values():
        assert color_mask(hsv, spec).max() == 0


def test_spec_roundtrip_and_validation():
    for spec in COLOR_SPECS.values():
        assert make_spec(spec.name, flat_hue_ranges(spec), spec.s_min, spec.v_min) == spec
    with pytest.raises(ValueError):
        make_spec('red', [0, 8, 172], 120, 50)


def test_back_projection_optical_frame():
    K = np.array([[320.0, 0, 320], [0, 320.0, 240], [0, 0, 1]])
    pts = cg.back_project(np.array([320.0, 640.0]), np.array([240.0, 240.0]), np.array([1.0, 2.0]), K)
    np.testing.assert_allclose(pts, [[0, 0, 1], [2, 0, 2]])


def test_link_from_optical_matches_urdf_rpy():
    # URDF optical joint rpy = (-pi/2, 0, -pi/2): R = Rz(yaw) Ry(pitch) Rx(roll)
    r, y = -math.pi / 2, -math.pi / 2
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    np.testing.assert_allclose(Rz @ Rx, cg.R_LINK_FROM_OPTICAL, atol=1e-12)


def test_quaternion_helpers():
    q = cg.yaw_to_quaternion(math.pi / 2)
    R = cg.quat_to_matrix(*q)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)

"""HSV colour segmentation of the red and blue cubes (pure OpenCV / NumPy, no ROS).

OpenCV HSV ranges: H in [0, 180), S and V in [0, 255].
Red wraps around H = 0, so it is the union of two hue bands.
The thresholds were checked against the actual Gazebo renders of the cup-sorting world
(see docs/packages/PERCEPTION.md): the birch table (S ~ 50), the white cup
(S < 30) and the grey floor/robot (S ~ 0) are all far below the saturation threshold,
so only the cubes survive.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorSpec:
    name: str
    hue_ranges: tuple          # ((h_lo, h_hi), ...) inclusive, OpenCV hue units
    s_min: int = 120
    v_min: int = 50


COLOR_SPECS = {
    'red': ColorSpec('red', ((0, 8), (172, 179))),
    'blue': ColorSpec('blue', ((100, 128),)),
}


def make_spec(name: str, hue_ranges_flat, s_min: int, v_min: int) -> ColorSpec:
    """ColorSpec from a flat [lo0, hi0, lo1, hi1, ...] hue list (ROS parameters cannot
    hold nested lists)."""
    flat = [int(h) for h in hue_ranges_flat]
    if not flat or len(flat) % 2:
        raise ValueError(f'{name}: hue_ranges needs an even, non-zero number of values, got {flat}')
    pairs = tuple((flat[i], flat[i + 1]) for i in range(0, len(flat), 2))
    return ColorSpec(name, pairs, int(s_min), int(v_min))


def flat_hue_ranges(spec: ColorSpec) -> list:
    return [h for pair in spec.hue_ranges for h in pair]


@dataclass
class Blob:
    color: str
    mask: np.ndarray           # uint8 mask of this blob only (full image size)
    area: int
    centroid: tuple            # (u, v) pixel centroid
    bbox: tuple                # (x, y, w, h)
    fill_ratio: float          # blob area / min-area-rectangle area (1.0 for a perfect square)


def color_mask(hsv: np.ndarray, spec: ColorSpec) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in spec.hue_ranges:
        mask |= cv2.inRange(hsv, (lo, spec.s_min, spec.v_min), (hi, 255, 255))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_blobs(mask: np.ndarray, color: str, min_area: int, max_area: int,
               min_fill_ratio: float) -> list:
    """Connected components that look like a cube face (compact, roughly rectangular)."""
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if not (min_area <= area <= max_area):
            continue
        blob_mask = (labels == i).astype(np.uint8) * 255
        contours, _ = cv2.findContours(blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        (_, _), (rw, rh), _ = cv2.minAreaRect(max(contours, key=cv2.contourArea))
        fill = area / max(rw * rh, 1.0)
        if fill < min_fill_ratio:
            continue
        x, y, w, h = (int(v) for v in stats[i, :4])
        blobs.append(Blob(color, blob_mask, area, (float(cents[i][0]), float(cents[i][1])),
                          (x, y, w, h), float(fill)))
    blobs.sort(key=lambda b: b.area, reverse=True)
    return blobs

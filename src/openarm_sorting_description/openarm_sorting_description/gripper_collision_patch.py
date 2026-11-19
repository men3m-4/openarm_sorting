"""Gazebo-only simplified collision geometry for the OpenArm pinch-gripper fingers.

Why (measured in this project, see docs/packages/ROBOT_DESCRIPTION.md):
  * The cup-sorting world uses the bullet-featherstone physics engine (the only Gazebo
    Harmonic engine that enforces the finger mimic constraint).
  * bullet-featherstone collides MESH shapes as their convex hull inflated by a ~10 mm
    collision margin (a 10 cm mesh box rests 10.0 mm above the table; the same primitive
    box rests exactly on it).
  * The finger mesh is concave and its tip sits only ~9 mm above the table at the grasp pose,
    so the inflated finger hulls pressed on the table and on the cube long before the real
    pads touched it: the gripper could not close around the cube.

What it does:
  Replaces the single <collision> mesh of each finger link (openarm_*_ee_link1/2) with one
  BOX per part of the convex decomposition shipped with openarm_description
  (meshes/collision/<finger>/normal_mesh_5parts/*.stl). Boxes are primitives - no margin.
  Box = axis-aligned bounding box of the part in the link frame (URDF mesh scale applied).
  Part 0 is the fingertip pad, so grasp contacts happen on a flat pad face.

Only the robot_description given to robot_state_publisher -> Gazebo is patched
(robot_description.gazebo_robot_description()). MoveIt keeps the exact meshes (FCL has no
such margin). The upstream OpenArm files are not modified.
"""
import os
import struct
from typing import List, Tuple
import xml.etree.ElementTree as ET

import numpy as np

FINGER_LINK_SUFFIXES = ('ee_link1', 'ee_link2')


def _load_stl(path: str) -> np.ndarray:
    """Vertices (Nx3) of an ASCII or binary STL file."""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        return np.array([[float(v) for v in line.split()[1:]] for line in data.decode().splitlines()
                         if line.strip().startswith('vertex')])
    n = struct.unpack('<I', data[80:84])[0]
    tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return tri['v'].reshape(-1, 3).astype(float)


def _resolve(uri: str, share_dirs: dict) -> str:
    """package:// or file:// mesh URI -> filesystem path."""
    if uri.startswith('package://'):
        pkg, rel = uri[len('package://'):].split('/', 1)
        return os.path.join(share_dirs[pkg], rel)
    if uri.startswith('file://'):
        return uri[len('file://'):]
    return uri


def patch_gripper_collisions(urdf_xml: str, share_dirs: dict,
                             parts_subdir: str = 'normal_mesh_5parts') -> Tuple[str, List[str]]:
    """Return (patched_urdf_xml, report_lines)."""
    root = ET.fromstring(urdf_xml)
    report = []
    for link in root.findall('link'):
        name = link.get('name', '')
        if not name.endswith(FINGER_LINK_SUFFIXES):
            continue
        cols = link.findall('collision')
        if len(cols) != 1 or cols[0].find('geometry/mesh') is None:
            report.append(f'{name}: unexpected collision layout, left unchanged')
            continue
        col = cols[0]
        mesh = col.find('geometry/mesh')
        scale = np.array([float(s) for s in (mesh.get('scale') or '1 1 1').split()])
        origin = col.find('origin')
        oxyz = np.array([float(v) for v in ((origin.get('xyz') if origin is not None else None) or '0 0 0').split()])
        orpy = (origin.get('rpy') if origin is not None else None) or '0 0 0'
        if any(abs(float(v)) > 1e-9 for v in orpy.split()):
            report.append(f'{name}: rotated collision origin not supported, left unchanged')
            continue
        mesh_path = _resolve(mesh.get('filename'), share_dirs)
        stem = os.path.splitext(os.path.basename(mesh_path))[0]
        parts_dir = os.path.join(os.path.dirname(mesh_path), parts_subdir)
        part_files = sorted(f for f in os.listdir(parts_dir) if f.startswith(stem) and f.endswith('.stl'))
        if not part_files:
            report.append(f'{name}: no decomposition found in {parts_dir}, left unchanged')
            continue
        link.remove(col)
        for i, pf in enumerate(part_files):
            v = _load_stl(os.path.join(parts_dir, pf)) * scale
            lo, hi = v.min(axis=0), v.max(axis=0)
            c = ET.SubElement(link, 'collision', {'name': f'{name}_box_{i}'})
            ET.SubElement(c, 'origin', {'xyz': ' '.join(f'{x:.5f}' for x in (lo + hi) / 2 + oxyz), 'rpy': '0 0 0'})
            g = ET.SubElement(c, 'geometry')
            ET.SubElement(g, 'box', {'size': ' '.join(f'{x:.5f}' for x in hi - lo)})
        report.append(f'{name}: mesh collision -> {len(part_files)} boxes ({parts_subdir})')
    return ET.tostring(root, encoding='unicode'), report

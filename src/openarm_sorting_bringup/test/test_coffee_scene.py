"""Consistency of the coffee scenario: world, models, generated geometry file, task config."""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from openarm_sorting_bringup import launch_support
from visual_manipulation_core.se3 import Transform, pose_error

COFFEE = launch_support.share_path('config', 'coffee')


def load(name):
    with open(os.path.join(COFFEE, name)) as f:
        return yaml.safe_load(f)


def geometry():
    return load('coffee_geometry.yaml')['/**']['ros__parameters']


def sdf_pose(text):
    v = [float(x) for x in text.split()]
    return Transform.from_xyz_rpy(v[:3], v[3:])


def marker_visual(model):
    root = ET.parse(launch_support.share_path('models', model, 'model.sdf')).getroot()
    for vis in root.iter('visual'):
        uri = vis.find('geometry/mesh/uri')
        if uri is not None and 'aruco' in uri.text:
            return sdf_pose(vis.find('pose').text), uri.text
    raise AssertionError(f'no ArUco visual in {model}')


def world_includes():
    root = ET.parse(launch_support.share_path('worlds', 'openarm_coffee_world.sdf')).getroot()
    return {i.find('name').text: (i.find('uri').text, sdf_pose(i.find('pose').text)) for i in root.iter('include')}


def test_world_models_installed():
    for name, (uri, _) in world_includes().items():
        model = uri.replace('model://', '')
        assert os.path.isfile(launch_support.share_path('models', model, 'model.sdf')), name


def test_marker_textures_have_unique_names():
    """Gazebo caches meshes / textures by file name: two markers must not share one."""
    uris = [marker_visual(m)[1] for m in ('portafilter', 'espresso_machine')]
    assert len({os.path.basename(u) for u in uris}) == 2


def _homogeneous(node, path=''):
    if isinstance(node, dict):
        for k, v in node.items():
            _homogeneous(v, f'{path}.{k}')
    elif isinstance(node, list):
        assert len({type(v) for v in node}) <= 1, f'{path}: mixed types {node}'


@pytest.mark.parametrize('name', ['coffee_geometry.yaml', 'perception.yaml', 'task.yaml'])
def test_parameter_files_are_valid_ros_parameters(name):
    _homogeneous(load(name))


def test_portafilter_marker_transform_matches_model():
    g = geometry()
    T_P_marker, _ = marker_visual('portafilter')
    T_mk_obj = Transform.from_flat(g['objects']['portafilter']['marker_to_object'])
    d, a = pose_error(T_P_marker @ T_mk_obj, Transform())
    assert d < 1e-3 and a < 1e-3


def test_mount_marker_transform_matches_model():
    """machine model origin = world origin; the mount frame is the world layout used by the
    ground-truth tool (check_portafilter.MOUNT)."""
    from visual_manipulation_gazebo.check_portafilter import MOUNT
    g = geometry()
    T_W_marker, _ = marker_visual('espresso_machine')
    T_W_machine = world_includes()['espresso_machine'][1]
    T_mk_M = Transform.from_flat(g['objects']['machine_mount']['marker_to_object'])
    d, a = pose_error(T_W_machine @ T_W_marker @ T_mk_M, MOUNT)
    assert d < 1e-3 and a < 1e-3


def test_portafilter_starts_in_the_dock():
    from visual_manipulation_gazebo.check_portafilter import DOCK
    inc = world_includes()
    d, a = pose_error(inc['portafilter'][1], DOCK)
    assert d < 0.002 and a < 1e-3
    d, a = pose_error(inc['portafilter_dock'][1], DOCK)
    assert d < 1e-6 and a < 1e-6


def test_marker_sizes_declared_for_the_tracked_objects():
    g = geometry()
    sizes = dict(zip(g['marker_ids'], g['marker_sizes']))
    p = load('perception.yaml')['fiducial_object_tracker']['ros__parameters']
    for name in p['tracked_objects']:
        assert sizes[g['objects'][name]['marker_id']] > 0


def test_grasp_candidates_lie_on_the_handle_grip():
    t = load('task.yaml')['portafilter_task']['ros__parameters']
    for name in t['grasp']['candidates']:
        T = Transform.from_flat(t['grasp'][name]['object_to_tcp'])
        x, y, z = T.t
        assert 0.06 <= x <= 0.19 and abs(y) < 1e-6 and -0.048 <= z <= -0.020, name
        # fingers pitched towards the basket, closing across the handle (tool y = +-P y)
        assert abs(abs(T.R[1, 1]) - 1.0) < 1e-6


def test_task_ready_poses_and_bayonet():
    t = load('task.yaml')['portafilter_task']['ros__parameters']
    assert len(t['motion']['ready_tcp_poses']) % 6 == 0
    b = geometry()['bayonet']
    assert 0 < b['lock_angle_deg'] < 90 and b['insertion_depth'] > 0
    assert b['lip_inner_radius'] < b['channel_radius']


def test_lock_angle_matches_the_machine_notches():
    """The notches in the group-head lip must be where the lugs are when INSERTED."""
    g = geometry()
    lock = math.radians(g['bayonet']['lock_angle_deg'])
    root = ET.parse(launch_support.share_path('models', 'espresso_machine', 'model.sdf')).getroot()
    lip = [sdf_pose(c.find('pose').text) for c in root.iter('collision') if c.get('name', '').startswith('lip_')]
    from visual_manipulation_gazebo.check_portafilter import MOUNT
    angles = []
    for T in lip:
        p = MOUNT.inverse() @ T.t
        angles.append(math.atan2(p[1], p[0]))
    for lug in (math.pi / 2 - lock, -math.pi / 2 - lock):     # lugs at +-y_P, inserted at -lock
        gap = min(abs((a - lug + math.pi) % (2 * math.pi) - math.pi) for a in angles)
        assert gap > math.radians(10), f'no notch at {math.degrees(lug):.0f} deg'
    for lug in (math.pi / 2, -math.pi / 2):                    # locked: lugs over solid lip
        gap = min(abs((a - lug + math.pi) % (2 * math.pi) - math.pi) for a in angles)
        assert gap < math.radians(5), f'lip missing above the locked lug at {math.degrees(lug):.0f} deg'
    assert np.isfinite(lock)

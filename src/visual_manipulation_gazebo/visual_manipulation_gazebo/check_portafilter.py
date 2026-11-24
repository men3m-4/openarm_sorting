"""check_portafilter: Gazebo ground-truth report of the coffee scene (TEST TOOL ONLY).

Prints where the portafilter really is: relative to the group-head mount frame (radial /
axial offset, rotation about the axis, tilt, bayonet state) and relative to the dock pose, and
optionally compares the camera-based estimates of the tracker with ground truth.

  ros2 run visual_manipulation_gazebo check_portafilter [--compare-perception] [--expect locked|dock]

Exit code 0 if the --expect condition holds. The mount / dock reference poses are the generated
world layout (openarm_sorting_bringup/scripts/generate_coffee_models.py), i.e. GROUND TRUTH -
the task software never reads them.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

from visual_manipulation_core.bayonet import BayonetPlanner, BayonetSpec
from visual_manipulation_core.se3 import Transform, pose_error

from .gz_world import GzWorld

MOUNT = Transform.from_xyz_rpy([0.455, 0.08, 0.45], [0, 0, math.pi])        # world layout (ground truth)
DOCK = Transform.from_xyz_rpy([0.44, 0.32, 0.434], [0, 0, math.pi])
BAYONET = BayonetPlanner(BayonetSpec(Transform.translation(0, 0, 0.019), math.radians(45.0), 0.045))


def report(T):
    e = BAYONET.error(MOUNT, T)
    state = BAYONET.classify(MOUNT, T)
    d, a = pose_error(T, DOCK)
    return state, e, d, a


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--world', default='openarm_coffee')
    ap.add_argument('--expect', choices=['locked', 'dock', 'inserted', 'out'])
    ap.add_argument('--compare-perception', action='store_true')
    ap.add_argument('--timeout', type=float, default=10.0)
    a = ap.parse_args(argv)
    w = GzWorld(a.world)
    t0 = time.time()
    while w.pose('portafilter') is None and time.time() - t0 < a.timeout:
        time.sleep(0.1)
    T = w.pose('portafilter')
    if T is None:
        print('portafilter pose not available (is the coffee world running?)')
        return 2
    state, e, d, ang = report(T)
    print(f'portafilter (ground truth) at {np.round(T.t, 4).tolist()} rpy {np.round(np.degrees(T.rpy()), 1).tolist()} deg')
    print(f'  vs group-head LOCKED pose: radial {e["radial"] * 1000:.1f} mm, axial {e["axial"] * 1000:.1f} mm, '
          f'angle {math.degrees(e["angle"]):.1f} deg, tilt {math.degrees(e["tilt"]):.1f} deg -> bayonet state: {state}')
    print(f'  vs dock pose: {d * 1000:.1f} mm, {math.degrees(ang):.1f} deg')
    if a.compare_perception:
        import rclpy
        from visual_manipulation_core.ros_conversions import transform_from_pose
        from visual_manipulation_interfaces.msg import TrackedObjectArray
        rclpy.init()
        n = rclpy.create_node('check_portafilter')
        est = {}
        n.create_subscription(TrackedObjectArray, '/fiducial_object_tracker/objects',
                              lambda m: est.update({o.name: o for o in m.objects}), 10)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        for name, gt in (('portafilter', w.pose('portafilter')), ('machine_mount', MOUNT)):
            o = est.get(name)
            if o is None:
                print(f'  perception: no estimate of {name}')
                continue
            dp, da = pose_error(transform_from_pose(o.pose.pose), gt)
            print(f'  perception {name}: error {dp * 1000:.1f} mm, {math.degrees(da):.2f} deg (stable {o.stable})')
        rclpy.shutdown()
    if a.expect == 'dock':
        return 0 if d < 0.01 else 1
    if a.expect:
        return 0 if state == a.expect else 1
    return 0


def entry():
    """Console entry point. os._exit: the gz-transport and rclpy contexts crash when the
    interpreter tears them down in the wrong order (segfault after a correct run)."""
    code = main()
    sys.stdout.flush()
    os._exit(code)


if __name__ == '__main__':
    entry()

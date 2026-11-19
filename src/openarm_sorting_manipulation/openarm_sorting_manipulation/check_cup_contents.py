"""Report where the cubes really are, from Gazebo ground truth (TEST TOOL ONLY).

  ros2 run openarm_sorting_manipulation check_cup_contents
  ros2 run openarm_sorting_manipulation check_cup_contents --scene-file /path/to/scene.yaml

Exit code 0 if both cubes rest inside the cup, 1 otherwise. The coordinator never uses
ground truth; this tool exists to validate its self-verification.

The cup geometry is read from the same scene file the coordinator uses
(config/cup_sorting_scene.yaml of this package), so the two can never disagree.
"""
import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from openarm_sorting_vision.ground_truth import GroundTruth
import yaml

NODE_KEY = 'cup_sorting_coordinator'


def default_scene_file() -> str:
    return os.path.join(get_package_share_directory('openarm_sorting_manipulation'),
                        'config', 'cup_sorting_scene.yaml')


def load_scene(path: str) -> dict:
    with open(path) as f:
        params = yaml.safe_load(f)[NODE_KEY]['ros__parameters']
    cup = params['cup']
    return {'base': cup['base'], 'inner_radius': cup['inner_radius'], 'height': cup['height'],
            'floor_thickness': cup['floor_thickness'], 'cube_size': params['cube_size']}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--scene-file', default=None, help='scene YAML (default: installed config)')
    ap.add_argument('--world', default='openarm_cup_sorting', help='Gazebo world name')
    args, _ = ap.parse_known_args(argv)        # ignore --ros-args if given
    scene = load_scene(args.scene_file or default_scene_file())
    cup = scene['base']
    inner_r, height = scene['inner_radius'], scene['height']
    floor, cube = scene['floor_thickness'], scene['cube_size']

    gt = GroundTruth(args.world)
    t0 = time.time()
    while time.time() - t0 < 5 and (gt.get('red_cube') is None or gt.get('blue_cube') is None):
        time.sleep(0.1)
    all_in = True
    for name in ('red_cube', 'blue_cube'):
        g = gt.get(name)
        if g is None:
            print(f'{name}: no ground truth received')
            all_in = False
            continue
        p, _yaw = g
        r = math.hypot(p[0] - cup[0], p[1] - cup[1])
        inside = r < inner_r and cup[2] < p[2] < cup[2] + height
        on_floor = abs(p[2] - (cup[2] + floor + cube / 2)) < 0.004
        where = 'INSIDE the cup' + (' (resting on the cup floor)' if on_floor else '') if inside else 'outside the cup'
        print(f'{name}: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  {r * 100:.1f} cm from cup axis  -> {where}')
        all_in &= inside
    sys.exit(0 if all_in else 1)


if __name__ == '__main__':
    main()

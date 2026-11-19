"""Compare the detector output with Gazebo ground truth (TEST TOOL ONLY).

Ground truth = Gazebo's /world/openarm_cup_sorting/dynamic_pose/info, read with the Gazebo
Python bindings (ground_truth.py). The robot logic never uses it; this node only reports
detection errors.

  ros2 run openarm_sorting_vision detection_validator            # runs ~10 s, prints a report
  ros2 run openarm_sorting_vision detection_validator --ros-args -p duration:=30.0
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from .ground_truth import GroundTruth


class DetectionValidator(Node):
    def __init__(self):
        super().__init__('detection_validator')
        self.duration = self.declare_parameter('duration', 10.0).value
        world = self.declare_parameter('world_name', 'openarm_cup_sorting').value
        ns = self.declare_parameter('detection_namespace', '/cup_sorting').value.rstrip('/')
        self.max_error = self.declare_parameter('max_error', 0.010).value
        self.gt = GroundTruth(world)
        self.errors = {'red': [], 'blue': []}
        self.last = {}
        for c in self.errors:
            self.create_subscription(PoseStamped, f'{ns}/{c}_cube/pose',
                                     lambda m, c=c: self.on_det(c, m), 10)

    def on_det(self, color, msg):
        truth = self.gt.get(f'{color}_cube')
        if truth is None:
            return
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        gt, gyaw = truth
        err = np.array([p.x, p.y, p.z]) - gt
        dyaw = (yaw - gyaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
        self.errors[color].append((err, dyaw))
        self.last[color] = (np.array([p.x, p.y, p.z]), gt, yaw, gyaw)

    def report(self):
        ok = True
        for c, errs in self.errors.items():
            if not errs:
                print(f'{c:5s}: NO stable detections received')
                ok = False
                continue
            e = np.array([x[0] for x in errs])
            dy = np.degrees([x[1] for x in errs])
            det, gt, yaw, gyaw = self.last[c]
            norm = np.linalg.norm(e, axis=1)
            print(f'{c:5s}: n={len(errs):3d}  detected=({det[0]:.4f}, {det[1]:.4f}, {det[2]:.4f})  '
                  f'truth=({gt[0]:.4f}, {gt[1]:.4f}, {gt[2]:.4f})')
            print(f'       position error mean=({e[:, 0].mean()*1000:+.1f}, {e[:, 1].mean()*1000:+.1f}, '
                  f'{e[:, 2].mean()*1000:+.1f}) mm   |err| mean={norm.mean()*1000:.1f} mm  max={norm.max()*1000:.1f} mm'
                  f'   yaw error mean={dy.mean():+.1f} deg')
            ok &= norm.max() < self.max_error
        print('RESULT:', f'PASS (all errors < {self.max_error * 1000:.0f} mm)' if ok else 'FAIL')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = DetectionValidator()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < node.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = node.report()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()

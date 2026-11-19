"""Launch readiness gate: block until the given ros2_control controllers are ACTIVE.

Used by cup_sorting.launch.py to start MoveIt, perception and the coordinator only after the
robot is spawned in Gazebo and all its controllers run (event-driven via OnProcessExit on
this process - no fixed sleeps). It only waits for a service and reads the controller list.

  ros2 run openarm_sorting_bringup wait_for_controllers joint_state_broadcaster \\
      left_joint_trajectory_controller --timeout 240

Exit codes: 0 all active, 1 timeout, 130 interrupted.
"""
import argparse
import sys
import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def main(argv=None) -> int:
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    ap = argparse.ArgumentParser(description='Wait until ros2_control controllers are active.')
    ap.add_argument('controllers', nargs='+')
    ap.add_argument('--controller-manager', default='/controller_manager')
    ap.add_argument('--timeout', type=float, default=240.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node('wait_for_controllers')
    log = node.get_logger()
    client = node.create_client(ListControllers, f'{args.controller_manager}/list_controllers')
    t0 = time.monotonic()
    last = None
    code = 1
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.timeout:
            if not client.wait_for_service(timeout_sec=1.0):
                missing = [f'{args.controller_manager} services']
            else:
                fut = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                states = {c.name: c.state for c in fut.result().controller} if fut.result() else {}
                missing = [f'{c} ({states.get(c, "not loaded")})' for c in args.controllers
                           if states.get(c) != 'active']
                if not missing:
                    log.info(f'all {len(args.controllers)} controllers active after '
                             f'{time.monotonic() - t0:.0f} s')
                    code = 0
                    break
                time.sleep(0.5)       # poll interval of the controller list (not a startup delay)
            msg = ', '.join(missing)
            if msg != last:
                log.info(f'waiting for: {msg}')
                last = msg
        else:
            log.error(f'controllers not active after {args.timeout:.0f} s, still missing: {last}')
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Integration test of the INDIVIDUAL subsystem launch files (as a developer would use them).

Starts, one after the other (each its own `ros2 launch`, like separate terminals):
  simulation.launch.py gz_gui:=false   -> waits with wait_for_controllers
  moveit.launch.py, perception.launch.py, coordinator.launch.py prompt:=false
  rviz.launch.py rviz_config:=camera_view.rviz   (only with --rviz; needs a display)
then checks, WITHOUT moving the robot:
  /cup_sorting/ready succeeds, both cube poses are published,
  /cup_sorting/plan_check (IK + OMPL plan to the red pre-grasp, not executed) succeeds,
  the left arm did not move,
and finally stops every launch with SIGINT (reverse order) and checks nothing is left.

  python3 tests/integration/subsystem_launch_test.py [--rviz] [--log-dir DIR]
Exit code 0 = all checks passed.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

PATTERNS = ('gz sim', 'move_group', 'cube_detector', 'cup_sorting_coordinator', 'parameter_bridge',
            'robot_state_publisher', 'rviz2')


def start(name, args, log_dir, env):
    """Start `ros2 launch` with SIGINT at its default disposition (like a terminal)."""
    out = open(os.path.join(log_dir, f'{name}.log'), 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'openarm_sorting_bringup'] + args, stdout=out,
                            stderr=subprocess.STDOUT, env=env, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    print(f'  started {" ".join(args)} (pid {proc.pid})', flush=True)
    return proc


def run(cmd, timeout=60):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def trigger(service, timeout=120):
    out = run(['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger'], timeout).stdout
    m = re.search(r"success=(True|False), message='(.*)'\)", out, re.S)
    return (m.group(1) == 'True', m.group(2)) if m else (False, out.strip()[-200:])


def left_arm():
    out = run(['ros2', 'topic', 'echo', '--once', '/joint_states'], 20).stdout
    names = [n[2:] for n in re.search(r'name:\n((?:- .*\n)+)', out).group(1).splitlines()]
    pos = [float(v) for v in re.search(r'position:\n((?:- .*\n)+)', out).group(1).replace('- ', '').split()]
    return {n: p for n, p in zip(names, pos) if n.startswith('openarm_left_joint')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rviz', action='store_true')
    ap.add_argument('--log-dir', default=os.path.join(os.getcwd(), 'subsystem_logs'))
    args = ap.parse_args()
    log_dir = os.path.join(args.log_dir, time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(log_dir, exist_ok=True)
    env = dict(os.environ, ROS_LOG_DIR=os.path.join(log_dir, 'ros'))
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' - {detail}' if detail else ''), flush=True)

    procs = []
    try:
        procs.append(start('simulation', ['simulation.launch.py', 'gz_gui:=false'], log_dir, env))
        gate = run(['ros2', 'run', 'openarm_sorting_bringup', 'wait_for_controllers', 'joint_state_broadcaster',
                    'left_joint_trajectory_controller', 'right_joint_trajectory_controller',
                    'left_gripper_controller', 'right_gripper_controller', '--timeout', '180'], 200)
        check('simulation.launch.py: all 5 controllers active', gate.returncode == 0)
        procs.append(start('moveit', ['moveit.launch.py'], log_dir, env))
        procs.append(start('perception', ['perception.launch.py', 'output:=log'], log_dir, env))
        procs.append(start('coordinator', ['coordinator.launch.py', 'prompt:=false'], log_dir, env))
        if args.rviz:
            procs.append(start('rviz', ['rviz.launch.py', 'rviz_config:=camera_view.rviz'], log_dir, env))

        t0 = time.monotonic()
        ok, msg = False, ''
        while time.monotonic() - t0 < 180 and not ok:
            ok, msg = trigger('/cup_sorting/ready', 30)
            if not ok:
                time.sleep(2)
        check('coordinator.launch.py: /cup_sorting/ready', ok, f'{time.monotonic() - t0:.0f} s, {msg}')
        for c in ('red', 'blue'):
            out = run(['ros2', 'topic', 'echo', '--once', f'/cup_sorting/{c}_cube/pose'], 30).stdout
            check(f'perception.launch.py: {c} cube pose published', 'position' in out)
        q0 = left_arm()
        ok, msg = trigger('/cup_sorting/plan_check', 180)
        check('moveit.launch.py: plan_check (IK + OMPL, not executed)', ok, msg[:160])
        q1 = left_arm()
        drift = max(abs(q1[j] - q0[j]) for j in q0)
        check('left arm did not move during plan_check', drift < 0.01, f'{drift:.4f} rad')
        if args.rviz:
            check('rviz.launch.py (camera_view.rviz) running', procs[-1].poll() is None)
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(60)
                except subprocess.TimeoutExpired:
                    p.kill()
        time.sleep(3)
        ps = run(['ps', '-eo', 'args']).stdout.splitlines()
        left = [line for line in ps if any(x in line for x in PATTERNS) and 'subsystem_launch_test' not in line]
        check('all launches stopped with SIGINT, nothing left', not left, '; '.join(left)[:200])
    ok = all(results)
    print(f'RESULT: {"PASS" if ok else "FAIL"}  (logs: {log_dir})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

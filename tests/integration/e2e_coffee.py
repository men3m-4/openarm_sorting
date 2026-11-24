#!/usr/bin/env python3
"""End-to-end test of the coffee / portafilter task (headless Gazebo, real MoveIt, perception).

  python3 tests/integration/e2e_coffee.py [--runs N] [--no-grasp-assist] [--rgb-only]
                                          [--mode run|install-remove] [--log-dir DIR]

Starts coffee_portafilter.launch.py (gz_gui:=false rviz:=false prompt:=false), waits for
/portafilter_task/ready, then for every run:
  run            : /portafilter_task/run, expects success and the portafilter back in its stand
                   (Gazebo ground truth, check_portafilter --expect dock)
  install-remove : /portafilter_task/install (expects LOCKED by ground truth), then
                   /portafilter_task/remove (expects back in the stand)
Before the first run: /portafilter_task/plan_check must succeed (nothing moves).
Finally SIGINT to the launch and a check that no simulation process is left.
Exit code 0 = every check passed. Result summary in <log-dir>/result.txt.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

PATTERNS = ('gz sim', 'move_group', 'aruco_detector', 'fiducial_object_tracker', 'portafilter_task',
            'parameter_bridge', 'robot_state_publisher', 'grasp_assist')


def run(cmd, timeout=60):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def trigger(service, timeout=900):
    out = run(['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger'], timeout).stdout
    m = re.search(r"success=(True|False), message='(.*)'\)", out, re.S)
    return (m.group(1) == 'True', m.group(2)) if m else (False, out.strip()[-300:])


def ground_truth(expect):
    r = run(['ros2', 'run', 'visual_manipulation_gazebo', 'check_portafilter', '--expect', expect], 60)
    lines = [ln for ln in r.stdout.splitlines() if 'vs ' in ln]
    return r.returncode == 0, ' / '.join(ln.strip() for ln in lines)


def leftovers():
    """Simulation processes still running (shell wrappers whose command TEXT merely contains a
    pattern, e.g. an interactive `grep`, are ignored)."""
    out = run(['ps', '-eo', 'cmd']).stdout.splitlines()
    return [ln for ln in out if any(p in ln for p in PATTERNS) and 'e2e_coffee' not in ln
            and not ln.startswith(('/bin/bash', 'bash ', 'grep'))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=1)
    ap.add_argument('--mode', choices=['run', 'install-remove'], default='run')
    ap.add_argument('--no-grasp-assist', action='store_true')
    ap.add_argument('--rgb-only', action='store_true')
    ap.add_argument('--log-dir', default=os.path.join(os.getcwd(), 'e2e_coffee_logs'))
    a = ap.parse_args()
    os.makedirs(a.log_dir, exist_ok=True)
    if leftovers():
        print('another simulation is running - stop it first:\n  ' + '\n  '.join(leftovers()))
        return 2
    args = ['gz_gui:=false', 'rviz:=false', 'prompt:=false', f'grasp_assist:={"false" if a.no_grasp_assist else "true"}']
    if a.rgb_only:
        prefix = run(['ros2', 'pkg', 'prefix', 'openarm_sorting_bringup']).stdout.strip()
        args.append(f'perception_params:={prefix}/share/openarm_sorting_bringup/config/coffee/perception_rgb_only.yaml')
    log = open(os.path.join(a.log_dir, 'launch.log'), 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'openarm_sorting_bringup', 'coffee_portafilter.launch.py'] + args,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    results = []

    def check(name, ok, detail=''):
        results.append((name, ok, detail))
        print(f'[{"PASS" if ok else "FAIL"}] {name} {detail}', flush=True)
        return ok

    try:
        t0 = time.time()
        ready = False
        while time.time() - t0 < 240 and not ready:
            ready, msg = trigger('/portafilter_task/ready', 30)
            time.sleep(2)
        check('system ready', ready, f'after {time.time() - t0:.0f} s')
        if ready:
            ok, msg = trigger('/portafilter_task/plan_check', 180)
            check('plan_check (no motion)', ok, msg[:160])
            for i in range(1, a.runs + 1):
                if a.mode == 'run':
                    t = time.time()
                    ok, msg = trigger('/portafilter_task/run')
                    check(f'run {i}: task', ok, f'({time.time() - t:.0f} s) {msg[:220]}')
                    gt, d = ground_truth('dock')
                    check(f'run {i}: portafilter back in the stand (ground truth)', gt, d)
                else:
                    ok, msg = trigger('/portafilter_task/install')
                    check(f'run {i}: install', ok, msg[:200])
                    gt, d = ground_truth('locked')
                    check(f'run {i}: LOCKED in the group head (ground truth)', gt, d)
                    ok, msg = trigger('/portafilter_task/remove')
                    check(f'run {i}: remove', ok, msg[:200])
                    gt, d = ground_truth('dock')
                    check(f'run {i}: back in the stand (ground truth)', gt, d)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(60)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        time.sleep(3)
        check('nothing left running after SIGINT', not leftovers(), '; '.join(leftovers())[:200])
    ok = all(r[1] for r in results)
    with open(os.path.join(a.log_dir, 'result.txt'), 'w') as f:
        for name, r, d in results:
            f.write(f'{"PASS" if r else "FAIL"} {name} {d}\n')
        f.write(f'RESULT: {"PASS" if ok else "FAIL"}\n')
    print(f'RESULT: {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

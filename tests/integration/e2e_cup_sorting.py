#!/usr/bin/env python3
"""End-to-end system test of the complete cup-sorting application (real Gazebo + MoveIt).

It starts   ros2 launch openarm_sorting_bringup cup_sorting.launch.py   in a pseudo-terminal
(exactly like a user in a terminal), waits for the operator prompt, answers it, verifies the
outcome and finally stops the launch with Ctrl+C and checks that nothing is left running.

  # workspace built and sourced first:
  python3 tests/integration/e2e_cup_sorting.py --answer n            # NO path
  python3 tests/integration/e2e_cup_sorting.py --answer y            # YES path (full sort, ~2 min)
  python3 tests/integration/e2e_cup_sorting.py --answer y --gui      # with Gazebo GUI + RViz

Checks
  always : invalid input is rejected and the question is asked again; Ctrl+C stops everything
           (no Gazebo / ROS processes of the run left)
  --answer n : "NOT executed" message, the coordinator never starts a sequence, the left arm
               does not move, both cubes stay at their spawn poses (Gazebo ground truth)
  --answer y : "TASK FINISHED SUCCESSFULLY", both cubes inside the cup (Gazebo ground truth,
               check_cup_contents)

Exit code 0 = all checks passed. The full terminal transcript is written to --log-dir.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

import pexpect

QUESTION = 'Do you want to execute the sorting task? [y/N]:'
BANNER = 'OpenArm Cup Sorting System'
SEQUENCE_MARKERS = ('[red] 0. detected', 'starting the cup-sorting task')
SPAWN = {'red_cube': (0.30, 0.04, 0.32), 'blue_cube': (0.30, 0.16, 0.32)}
PROCESS_PATTERNS = ('gz sim', 'move_group', 'cube_detector', 'cup_sorting_coordinator',
                    'parameter_bridge', 'robot_state_publisher', 'task_prompt', 'rviz2')


class Result:
    def __init__(self):
        self.checks = []

    def check(self, name, ok, detail=''):
        self.checks.append((name, bool(ok), detail))
        print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' - {detail}' if detail else ''), flush=True)
        return ok

    @property
    def ok(self):
        return all(c[1] for c in self.checks)


def left_arm_positions():
    out = subprocess.run(['ros2', 'topic', 'echo', '--once', '/joint_states'], capture_output=True,
                         text=True, timeout=20).stdout
    names = re.search(r'name:\n((?:- .*\n)+)', out).group(1).split('\n')
    names = [n[2:] for n in names if n]
    pos = [float(v) for v in re.search(r'position:\n((?:- .*\n)+)', out).group(1).replace('- ', '').split()]
    return {n: p for n, p in zip(names, pos) if n.startswith('openarm_left_joint')}


def ground_truth():
    p = subprocess.run(['ros2', 'run', 'openarm_sorting_manipulation', 'check_cup_contents'],
                       capture_output=True, text=True, timeout=60)
    cubes = {}
    for m in re.finditer(r'(\w+_cube): \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)', p.stdout):
        cubes[m.group(1)] = tuple(float(m.group(i)) for i in (2, 3, 4))
    return p.returncode, p.stdout.strip(), cubes


def leftover_processes(launch_pid):
    out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
    me = {str(os.getpid())}
    return [line.strip() for line in out.splitlines()[1:]
            if any(p in line for p in PROCESS_PATTERNS) and line.split()[0] not in me
            and 'e2e_cup_sorting' not in line and 'ps -eo' not in line]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--answer', choices=['y', 'n'], required=True)
    ap.add_argument('--gui', action='store_true', help='Gazebo GUI + RViz (default: headless)')
    ap.add_argument('--log-dir', default=os.path.join(os.getcwd(), 'e2e_logs'))
    ap.add_argument('--ready-timeout', type=float, default=420)
    ap.add_argument('--task-timeout', type=float, default=600)
    args = ap.parse_args()

    run = time.strftime('%Y%m%d_%H%M%S') + f'_answer_{args.answer}' + ('_gui' if args.gui else '_headless')
    log_dir = os.path.join(args.log_dir, run)
    os.makedirs(log_dir, exist_ok=True)
    env = dict(os.environ, ROS_LOG_DIR=os.path.join(log_dir, 'ros'))
    launch_args = [] if args.gui else ['gz_gui:=false', 'rviz:=false']
    cmd = 'ros2 launch openarm_sorting_bringup cup_sorting.launch.py ' + ' '.join(launch_args)
    print(f'== {run}\n   {cmd}\n   logs: {log_dir}', flush=True)

    res = Result()
    transcript = open(os.path.join(log_dir, 'terminal.log'), 'w')
    t0 = time.monotonic()
    child = pexpect.spawn('/bin/bash', ['-c', cmd], env=env, encoding='utf-8', timeout=args.ready_timeout,
                          dimensions=(50, 200))
    child.logfile_read = transcript
    try:
        idx = child.expect([re.escape(QUESTION), pexpect.EOF, pexpect.TIMEOUT])
        t_ready = time.monotonic() - t0
        if not res.check('prompt shown after the system became ready', idx == 0,
                         f'{t_ready:.0f} s after launch' if idx == 0 else 'no prompt'):
            return finish(res, child, transcript, log_dir)
        res.check('banner printed', BANNER in child.before)
        res.check('no manipulation before confirmation',
                  not any(m in child.before for m in SEQUENCE_MARKERS))

        # invalid input -> asked again
        child.sendline('maybe')
        idx = child.expect([re.escape(QUESTION), pexpect.TIMEOUT], timeout=10)
        res.check('invalid answer rejected and question repeated',
                  idx == 0 and 'Please answer' in child.before)

        q_before = left_arm_positions()
        gt_rc, gt_text, cubes_before = ground_truth()
        child.sendline(args.answer)
        if args.answer == 'n':
            idx = child.expect(['NOT executed', pexpect.TIMEOUT], timeout=15)
            res.check('"n" acknowledged: task NOT executed', idx == 0)
            # keep observing: nothing may move
            child.expect([pexpect.TIMEOUT], timeout=20)
            seen = child.before
            res.check('coordinator did not start a sequence', not any(m in seen for m in SEQUENCE_MARKERS))
            q_after = left_arm_positions()
            drift = max(abs(q_after[j] - q_before[j]) for j in q_before)
            res.check('left arm did not move', drift < 0.01, f'max joint change {drift:.4f} rad')
            _rc, gt_text, cubes = ground_truth()
            moved = max(max(abs(a - b) for a, b in zip(cubes[c], SPAWN[c])) for c in SPAWN) if len(cubes) == 2 else 1
            res.check('cubes still at their spawn poses (ground truth)', moved < 0.01, f'max offset {moved * 1000:.1f} mm')
            res.check('simulation still running after "n"', child.isalive())
        else:
            idx = child.expect(['TASK FINISHED SUCCESSFULLY', 'TASK DID NOT COMPLETE', pexpect.TIMEOUT],
                               timeout=args.task_timeout)
            t_task = time.monotonic() - t0 - t_ready
            res.check('"y": sorting task executed and reported success', idx == 0,
                      f'{t_task:.0f} s after the answer')
            text = child.before
            res.check('red cube handled before blue cube',
                      0 <= text.find('[red] 0. detected') < text.find('[blue] 0. detected'))
            for c in ('red', 'blue'):
                m = re.search(rf'\[{c}\] 5\. GRASP VERIFIED \(finger stopped at ([\d.]+)', text)
                res.check(f'{c}: grasp verified by finger angle', m, m.group(1) + ' rad' if m else '')
                res.check(f'{c}: placement verified by camera', f'[{c}] 11. PLACEMENT VERIFIED' in text)
            rc, gt_text, _ = ground_truth()
            res.check('both cubes inside the cup (Gazebo ground truth)', rc == 0, gt_text.replace('\n', ' | '))
            res.check('simulation still running after the task', child.isalive())
    except Exception as exc:  # noqa: BLE001
        res.check('test ran without exceptions', False, repr(exc))
    return finish(res, child, transcript, log_dir)


def finish(res, child, transcript, log_dir):
    # Ctrl+C like a user, then make sure nothing of the run is left
    if child.isalive():
        child.sendintr()
        try:
            child.expect(pexpect.EOF, timeout=60)
        except pexpect.TIMEOUT:
            pass
    res.check('launch stopped after Ctrl+C', not child.isalive())
    if child.isalive():
        child.kill(signal.SIGKILL)
    time.sleep(3)
    left = leftover_processes(child.pid)
    res.check('no Gazebo / ROS processes left after Ctrl+C', not left, '; '.join(left)[:300])
    transcript.close()
    summary = os.path.join(log_dir, 'result.txt')
    with open(summary, 'w') as f:
        for name, ok, detail in res.checks:
            f.write(f'[{"PASS" if ok else "FAIL"}] {name}' + (f' - {detail}' if detail else '') + '\n')
        f.write(f'RESULT: {"PASS" if res.ok else "FAIL"}\n')
    print(f'RESULT: {"PASS" if res.ok else "FAIL"}  ({summary})', flush=True)
    return 0 if res.ok else 1


if __name__ == '__main__':
    sys.exit(main())

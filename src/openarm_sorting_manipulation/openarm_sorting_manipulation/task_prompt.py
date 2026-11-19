"""Operator confirmation for the cup-sorting task (started by cup_sorting.launch.py).

Why this node exists
--------------------
The task must never start without an explicit "yes" from the operator, but everything else
(Gazebo, controllers, MoveIt, perception, coordinator) is started by ONE launch command.
This small node is the bridge between the two:

  1. it waits - with real readiness checks, no fixed sleeps - until every component the task
     needs is available (the coordinator's own readiness check via /cup_sorting/ready, plus
     fresh camera detections of both cubes);
  2. it asks on the terminal:   Do you want to execute the sorting task? [y/N]:
       y / Y          -> calls /cup_sorting/run (std_srvs/Trigger: red cube, then blue cube)
                         and prints the result;
       n / N / Enter  -> does nothing; the simulation keeps running;
       anything else  -> asks again;
  3. it exits. Its exit never stops the simulation (Ctrl+C in the launch terminal does).

The node only observes the system and calls the coordinator's service; all task logic stays
in the coordinator.

Terminal handling: `ros2 launch` connects the stdin/stdout of its child processes to pipes
(stdout is printed line by line, so a question without a trailing newline would not appear),
but the children stay in the terminal's session. The question is therefore written to and
the answer read from the controlling terminal /dev/tty directly. Without a terminal (e.g. a
non-interactive launch) the task is NOT started; use auto_start:=true or call the service.

Output: the node is started with --disable-stdout-logs. Its ROS log messages go to the log
file and /rosout; everything meant for the operator (progress, the question, the result) is
written to the terminal directly, so it appears in order and the question stays on its own
line (console logs travel through launch's pipes and would arrive late).

Readiness conditions:
  - coordinator:   /cup_sorting/ready returns success = the coordinator's own readiness check
                   passed (move_group interfaces, all controllers active, left gripper action +
                   finger joint state, joint limits from /robot_description) and it is idle
  - perception:    <detection_namespace>/{red,blue}_cube/pose received within detection_max_age
"""
import sys
import time
from typing import List, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

BANNER = """
========================================
 OpenArm Cup Sorting System
========================================

All required components are ready.
"""
QUESTION = 'Do you want to execute the sorting task? [y/N]: '
COLORS = ('red', 'blue')


def parse_answer(text: Optional[str]) -> Optional[bool]:
    """True = execute, False = do not execute, None = invalid (ask again).

    EOF (text is None) counts as "no": the task never starts without an explicit yes.
    """
    if text is None:
        return False
    answer = text.strip()
    if answer in ('y', 'Y'):
        return True
    if answer in ('', 'n', 'N'):
        return False
    return None


class TaskPrompt(Node):

    def __init__(self):
        super().__init__('cup_sorting_task_prompt')
        P = self.declare_parameter
        self.ready_timeout = float(P('ready_timeout', 300.0).value)
        self.task_timeout = float(P('task_timeout', 900.0).value)
        self.detection_max_age = float(P('detection_max_age', 2.0).value)
        self.run_service = P('run_service', '/cup_sorting/run').value
        self.ready_service = P('ready_service', '/cup_sorting/ready').value
        det_ns = P('detection_namespace', '/cup_sorting').value.rstrip('/')

        self.run_client = self.create_client(Trigger, self.run_service)
        self.ready_client = self.create_client(Trigger, self.ready_service)
        self.tty = None             # Terminal for operator output (set by main)
        self.det_time = {}
        for c in COLORS:
            self.create_subscription(PoseStamped, f'{det_ns}/{c}_cube/pose',
                                     lambda _m, c=c: self.det_time.__setitem__(c, time.monotonic()), 10)

    def say(self, msg: str, level: str = 'info') -> None:
        """Log `msg` (file + /rosout) and show it to the operator on the terminal."""
        getattr(self.get_logger(), level)(msg)
        out = self.tty if self.tty is not None else sys.stderr
        out.write(f'[task_prompt] {"ERROR: " if level == "error" else ""}{msg}\n')
        out.flush()

    # ------------------------------------------------------------------ readiness
    def _coordinator_status(self) -> Optional[str]:
        """None if the coordinator reports ready, else what it is waiting for."""
        if not self.ready_client.service_is_ready():
            return f'coordinator ({self.ready_service})'
        fut = self.ready_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or fut.result() is None:
            return 'coordinator status (no answer)'
        res = fut.result()
        return None if res.success else f'coordinator ({res.message})'

    def missing(self) -> List[str]:
        """Names of the conditions that are not met yet (empty list = ready)."""
        out = []
        status = self._coordinator_status()
        if status:
            out.append(status)
        now = time.monotonic()
        out += [f'{c} cube detection' for c in COLORS
                if now - self.det_time.get(c, -1e9) > self.detection_max_age]
        return out

    def wait_until_ready(self) -> bool:
        t0 = time.monotonic()
        last = None
        while rclpy.ok() and time.monotonic() - t0 < self.ready_timeout:
            end = time.monotonic() + 1.0          # let discovery / subscriptions run
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            miss = self.missing()
            if not miss:
                self.say(f'READY after {time.monotonic() - t0:.0f} s: coordinator ready (MoveIt, '
                         'controllers, left gripper, joint limits), red + blue cube detected')
                return True
            msg = ', '.join(miss)
            if msg != last:
                self.say(f'waiting for: {msg}')
                last = msg
        self.say(f'system not ready after {self.ready_timeout:.0f} s, still missing: {last}', 'error')
        return False

    # ------------------------------------------------------------------ task
    def run_task(self) -> bool:
        self.say(f'starting the cup-sorting task (red cube, then blue cube) via {self.run_service}')
        fut = self.run_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.task_timeout)
        if not fut.done() or fut.result() is None:
            self.say(f'no answer from {self.run_service} within {self.task_timeout:.0f} s', 'error')
            return False
        res = fut.result()
        if res.success:
            self.say(f'coordinator result: {res.message}')
        else:
            self.say(f'coordinator result: {res.message}', 'error')
        return bool(res.success)


class Terminal:
    """The controlling terminal (/dev/tty), opened as separate read and write streams
    (a tty is not seekable, so a single text-mode 'r+' stream cannot be used)."""

    def __init__(self, path: str = '/dev/tty'):
        self._out = open(path, 'w', buffering=1)
        try:
            self._in = open(path, 'r')
        except OSError:
            self._out.close()
            raise

    def write(self, text: str) -> None:
        self._out.write(text)

    def flush(self) -> None:
        self._out.flush()

    def readline(self) -> str:
        return self._in.readline()

    def close(self) -> None:
        self._in.close()
        self._out.close()


def open_terminal() -> Optional[Terminal]:
    try:
        return Terminal()
    except OSError:          # no controlling terminal (non-interactive launch)
        return None


def ask(tty: Terminal) -> bool:
    """Ask until the answer is valid. Returns True only for an explicit y/Y."""
    while True:
        tty.write(QUESTION)
        tty.flush()
        line = tty.readline()
        decision = parse_answer(line if line else None)
        if decision is not None:
            return decision
        tty.write("Please answer 'y' (execute) or 'n' (do not execute).\n")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = TaskPrompt()
    code = 0
    tty = None
    try:
        tty = node.tty = open_terminal()
        if tty is None:
            node.say('no interactive terminal to ask for confirmation - the task will NOT be started. '
                     f'Start it with  ros2 service call {node.run_service} std_srvs/srv/Trigger  '
                     'or launch with auto_start:=true.', 'error')
            code = 1
            return code
        node.say('waiting until the system is ready (MoveIt, controllers, gripper, coordinator, '
                 'cube detections)...')
        if not node.wait_until_ready():
            node.say('the task was NOT started. The simulation keeps running; check the log, then start '
                     f'it manually:  ros2 service call {node.run_service} std_srvs/srv/Trigger', 'error')
            code = 1
            return code
        tty.write(BANNER + '\n')
        if ask(tty):
            tty.write('\nExecuting the sorting task (red cube, then blue cube)...\n\n')
            tty.flush()
            ok = node.run_task()
            tty.write('\n' + ('TASK FINISHED SUCCESSFULLY.' if ok else
                              'TASK DID NOT COMPLETE - see the coordinator messages above.') + '\n')
            code = 0 if ok else 1
        else:
            tty.write('\nThe sorting task was NOT executed. The simulation keeps running.\n'
                      f'Run it later with:  ros2 service call {node.run_service} std_srvs/srv/Trigger\n')
        tty.write('Press Ctrl+C to stop the simulation.\n')
        tty.flush()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.tty = None
        if tty is not None:
            tty.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())

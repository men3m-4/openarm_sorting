"""Generic operator confirmation for a task coordinator (same behaviour as the cup-sorting
prompt of openarm_sorting_manipulation, with the service names / texts as parameters):

  1. wait until `ready_service` (std_srvs/Trigger) reports success (real readiness check);
  2. ask on the controlling terminal:  <question> [y/N]
       y / Y -> call `run_service` (std_srvs/Trigger) and print the result
       n / N / Enter / EOF -> nothing is executed (the system keeps running)
       anything else -> ask again
  3. exit (never stops the rest of the system).

The question / answer use /dev/tty directly because `ros2 launch` pipes the stdout of its
children line by line. Without a terminal the task is NOT started.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def parse_answer(text):
    """True = execute, False = do not execute, None = invalid (ask again). EOF = no."""
    if text is None:
        return False
    a = text.strip()
    if a in ('y', 'Y'):
        return True
    if a in ('', 'n', 'N'):
        return False
    return None


class TaskPrompt(Node):
    def __init__(self):
        super().__init__('task_prompt')
        P = self.declare_parameter
        self.ready_srv = P('ready_service', '/portafilter_task/ready').value
        self.run_srv = P('run_service', '/portafilter_task/run').value
        self.title = P('title', 'Portafilter task').value
        self.question = P('question', 'Do you want to execute the portafilter task (install + remove)? [y/N]: ').value
        self.ready_timeout = P('ready_timeout', 600.0).value
        self.task_timeout = P('task_timeout', 1800.0).value
        self.ready_cli = self.create_client(Trigger, self.ready_srv)
        self.run_cli = self.create_client(Trigger, self.run_srv)

    def call(self, cli, timeout):
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def wait_ready(self, out):
        t0, last = time.monotonic(), None
        while rclpy.ok() and time.monotonic() - t0 < self.ready_timeout:
            r = self.call(self.ready_cli, 5.0)
            msg = r.message if r is not None else f'{self.ready_srv} not available'
            if r is not None and r.success:
                return True
            if msg != last:
                out.write(f'[task_prompt] waiting for: {msg}\n')
                out.flush()
                last = msg
            time.sleep(1.0)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TaskPrompt()
    try:
        tty = open('/dev/tty', 'r+')
    except OSError:
        node.get_logger().error('no interactive terminal: the task is NOT started '
                                '(use auto_start:=true or call the run service)')
        tty = None
    out = tty or sys.stdout
    try:
        if tty is not None and node.wait_ready(out):
            bar = '=' * max(40, len(node.title) + 2)
            out.write(f'\n{bar}\n {node.title}\n{bar}\n\nAll required components are ready.\n\n')
            while True:
                out.write(node.question)
                out.flush()
                line = tty.readline()
                ans = parse_answer(line if line else None)
                if ans is None:
                    out.write('Please answer y or n.\n')
                    continue
                if not ans:
                    out.write(f'NOT executed. Run it later with:  ros2 service call {node.run_srv} '
                              'std_srvs/srv/Trigger\n')
                    break
                out.write('Executing ... (progress in the coordinator log)\n')
                out.flush()
                r = node.call(node.run_cli, node.task_timeout)
                out.write(('TASK FINISHED SUCCESSFULLY.\n' if r and r.success else 'TASK FAILED.\n')
                          + (r.message if r else 'no response') + '\n')
                break
            out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
